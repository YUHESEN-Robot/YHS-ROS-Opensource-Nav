/**
 * @file      CameraPublisher.cpp
 * @brief     angstrong camera publisher.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */

#include "CameraPublisher.h"

#define LOG_PARA(value, str)\
    do { \
        if (value != -1) { \
            LOG(INFO) << "get " << str << " " << value << std::endl; \
        } \
    } while(0)


CameraPublisher::CameraPublisher()
{
    nh = new ros::NodeHandle("~");
    initLaunchParams();
}

CameraPublisher::~CameraPublisher()
{
    if (nh != nullptr) {
        delete nh;
        nh = nullptr;
    }
}

int CameraPublisher::start()
{
    int ret = 0;

    std::string config_path;
    nh->param<std::string>("confiPath", config_path, "");
    if (config_path.size() == 0) {
        LOG(ERROR) << "config file path error" << std::endl;
        return -1;
    }

    if (server == nullptr) {
        server = new CameraSrv(this, config_path);
        ret = server->start();
        if (ret != 0) {
            LOG(ERROR) << "start server failed" << std::endl;
        }
        m_monitor_flg = true;
        m_monitor_thd = std::thread(&CameraPublisher::streamController, this);
    }

    return ret;
}

void CameraPublisher::stop()
{
    if (server != nullptr) {
        server->stop();
        m_monitor_flg = false;
        if (m_monitor_thd.joinable()) {
            m_monitor_thd.join();
        }
        delete server;
        server = nullptr;
    }

    /* free the map */
    m_camera_map.erase(m_camera_map.begin(), m_camera_map.end());
}

void CameraPublisher::saveImage()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        it->second->enableSaveImage(true);
    }
}

void CameraPublisher::logFps(bool enable)
{
    m_logfps = enable;
}

bool CameraPublisher::getLogFps()
{
    return m_logfps;
}

int CameraPublisher::onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                      const AS_SDK_CAM_MODEL_E &cam_type)
{
    int ret = 0;
    PUBLISHER_INFO_S stPublisherInfo;
    AS_CAM_ATTR_S attr_t;
    bool reconnected = false;
    unsigned int dev_idx = imgPubList.size();
    // m_cam_attached_cnt++;

    param.open = true;
    memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
    ret = AS_SDK_GetCameraAttrs(pCamera, attr_t);
    if (ret != 0) {
        LOG(ERROR) << "get device path info failed" << std::endl;
    }

    /* open the specified camera */
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        if ((m_launch_param.usb_bus_num != -1) && (strcmp(m_launch_param.usb_port_nums, "null") != 0)) {
            if ((attr_t.attr.usbAttrs.bnum != m_launch_param.usb_bus_num) ||
                (std::string(attr_t.attr.usbAttrs.port_numbers) != m_launch_param.usb_port_nums)) {
                param.open = false;
                LOG(WARN) << "found a dev but cannot match the specified camera, launch bus num " << m_launch_param.usb_bus_num <<
                          ", port num " << m_launch_param.usb_port_nums << ", but found bus num " << attr_t.attr.usbAttrs.bnum << ", port num " <<
                          attr_t.attr.usbAttrs.port_numbers << std::endl;
                return -1;
            } else {
                LOG(INFO) << "open the specified camera : bus num:" << attr_t.attr.usbAttrs.bnum
                          << "  port numbers:" << std::string(attr_t.attr.usbAttrs.port_numbers) << std::endl;
            }
        }
        break;
    case AS_CAMERA_ATTR_NET:
        break;
    default:
        break;
    }
    logCameraPathInfo(attr_t);

    // unsigned int dev_idx = imgPubList.size();
    stPublisherInfo.index = dev_idx;

    /* create publisher */
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.usbAttrs.bnum == attr_t.attr.usbAttrs.bnum)
                && (strcmp(it->attr_s.attr.usbAttrs.port_numbers, attr_t.attr.usbAttrs.port_numbers) == 0)) {
                LOG(INFO) << "reconnected, update the camera info" << std::endl;
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                break;
            }
        }
        break;
    case AS_CAMERA_ATTR_NET:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.netAttrs.port == attr_t.attr.netAttrs.port)
                && (strcmp(it->attr_s.attr.netAttrs.ip_addr, attr_t.attr.netAttrs.ip_addr) == 0)) {
                LOG(INFO) << "reconnected, update the camera info" << std::endl;
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                break;
            }
        }
        break;
    default:
        break;
    }

    if (!reconnected) {
        LOG(INFO) << "create a new publisher info set" << std::endl;
        switch (cam_type) {
        case AS_SDK_CAM_MODEL_NUWA_XB40:
        case AS_SDK_CAM_MODEL_NUWA_X100:
        case AS_SDK_CAM_MODEL_NUWA_HP60:
        case AS_SDK_CAM_MODEL_NUWA_HP60V:
            stPublisherInfo.depth_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
            stPublisherInfo.depth_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.ir_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("ir/camera_info", 100);
            stPublisherInfo.ir_img_pub =
                nh->advertise<sensor_msgs::Image>("ir/image" + std::to_string(dev_idx), 100);
            stPublisherInfo.depth_points_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/points" +
                                               std::to_string(dev_idx), 100);
            break;
        case AS_SDK_CAM_MODEL_HP60C:
        case AS_SDK_CAM_MODEL_HP60CN:
        case AS_SDK_CAM_MODEL_VEGA:
            stPublisherInfo.depth_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
            stPublisherInfo.depth_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.rgb_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 100);
            stPublisherInfo.rgb_img_raw_pub =
                nh->advertise<sensor_msgs::Image>("rgb/image" + std::to_string(dev_idx), 100);
            if (m_launch_param.pub_mono8) {
                stPublisherInfo.mono8_img_pub =
                    nh->advertise<sensor_msgs::Image>("mono8/image" + std::to_string(dev_idx), 100);
            }
            stPublisherInfo.depth_points_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/points"/* +
                                               std::to_string(dev_idx)*/, 100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_A:
            stPublisherInfo.depth_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
            stPublisherInfo.depth_odd_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/odd_image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.depth_points_odd_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/odd_points" +
                                                   std::to_string(dev_idx), 100);
            stPublisherInfo.depth_even_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/even_image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.depth_points_even_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/even_points" +
                                                    std::to_string(dev_idx), 100);
            stPublisherInfo.depth_merge_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/merge_image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.depth_points_merge_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/merge_points" +
                    std::to_string(dev_idx), 100);
            stPublisherInfo.peak_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("peak/camera_info", 100);
            stPublisherInfo.peak_odd_img_pub =
                nh->advertise<sensor_msgs::Image>("peak/odd_image" + std::to_string(dev_idx), 100);
            stPublisherInfo.peak_even_img_pub =
                nh->advertise<sensor_msgs::Image>("peak/even_image" + std::to_string(dev_idx), 100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_C:
            stPublisherInfo.depth_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
            stPublisherInfo.depth_raw_pub =
                nh->advertise<sensor_msgs::Image>("depth/image_raw" + std::to_string(dev_idx), 100);
            stPublisherInfo.rgb_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 100);
            stPublisherInfo.rgb_img_raw_pub =
                nh->advertise<sensor_msgs::Image>("rgb/image" + std::to_string(dev_idx), 100);
            if (m_launch_param.pub_mono8) {
                stPublisherInfo.mono8_img_pub =
                    nh->advertise<sensor_msgs::Image>("mono8/image" + std::to_string(dev_idx), 100);
            }
            stPublisherInfo.peak_camera_info_pub =
                nh->advertise<sensor_msgs::CameraInfo>("peak/camera_info", 100);
            stPublisherInfo.peak_img_pub =
                nh->advertise<sensor_msgs::Image>("peak/image" + std::to_string(dev_idx), 100);
            stPublisherInfo.depth_points_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/points" +
                                               std::to_string(dev_idx), 100);
            break;
        case AS_SDK_CAM_MODEL_KONDYOR:
            stPublisherInfo.depth_points_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/points" +
                                               std::to_string(dev_idx), 100);
            break;
        case AS_SDK_CAM_MODEL_KONDYOR_NET:
            stPublisherInfo.depth_points_pub = nh->advertise<sensor_msgs::PointCloud2>("depth/points" +
                                               std::to_string(dev_idx), 100);
            break;
        default:
            break;
        }

        stPublisherInfo.pCamera = pCamera;
        memcpy(&stPublisherInfo.attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
        stPublisherInfo.stream_flg = param.image_flag;
        imgPubList.push_back(stPublisherInfo);
    }
    m_camera_map.insert(std::make_pair(pCamera, new Camera(pCamera, cam_type)));
    m_cam_type_map.insert(std::make_pair(pCamera, cam_type));

    /* nuwa camera whether match usb device.
       If it is a virtual machine, do not match it. */
    if ((cam_type == AS_SDK_CAM_MODEL_NUWA_XB40) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_X100) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60V)) {
        extern int AS_Nuwa_SetUsbDevMatch(bool is_match);
        AS_Nuwa_SetUsbDevMatch(!virtualMachine());
        // AS_Nuwa_SetUsbDevMatch(false);
    }
    return 0;
}

int CameraPublisher::onCameraDetached(AS_CAM_PTR pCamera)
{
    int ret = 0;

    LOG(INFO) << "camera detached" << std::endl;
    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        delete camIt->second;
        m_camera_map.erase(pCamera);
    }

    if (m_cam_type_map.find(pCamera) != m_cam_type_map.end()) {
        m_cam_type_map.erase(pCamera);
    }
    return ret;
}

void CameraPublisher::onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    ros::Time time = ros::Time::now();

    if (ret == 0) {
        depthInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudPublisher(pCamera, pstData, stIrRgbParameter);
        rgbInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        irInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        peakInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
//        tfPublisher(pCamera, stIrRgbParameter);
    }
}

void CameraPublisher::onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveMergeImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    ros::Time time = ros::Time::now();

    if (ret == 0) {
        depthMergeInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudMergePublisher(pCamera, pstData, stIrRgbParameter);
    }
}

int CameraPublisher::onCameraOpen(AS_CAM_PTR pCamera)
{
    int ret = 0;
    LOG(INFO) << "camera opened" << std::endl;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            memset(&it->config_info, 0, sizeof(AS_CONFIG_INFO_S));
            AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
            ret = AS_SDK_GetCameraModel(pCamera, cam_type);
            if (ret < 0) {
                LOG(ERROR) << "get camera model fail" << std::endl;
                return ret;
            }
            if ((cam_type == AS_SDK_CAM_MODEL_HP60C) || (cam_type == AS_SDK_CAM_MODEL_HP60CN)
                || (cam_type == AS_SDK_CAM_MODEL_VEGA)) {
                ret = AS_SDK_GetConfigInfo(pCamera, it->config_info);
                LOG(INFO) << "get config info, ret " << ret << ", is_Registration " << it->config_info.is_Registration << std::endl;
            }
        }
    }

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        camIt->second->init();
    }

    ret = setResolution(pCamera, m_launch_param);
    if (ret < 0) {
        LOG(ERROR) << "set resolution fail." << std::endl;
        return ret;
    }

    return ret;
}

int CameraPublisher::onCameraClose(AS_CAM_PTR pCamera)
{
    int ret = 0;

    return ret;
}

int CameraPublisher::onCameraStart(AS_CAM_PTR pCamera)
{
    int ret = 0;
    return ret;
}

int CameraPublisher::onCameraStop(AS_CAM_PTR pCamera)
{
    int ret = 0;

    return ret;
}

int CameraPublisher::genRosDepthCamInfo(const AS_Frame_s *pstFrame,
                                        sensor_msgs::CameraInfo *pstCamInfo,
                                        unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->header.frame_id = "ascamera";
    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxir;
    pstCamInfo->K[2] = stParameter.cxir;
    pstCamInfo->K[4] = stParameter.fyir;
    pstCamInfo->K[5] = stParameter.cyir;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosRgbCamInfo(const AS_Frame_s *pstFrame,
                                      sensor_msgs::CameraInfo *pstCamInfo,
                                      unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxrgb;
    pstCamInfo->K[2] = stParameter.cxrgb;
    pstCamInfo->K[4] = stParameter.fyrgb;
    pstCamInfo->K[5] = stParameter.cyrgb;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosIrCamInfo(const AS_Frame_s *pstFrame,
                                     sensor_msgs::CameraInfo *pstCamInfo,
                                     unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxir;
    pstCamInfo->K[2] = stParameter.cxir;
    pstCamInfo->K[4] = stParameter.fyir;
    pstCamInfo->K[5] = stParameter.cyir;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosPeakCamInfo(const AS_Frame_s *pstFrame,
                                       sensor_msgs::CameraInfo *pstCamInfo,
                                       unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.frame_id = "ascamera";
    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxir;
    pstCamInfo->K[2] = stParameter.cxir;
    pstCamInfo->K[4] = stParameter.fyir;
    pstCamInfo->K[5] = stParameter.cyir;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosDepthImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                      unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width * 2;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

int CameraPublisher::genRosRgbImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                    unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::BGR8;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width * 3;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));
    return 0;
}

int CameraPublisher::genRosIrImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                   unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

int CameraPublisher::genRosMono8Image(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                      unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::MONO8;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

int CameraPublisher::genRosPeakImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                     unsigned int seq)
{
    pstImage->header.frame_id = "ascamera";
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

void CameraPublisher::depthInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->depthImg.size == 0) {
        return;
    }
    sensor_msgs::CameraInfo stDepthCamInfo;
    sensor_msgs::Image stDepthImage;

    stDepthCamInfo.header.stamp = time;
    stDepthImage.header.stamp = time;

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
        genRosDepthCamInfo(&pstData->depthImg, &stDepthCamInfo, seq, stParameter);
    }

    genRosDepthImage(&pstData->depthImg, &stDepthImage, seq);

    seq++;
    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (it->config_info.is_Registration) {
                    stDepthImage.header.frame_id = "color" + std::to_string(it->index);
                    stDepthCamInfo.header.frame_id = "color" + std::to_string(it->index);
                } else {
                    stDepthImage.header.frame_id = "depth" + std::to_string(it->index);
                    stDepthCamInfo.header.frame_id = "depth" + std::to_string(it->index);
                }
                if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
                    it->depth_camera_info_pub.publish(stDepthCamInfo);
                }
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->depthImg.height == 96) {
                        it->depth_odd_raw_pub.publish(stDepthImage);
                    } else if (pstData->depthImg.height == 8) {
                        it->depth_even_raw_pub.publish(stDepthImage);
                    }
                } else {
                    it->depth_raw_pub.publish(stDepthImage);
                }
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::pointcloudPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    sensor_msgs::PointCloud2 pointCloudMsg;
    sensor_msgs::PointCloud2 pointCloudMsgRgb;
    
    /* publish point cloud */
    if (m_launch_param.color_pcl == false) {
        pcl::PointCloud<pcl::PointXYZ> stPointCloud;
        stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
        stPointCloud.height = 1;
        stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

        for (unsigned int i = 0 ; i < stPointCloud.points.size(); i += 90) {
            int index = i * 3;
            
            float z_value = *((float *)pData->pointCloud.data + index + 2) / 1000;
            
            if(z_value < 2.0) {
            	stPointCloud.points[i].x = z_value;
            	stPointCloud.points[i].y = -*((float *)pData->pointCloud.data + index) / 1000;
            	stPointCloud.points[i].z = -*((float *)pData->pointCloud.data + index + 1) / 1000;
            	
//            	std::cout << "x,y,z: " << stPointCloud.points[i].x << "," << stPointCloud.points[i].y << "," << stPointCloud.points[i].z << std::endl;
            }
        }

                // 创建旋转矩阵
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(roll_/180*3.1415, Eigen::Vector3f::UnitX()));
        transform.rotate(Eigen::AngleAxisf(pitch_/180*3.1415, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf(yaw_/180*3.1415, Eigen::Vector3f::UnitZ()));

        // 应用旋转
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(stPointCloud, *transformed_cloud, transform);

        static unsigned int seq = 0;
        pcl::toROSMsg(*transformed_cloud, pointCloudMsg);
        pointCloudMsg.header.stamp = ros::Time::now();
        pointCloudMsg.header.seq = seq++;

        AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
        auto it_par = m_cam_type_map.find(pCamera);
        if ( it_par != m_cam_type_map.end()) {
            cam_type = it_par->second;
        }
        if (ros::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                        pointCloudMsg.header.frame_id = "ascamera";
                        if (pData->pointCloud.height == 96) {
                            it->depth_points_odd_pub.publish(pointCloudMsg);
                        } else if (pData->pointCloud.height == 8) {
                            it->depth_points_even_pub.publish(pointCloudMsg);
                        }
                    } else {
                    	 std::string frame_id = ros::this_node::getName();
                        if (it->config_info.is_Registration) {
                            pointCloudMsg.header.frame_id = frame_id.erase(0, 1);;// + "color";// + std::to_string(it->index);
                        } else {
                            pointCloudMsg.header.frame_id = frame_id.erase(0, 1);// + "depth";// + std::to_string(it->index);
                        }
                        it->depth_points_pub.publish(pointCloudMsg);
                    }
                }
            }
            ros::spinOnce();
        }
    } else { /* publish point cloud with rgb */
        static unsigned int color_pcl_seq = 0;
        pcl::PointCloud<pcl::PointXYZRGB> stPointCloudRgb;
        for (unsigned int y = 0; y < pData->depthImg.height; y++) {
            for (unsigned int x = 0; x < pData->depthImg.width; x++) {
                unsigned short depth = ((unsigned short *)pData->depthImg.data)[y *
                                         pData->depthImg.width + x];
                if (depth == 0)
                    continue;
                pcl::PointXYZRGB p;

                p.z = double(depth) / 1000;
                p.x = (x - stParameter.cxrgb) * p.z / stParameter.fxrgb;
                p.y = (y - stParameter.cyrgb) * p.z / stParameter.fyrgb;

                p.b = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 ];
                p.g = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 + 1];
                p.r = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x ) * 3 + 2];

                stPointCloudRgb.points.push_back(p);
            }
        }
        pcl::toROSMsg(stPointCloudRgb, pointCloudMsgRgb);
        pointCloudMsgRgb.header.stamp = ros::Time::now();
        pointCloudMsgRgb.header.seq = color_pcl_seq++;
        if (ros::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (it->config_info.is_Registration) {
                        pointCloudMsgRgb.header.frame_id = "color" + std::to_string(it->index);
                    } else {
                        pointCloudMsgRgb.header.frame_id = "depth" + std::to_string(it->index);
                    }
                    it->depth_points_pub.publish(pointCloudMsgRgb);
                }
            }
            ros::spinOnce();
        }
    }
}

void CameraPublisher::rgbInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                       AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    if (pstData->rgbImg.size == 0) {
        return;
    }
    static unsigned int seq = 0;
    sensor_msgs::CameraInfo stRgbCamInfo;
    sensor_msgs::Image rgbImage;
    sensor_msgs::Image mono8Image;

    stRgbCamInfo.header.stamp = time;
    rgbImage.header.stamp = time;

    genRosRgbCamInfo(&pstData->rgbImg, &stRgbCamInfo, seq, stParameter);
    genRosRgbImage(&pstData->rgbImg, &rgbImage, seq);
    if (m_launch_param.pub_mono8) {
        unsigned int img_width = pstData->rgbImg.width;
        unsigned int img_height = pstData->rgbImg.height;
        std::shared_ptr<unsigned char> ptr(new (std::nothrow) unsigned char[img_width * img_height],
                                           std::default_delete<unsigned char[]>());
        unsigned char *mono8_data = ptr.get();;
        int ret = bgr2mono8(mono8_data, static_cast<unsigned char *>(pstData->rgbImg.data), img_width, img_height);
        if (ret == 0) {
            AS_Frame_s mono8_frame;
            mono8_frame.data = static_cast<void *>(mono8_data);
            mono8_frame.width = pstData->rgbImg.width;
            mono8_frame.height = pstData->rgbImg.height;
            mono8_frame.size = pstData->rgbImg.width * pstData->rgbImg.height;
            genRosMono8Image(&mono8_frame, &mono8Image, seq);
        }
    }
    seq++;

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                stRgbCamInfo.header.frame_id = "color" + std::to_string(it->index);
                it->rgb_camera_info_pub.publish(stRgbCamInfo);
                rgbImage.header.frame_id = "color" + std::to_string(it->index);
                it->rgb_img_raw_pub.publish(rgbImage);
                if (m_launch_param.pub_mono8) {
                    mono8Image.header.frame_id = "color" + std::to_string(it->index);
                    it->mono8_img_pub.publish(mono8Image);
                }
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::irInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                      AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->irImg.size == 0) {
        return;
    }
    sensor_msgs::CameraInfo stIrCamInfo;
    sensor_msgs::Image irImage;
    genRosIrCamInfo(&pstData->irImg, &stIrCamInfo, seq, stParameter);
    genRosIrImage(&pstData->irImg, &irImage, seq);
    seq++;
    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                stIrCamInfo.header.frame_id = "depth" + std::to_string(it->index);
                irImage.header.frame_id = "depth" + std::to_string(it->index);
                it->ir_camera_info_pub.publish(stIrCamInfo);
                it->ir_img_pub.publish(irImage);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::peakInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->peakImg.size == 0) {
        return;
    }
    sensor_msgs::CameraInfo stPeakCamInfo;
    sensor_msgs::Image peakImage;
    genRosPeakCamInfo(&pstData->peakImg, &stPeakCamInfo, seq, stParameter);
    genRosPeakImage(&pstData->peakImg, &peakImage, seq);
    seq++;

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                it->peak_camera_info_pub.publish(stPeakCamInfo);
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->peakImg.height == 96) {
                        it->peak_odd_img_pub.publish(peakImage);
                    } else if (pstData->peakImg.height == 8) {
                        it->peak_even_img_pub.publish(peakImage);
                    }
                } else {
                    it->peak_img_pub.publish(peakImage);
                }
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::depthMergeInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }
    if (cam_type != AS_SDK_CAM_MODEL_KUNLUN_A) {
        return;
    }

    static unsigned int seq = 0;
    if (pstData->depthImg.size == 0) {
        return;
    }
    sensor_msgs::CameraInfo stDepthCamInfo;
    sensor_msgs::Image stDepthImage;

    stDepthCamInfo.header.stamp = time;
    stDepthImage.header.stamp = time;

    genRosDepthCamInfo(&pstData->depthImg, &stDepthCamInfo, seq, stParameter);
    genRosDepthImage(&pstData->depthImg, &stDepthImage, seq);

    seq++;
    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                it->depth_camera_info_pub.publish(stDepthCamInfo);
                it->depth_merge_raw_pub.publish(stDepthImage);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::pointcloudMergePublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    sensor_msgs::PointCloud2 pointCloudMsg;
    sensor_msgs::PointCloud2 pointCloudMsgRgb;

    /* publish point cloud */
    pcl::PointCloud<pcl::PointXYZ> stPointCloud;
    stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
    stPointCloud.height = 1;
    stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

    for (unsigned int i = 0 ; i < stPointCloud.points.size(); i++) {
        int index = i * 3;
        stPointCloud.points[i].x = *((float *)pData->pointCloud.data + index) / 1000;
        stPointCloud.points[i].y = *((float *)pData->pointCloud.data + index + 1) / 1000;
        stPointCloud.points[i].z = *((float *)pData->pointCloud.data + index + 2) / 1000;
    }

    static unsigned int merge_seq = 0;
    pcl::toROSMsg(stPointCloud, pointCloudMsg);
    pointCloudMsg.header.frame_id = "ascamera";
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudMsg.header.seq = merge_seq++;

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                it->depth_points_merge_pub.publish(pointCloudMsg);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::tfPublisher(AS_CAM_PTR pCamera, AS_CAM_Parameter_s &stParameter)
{
    static unsigned int seq = 0;
    static tf2_ros::StaticTransformBroadcaster br;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            auto it_par = m_cam_type_map.find(pCamera);
            if (it_par != m_cam_type_map.end()) {
                if ((it_par->second == AS_SDK_CAM_MODEL_HP60C) || (it_par->second == AS_SDK_CAM_MODEL_HP60CN)
                    || (it_par->second == AS_SDK_CAM_MODEL_VEGA)) {
                    geometry_msgs::TransformStamped rgbTransform;
                    rgbTransform.header.seq = seq++;
                    rgbTransform.header.stamp = ros::Time::now();
                    rgbTransform.header.frame_id = ros::this_node::getName() + "_camera_link";// + std::to_string(it->index);
                    rgbTransform.child_frame_id = ros::this_node::getName() + "_color";// + std::to_string(it->index);
                    rgbTransform.transform.translation.x = stParameter.T1 / 1000;
                    rgbTransform.transform.translation.y = stParameter.T2 / 1000;
                    rgbTransform.transform.translation.z = stParameter.T3 / 1000;
                    rgbTransform.transform.rotation.x = 0;
                    rgbTransform.transform.rotation.y = 0;
                    rgbTransform.transform.rotation.z = 0;

                    tf2::Quaternion qtn;

                    double sy = std::sqrt(stParameter.R00 * stParameter.R00 + stParameter.R10 * stParameter.R10);
                    double angleX;
                    double angleY;
                    double angleZ;
                    bool singular = sy < 1e-6;
                    if (!singular) {
                        angleX = std::atan2(stParameter.R21, stParameter.R22);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = std::atan2(stParameter.R10, stParameter.R00);
                    } else {
                        angleX = std::atan2(-stParameter.R12, stParameter.R11);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = 0;
                    }
                    // std::cout << "angleX:" << angleX << std::endl;
                    // std::cout << "angleY:" << angleY << std::endl;
                    // std::cout << "angleZ:" << angleZ << std::endl;

                    qtn.setRPY(angleX, angleY, angleZ);
                    rgbTransform.transform.rotation.x = qtn.getX();
                    rgbTransform.transform.rotation.y = qtn.getY();
                    rgbTransform.transform.rotation.z = qtn.getZ();
                    rgbTransform.transform.rotation.w = qtn.getW();

                    br.sendTransform(rgbTransform);
                }
            }

            geometry_msgs::TransformStamped depthTransform;
            depthTransform.header.stamp = ros::Time::now();
            depthTransform.header.frame_id = "camera_link" + std::to_string(it->index);
            if (it->config_info.is_Registration) {
                depthTransform.child_frame_id = "color" + std::to_string(it->index);
            } else {
                depthTransform.child_frame_id = "depth" + std::to_string(it->index);
            }
            depthTransform.transform.translation.x = 0.0;
            depthTransform.transform.translation.y = 0.0;
            depthTransform.transform.translation.z = 0.0;
            depthTransform.transform.rotation.x = 0.0;
            depthTransform.transform.rotation.y = 0.0;
            depthTransform.transform.rotation.z = 0.0;
            depthTransform.transform.rotation.w = 1.0;

            br.sendTransform(depthTransform);
        }
    }

    ros::spinOnce();
}

int CameraPublisher::setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S resolution)
{
    int ret = 0;
    if ((resolution.set_depth_width != -1) && (resolution.set_depth_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s depthInfo;
        depthInfo.width = resolution.set_depth_width;
        depthInfo.height = resolution.set_depth_height;
        depthInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_DEPTH, &depthInfo);
        if (ret < 0) {
            LOG(ERROR) << "set depth param fail" << std::endl;
            return ret;
        }
        LOG(INFO) << "set depth resolution: " << depthInfo.width << " x " << depthInfo.height << " @ " << depthInfo.fps << "fps"
                  << std::endl;
    }
    if ((resolution.set_rgb_width != -1) && (resolution.set_rgb_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s rgbInfo;
        rgbInfo.width = resolution.set_rgb_width;
        rgbInfo.height = resolution.set_rgb_height;
        rgbInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_RGB, &rgbInfo);
        if (ret < 0) {
            LOG(ERROR) << "set rgb param fail" << std::endl;
            return ret;
        }
        LOG(INFO) << "set rgb resolution: " << rgbInfo.width << " x " << rgbInfo.height << " @ " << rgbInfo.fps << "fps" <<
                  std::endl;
    }
    if ((resolution.set_ir_width != -1) && (resolution.set_ir_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s irInfo;
        irInfo.width = resolution.set_ir_width;
        irInfo.height = resolution.set_ir_height;
        irInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_IR, &irInfo);
        if (ret < 0) {
            LOG(ERROR) << "set ir param fail" << std::endl;
            return ret;
        }
        LOG(INFO) << "set ir resolution: " << irInfo.width << " x " << irInfo.height << " @ " << irInfo.fps << "fps" <<
                  std::endl;
    }
    if ((resolution.set_peak_width != -1) && (resolution.set_peak_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s peakInfo;
        peakInfo.width = resolution.set_peak_width;
        peakInfo.height = resolution.set_peak_height;
        peakInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_PEAK, &peakInfo);
        if (ret < 0) {
            LOG(ERROR) << "set peak param fail" << std::endl;
            return ret;
        }
        LOG(INFO) << "set peak resolution: " << peakInfo.width << " x " << peakInfo.height << " @ " << peakInfo.fps << "fps" <<
                  std::endl;
    }
    return ret;
}

void CameraPublisher::saveImage(const std::string &serialno, const AS_SDK_Data_s *pstData)
{

    static int depthindex = 0;
    static int pointCloudIndex = 0;
    static int rgbindex = 0;
    static int irindex = 0;
    static int peakindex = 0;

    if (pstData->depthImg.size > 0) {
        std::string depthImgName(std::string("depth_" + std::to_string(pstData->depthImg.width) + "x" +
                                             std::to_string(pstData->depthImg.height)
                                             + "_" + std::to_string(depthindex++) + ".yuv"));
        if (saveYUVImg(depthImgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            LOG(ERROR) << "save img failed!" << std::endl;
        } else {
            LOG(INFO) << "save depth image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << depthImgName << std::endl;
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string("PointCloud_" + std::to_string(pointCloudIndex++) + ".txt"));
        if (savePointCloud(pointCloudName.c_str(), (float *)pstData->pointCloud.data,
                           pstData->pointCloud.size / sizeof(float)) != 0) {
            LOG(ERROR) << "save point cloud failed!" << std::endl;
        } else {
            LOG(INFO) << "save point cloud success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << pointCloudName << std::endl;
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string("rgb_" + std::to_string(pstData->rgbImg.width) + "x" +
                                        std::to_string(pstData->rgbImg.height)
                                        + "_" + std::to_string(rgbindex++) + ".yuv"));
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            LOG(ERROR) << "save rgb image failed!" << std::endl;
        } else {
            LOG(INFO) << "save rgb image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << rgbName << std::endl;
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irName(std::string("ir_" + std::to_string(pstData->irImg.width) + "x" +
                                       std::to_string(pstData->irImg.height)
                                       + "_" + std::to_string(irindex++) + ".yuv"));
        if (saveYUVImg(irName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            LOG(ERROR) << "save ir image failed!" << std::endl;
        } else {
            LOG(INFO) << "save ir image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << irName << std::endl;
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakName(std::string("peak_" + std::to_string(pstData->peakImg.width) + "x" +
                                         std::to_string(pstData->peakImg.height)
                                         + "_" + std::to_string(peakindex++) + ".yuv"));
        if (saveYUVImg(peakName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            LOG(ERROR) << "save peak image failed!" << std::endl;
        } else {
            LOG(INFO) << "save peak image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << peakName << std::endl;
        }
    }

}

void CameraPublisher::logCfgParameter()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        AS_SDK_LogCameraCfg(it->first);
    }
}

int CameraPublisher::streamController()
{
    int ret = 0;
    return ret;
}

bool CameraPublisher::virtualMachine()
{
    int cnt = 0;
    char szCnt[8];
    FILE *fp = nullptr;

    char cmd[128];
    snprintf(cmd, sizeof(cmd) - 1, R"(lscpu | grep "Hypervisor vendor" | wc -l)");
    fp = popen(cmd, "r");
    if (fgets(szCnt, sizeof(szCnt), fp) != nullptr) {
        if (strlen(szCnt) != 0) {
            cnt = std::stoi(szCnt);
        }
    }
    pclose(fp);
    fp = nullptr;
    if (cnt == 0) {
        return false;
    } else {
        return true;
    }
}

void CameraPublisher::logCameraPathInfo(AS_CAM_ATTR_S &attr_t)
{
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        LOG(INFO) << "usb camera" << std::endl;
        LOG(INFO) << "bnum:" << attr_t.attr.usbAttrs.bnum << std::endl;
        LOG(INFO) << "dnum:" << attr_t.attr.usbAttrs.dnum << std::endl;
        LOG(INFO) << "port_numbers:" << attr_t.attr.usbAttrs.port_numbers << std::endl;
        break;
    case AS_CAMERA_ATTR_NET:
        LOG(INFO) << "net camera" << std::endl;
        LOG(INFO) << "ip:" << attr_t.attr.netAttrs.ip_addr << std::endl;
        LOG(INFO) << "port:" << attr_t.attr.netAttrs.port << std::endl;
        break;
    default:
        break;
    }
}

int CameraPublisher::initLaunchParams()
{
    memset(&m_launch_param, 0, sizeof(LAUNCH_CONFI_PARAM_S));
    nh->param("depth_width", m_launch_param.set_depth_width, -1);
    nh->param("depth_height", m_launch_param.set_depth_height, -1);
    nh->param("rgb_width", m_launch_param.set_rgb_width, -1);
    nh->param("rgb_height", m_launch_param.set_rgb_height, -1);
    nh->param("ir_width", m_launch_param.set_ir_width, -1);
    nh->param("ir_height", m_launch_param.set_ir_height, -1);
    nh->param("peak_width", m_launch_param.set_peak_width, -1);
    nh->param("peak_height", m_launch_param.set_peak_height, -1);
    nh->param("fps", m_launch_param.set_fps, -1);
    nh->param("usb_bus_no", m_launch_param.usb_bus_num, -1);
    std::string usb_port_nums;
    nh->param<std::string>("usb_path", usb_port_nums, "null");
    strncpy(m_launch_param.usb_port_nums, usb_port_nums.c_str(),
            std::min(sizeof(m_launch_param.usb_port_nums), usb_port_nums.length()));
    nh->param("color_pcl", m_launch_param.color_pcl, false);
    nh->param("pub_mono8", m_launch_param.pub_mono8, false);
    printLaunchParams(m_launch_param);

    nh->param("roll", roll_, 0.0);
    nh->param("pitch", pitch_, 0.0);
    nh->param("yaw", yaw_, 0.0);

    return 0;
}

int CameraPublisher::printLaunchParams(LAUNCH_CONFI_PARAM_S para)
{
    LOG_PARA(para.set_depth_width, "depth_width");
    LOG_PARA(para.set_depth_height, "depth_height");

    LOG_PARA(para.set_rgb_width, "rgb_width");
    LOG_PARA(para.set_rgb_height, "rgb_height");

    LOG_PARA(para.set_ir_width, "ir_width");
    LOG_PARA(para.set_ir_height, "ir_height");

    LOG_PARA(para.set_peak_width, "peak_width");
    LOG_PARA(para.set_peak_height, "peak_height");

    LOG_PARA(para.set_fps, "set_fps");
    LOG_PARA(para.usb_bus_num, "usb_bus_num");

    if (strncmp(para.usb_port_nums, "null", strlen("null")) != 0) {
        LOG(INFO) << "get usb_port_nums " << para.usb_port_nums << std::endl;
    }

    if (para.color_pcl != false) {
        LOG(INFO) << "get color_pcl " << para.color_pcl << std::endl;
    }

    if (para.pub_mono8 != false) {
        LOG(INFO) << "get pub_mono8 " << para.pub_mono8 << std::endl;
    }

    return 0;
}
