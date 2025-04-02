/**
 * @file      Camera.cpp
 * @brief     angstrong Camera.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/05/29
 * @version   1.0

 */
#include <unistd.h>
#include "Camera.h"

Camera::Camera(AS_CAM_PTR pCamera, const AS_SDK_CAM_MODEL_E &cam_type)
{
    int ret = 0;
    m_handle = pCamera;
    m_cam_type = cam_type;
    m_check_fps = new CheckFps(pCamera);
    ret = AS_SDK_GetCameraAttrs(m_handle,  m_attr);
    if (ret != 0) {
        LOG(WARN) << "get camera attrs failed" << std::endl;
    }
    memset(&m_cam_parameter, 0, sizeof(AS_CAM_Parameter_s));
}

Camera::~Camera()
{
    if (m_check_fps != nullptr) {
        delete m_check_fps;
        m_check_fps = nullptr;
    }
    if (m_is_thread) {
        m_is_thread = false;
    }
    if (m_backgroundThread.joinable()) {
        m_backgroundThread.join();
    }
}

int Camera::init()
{
    int ret = 0;
    char sn_buff[64] = {0};
    ret = AS_SDK_GetSerialNumber(m_handle, sn_buff, sizeof(sn_buff));
    if (ret != 0) {
        LOG(ERROR) << "get camera serial number failed" << std::endl;
        return -1;
    }
    m_serialno = std::string(sn_buff);

    char fwVersion[100] = {0};
    ret = AS_SDK_GetFwVersion(m_handle, fwVersion, sizeof(fwVersion));
    if (ret == 0) {
        LOG(INFO) << "#camera[" << m_handle << "] SN[" << m_serialno << "]'s firmware version:" << fwVersion << std::endl;
    }
    m_is_thread = true;
    m_backgroundThread = std::thread(&Camera::backgroundThread, this);
    return ret;
}

double Camera::checkFps()
{
    std::string Info = "";
    switch (m_attr.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        Info = (std::to_string(m_attr.attr.usbAttrs.bnum) + ":" + m_attr.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        Info = (std::to_string(m_attr.attr.netAttrs.port) + ":" + m_attr.attr.netAttrs.ip_addr);
        break;
    default:
        LOG(ERROR) << "attr type error" << std::endl;
        break;
    }
    return m_check_fps->checkFps(m_serialno, Info);
}

int Camera::enableSaveImage(bool enable)
{
    m_save_img = enable;
    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        m_save_merge_img = enable;
    }
    return 0;
}

int Camera::getSerialNo(std::string &sn)
{
    sn = m_serialno;
    return 0;
}

int Camera::getCameraAttrs(AS_CAM_ATTR_S &info)
{
    info = m_attr;
    return 0;
}

int Camera::getCamParameter(AS_CAM_Parameter_s &cam_parameter)
{
    if (m_isGetParameter) {
        memcpy(&cam_parameter, &m_cam_parameter, sizeof(AS_CAM_Parameter_s));
        return 0;
    } else {
        return -1;
    }
}

int Camera::backgroundThread()
{
    int ret = 0;
    while (m_is_thread) {
        // KONDYOR not support to get CamParameter, KUNLUN A don't need to get CamParameter
        if ((m_cam_type != AS_SDK_CAM_MODEL_KONDYOR_NET) && (m_cam_type != AS_SDK_CAM_MODEL_KONDYOR)
            && (m_cam_type != AS_SDK_CAM_MODEL_KUNLUN_A)) {
            ret = AS_SDK_GetCamParameter(m_handle, &m_cam_parameter);
            if (ret == 0) {
                LOG(INFO) << "SN [ " << m_serialno << " ]'s parameter:" << std::endl;
                LOG(INFO) << "irfx: " << m_cam_parameter.fxir << std::endl;
                LOG(INFO) << "irfy: " << m_cam_parameter.fyir << std::endl;
                LOG(INFO) << "ircx: " << m_cam_parameter.cxir << std::endl;
                LOG(INFO) << "ircy: " << m_cam_parameter.cyir << std::endl;
                LOG(INFO) << "rgbfx: " << m_cam_parameter.fxrgb << std::endl;
                LOG(INFO) << "rgbfy: " << m_cam_parameter.fyrgb << std::endl;
                LOG(INFO) << "rgbcx: " << m_cam_parameter.cxrgb << std::endl;
                LOG(INFO) << "rgbcy: " << m_cam_parameter.cyrgb << std::endl << std::endl;
                m_is_thread = false;
                m_isGetParameter = true;
                break;
            }
        } else {
            m_isGetParameter = true;
            m_is_thread = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}

void Camera::saveImage(const AS_SDK_Data_s *pstData)
{
    if (!m_save_img) {
        m_cnt = 0;
        return;
    }

    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        if (m_cnt >= 1) {
            m_save_img = false;
        }
        m_cnt++;
    } else {
        m_save_img = false;
    }

    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            LOG(ERROR) << "save depth image failed!" << std::endl;
        } else {
            LOG(INFO) << "save depth image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << depthimgName << std::endl;
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string(m_serialno + "_rgb_") + std::to_string(pstData->rgbImg.width) + "x" +
                            std::to_string(pstData->rgbImg.height) + "_" + std::to_string(m_rgbindex++) + ".yuv");
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            LOG(ERROR) << "save rgb image failed!" << std::endl;
        } else {
            LOG(INFO) << "save rgb image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << rgbName << std::endl;
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            LOG(ERROR) << "save point cloud failed!" << std::endl;
        } else {
            LOG(INFO) << "save point cloud success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << pointCloudName << std::endl;
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irimgName(std::string(m_serialno + "_ir_" + std::to_string(pstData->irImg.width) + "x" +
                                          std::to_string(pstData->irImg.height) + "_" + std::to_string(m_irindex++) + ".yuv"));
        if (saveYUVImg(irimgName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            LOG(ERROR) << "save ir image failed!" << std::endl;
        } else {
            LOG(INFO) << "save ir image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << irimgName << std::endl;
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakimgName(std::string(m_serialno + "_peak_") + std::to_string(
                                    pstData->peakImg.width) + "x" + std::to_string(pstData->peakImg.height)
                                + "_" + std::to_string(m_peakindex++) + ".yuv");
        if (saveYUVImg(peakimgName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            LOG(ERROR) << "save peak image failed!" << std::endl;
        } else {
            LOG(INFO) << "save peak image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << peakimgName << std::endl;
        }
    }

    return;
}

void Camera::saveMergeImage(const AS_SDK_MERGE_s *pstData)
{
    if (!m_save_merge_img) {
        return;
    }
    m_save_merge_img = false;
    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_merge_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            LOG(ERROR) << "save depth image failed!" << std::endl;
        } else {
            LOG(INFO) << "save depth image success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << depthimgName << std::endl;
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_merge_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            LOG(ERROR) << "save point cloud failed!" << std::endl;
        } else {
            LOG(INFO) << "save point cloud success!" << std::endl;
            LOG(INFO) << "location: " << getcwd(nullptr, 0) << "/" << pointCloudName << std::endl;
        }
    }

    return;
}
