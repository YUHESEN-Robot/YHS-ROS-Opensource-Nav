#include "yhs_mk_can_control.h"

namespace can_control
{

  MkCanControl::MkCanControl()
  {
    ros::NodeHandle private_node("~");

    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
    private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
    private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
    private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));
    private_node.param("/yhs_can_control/wheel_base", wheel_base_, 0.6);

    ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::MkCtrlCmd>("ctrl_cmd", 5, &MkCanControl::CtrlCmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_msgs::MkIoCmd>("io_cmd", 5, &MkCanControl::IoCmdCallBack, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smoother_cmd_vel", 5, &MkCanControl::CmdCallBack, this);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &MkCanControl::ImuDataCallBack, this);

    chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::MkChassisInfoFb>("chassis_info_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_msgs::Ultrasonic>("ultrasonic", 5);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 5);

    // 获取超声波参数
    getUlParam();
  }

  bool MkCanControl::WaitForCanFrame()
  {
    struct timeval tv;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);
    tv.tv_sec = 0;
    tv.tv_usec = 50000; // 50ms

    int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1)
    {
      ROS_ERROR("Error waiting for CAN frame: %s", std::strerror(errno));
      return false;
    }
    else if (ret == 0)
    {
      ROS_WARN("Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
  whether the can line is connected correctly, and whether the chassis is powered on.");
      return false;
    }
    else
    {
      return true;
    }
    return false;
  }

  void MkCanControl::IoCmdCallBack(const yhs_msgs::MkIoCmd::ConstPtr &io_cmd_msg)
  {
    const yhs_msgs::MkIoCmd msg = *io_cmd_msg;

    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = msg.io_cmd_enable;

    if (msg.io_cmd_upper_beam_headlamp)
      sendDataTemp[1] |= 0x20;

    if (msg.io_cmd_turn_lamp == 0)
      sendDataTemp[1] |= 0x00;
    if (msg.io_cmd_turn_lamp == 1)
      sendDataTemp[1] |= 0x04;
    if (msg.io_cmd_turn_lamp == 2)
      sendDataTemp[1] |= 0x08;

    sendDataTemp[2] = msg.io_cmd_speaker;

    count++;
    if (count > 15)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D7D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      ROS_ERROR("Send message failed : %s", std::strerror(errno));
    }
  }

  void MkCanControl::CtrlCmdCallBack(const yhs_msgs::MkCtrlCmd::ConstPtr &ctrl_cmd_msg)
  {
    yhs_msgs::MkCtrlCmd msg = *ctrl_cmd_msg;
    const unsigned short vel = msg.ctrl_cmd_velocity * 1000;
    const short angular = msg.ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;
    const unsigned char brake = msg.ctrl_cmd_Brake;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    if (msg.ctrl_cmd_velocity < 0)
      return;

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((vel & 0x0f) << 4));

    sendDataTemp[1] = (vel >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (vel >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

    sendDataTemp[3] = (angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((brake & 0x0f) << 4));

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

    sendDataTemp[5] = (brake >> 4) & 0x0f;

    count++;
    if (count > 15)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D2D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      ROS_ERROR("Send message failed : %s", std::strerror(errno));
    }
  }

  void MkCanControl::CmdCallBack(const geometry_msgs::Twist::ConstPtr &cmd_msg)
  {
    auto ctrl_cmd_msg = boost::make_shared<yhs_msgs::MkCtrlCmd>();
    ctrl_cmd_msg->ctrl_cmd_velocity = fabs(cmd_msg->linear.x);
    ctrl_cmd_msg->ctrl_cmd_steering = cmd_msg->angular.z / 3.14 * 180;
    ctrl_cmd_msg->ctrl_cmd_gear = cmd_msg->linear.x < 0 ? 2 : 4;
    ctrl_cmd_msg->ctrl_cmd_Brake = 0;

    CtrlCmdCallBack(ctrl_cmd_msg);
  }

  void MkCanControl::RecvData()
  {
    ros::Rate loop(100);

    static yhs_msgs::MkChassisInfoFb chassis_info_msg;
    while (ros::ok())
    {
      if (!WaitForCanFrame())
        continue;

      if (read(can_socket_, &recv_frames_, sizeof(recv_frames_)) >= 0)
      {
        switch (recv_frames_.can_id)
        {
        //
        case 0x98C4D2EF:
        {
          yhs_msgs::MkCtrlFb msg;

          msg.ctrl_fb_gear = 0x0f & recv_frames_.data[0];

          msg.ctrl_fb_velocity = static_cast<float>(static_cast<unsigned int>((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;

          msg.ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          msg.ctrl_fb_Brake = (recv_frames_.data[4] & 0xf0) >> 4 | (recv_frames_.data[5] & 0x0f) << 4;

          msg.ctrl_fb_mode = (recv_frames_.data[5] & 0x30) >> 4;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.header.stamp = ros::Time::now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_pub_.publish(chassis_info_msg);
            if (msg.ctrl_fb_gear == 2)
              msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
            OdomPub(msg.ctrl_fb_velocity, msg.ctrl_fb_steering / 180 * 3.1415);
          }

          break;
        }

        //
        case 0x98C4DAEF:
        {
          yhs_msgs::MkIoFb msg;

          msg.io_fb_enable = (recv_frames_.data[0] & 0x01) != 0;
          msg.io_fb_upper_beam_headlamp = (recv_frames_.data[1] & 0x02) != 0;

          msg.io_fb_turn_lamp = (0x0c & recv_frames_.data[1]) >> 2;

          msg.io_fb_braking_lamp = (0x10 & recv_frames_.data[1]) != 0;
          msg.io_fb_speaker = (0x01 & recv_frames_.data[2]) != 0;
          msg.io_fb_fm_impact_sensor = (0x02 & recv_frames_.data[3]) != 0;
          msg.io_fb_rm_impact_sensor = (0x10 & recv_frames_.data[3]) != 0;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.io_fb = msg;
          }

          break;
        }

        //
        case 0x98C4E1EF:
        {
          yhs_msgs::MkBmsInfoFb msg;

          msg.bms_info_voltage = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.bms_info_current = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          msg.bms_info_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frames_.data[5] << 8 | recv_frames_.data[4])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.bms_info_fb = msg;
          }

          break;
        }

        //
        case 0x98C4E2EF:
        {
          yhs_msgs::MkBmsFlagInfoFb msg;

          msg.bms_flag_info_soc = recv_frames_.data[0];

          msg.bms_flag_info_single_ov = (recv_frames_.data[1] & 0x01) != 0;
          msg.bms_flag_info_single_uv = (recv_frames_.data[1] & 0x02) != 0;
          msg.bms_flag_info_ov = (recv_frames_.data[1] & 0x04) != 0;
          msg.bms_flag_info_uv = (recv_frames_.data[1] & 0x08) != 0;
          msg.bms_flag_info_charge_ot = (recv_frames_.data[1] & 0x10) != 0;
          msg.bms_flag_info_charge_ut = (recv_frames_.data[1] & 0x20) != 0;
          msg.bms_flag_info_discharge_ot = (recv_frames_.data[1] & 0x40) != 0;
          msg.bms_flag_info_discharge_ut = (recv_frames_.data[1] & 0x80) != 0;

          msg.bms_flag_info_charge_oc = (recv_frames_.data[2] & 0x01) != 0;
          msg.bms_flag_info_discharge_oc = (recv_frames_.data[2] & 0x02) != 0;
          msg.bms_flag_info_short = (recv_frames_.data[2] & 0x04) != 0;
          msg.bms_flag_info_ic_error = (recv_frames_.data[2] & 0x08) != 0;
          msg.bms_flag_info_lock_mos = (recv_frames_.data[2] & 0x10) != 0;
          msg.bms_flag_info_charge_flag = (recv_frames_.data[2] & 0x20) != 0;

          msg.bms_flag_info_hight_temperature = static_cast<float>(static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4)) / 10;
          msg.bms_flag_info_low_temperature = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5])) / 10;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.bms_flag_info_fb = msg;
          }

          break;
        }

        //
        case 0x98C4EAEF:
        {
          yhs_msgs::MkVehDiagFb msg;

          msg.veh_fb_fault_level = 0x0f & recv_frames_.data[0];

          msg.veh_fb_auto_can_ctrl_cmd = (recv_frames_.data[0] & 0x10) != 0;
          msg.veh_fb_auto_io_can_cmd = (recv_frames_.data[0] & 0x20) != 0;
          msg.veh_fb_eps_dis_on_line = (recv_frames_.data[1] & 0x01) != 0;
          msg.veh_fb_eps_fault = (recv_frames_.data[1] & 0x02) != 0;
          msg.veh_fb_eps_mosf_et_ot = (recv_frames_.data[1] & 0x04) != 0;
          msg.veh_fb_eps_warning = (recv_frames_.data[1] & 0x08) != 0;
          msg.veh_fb_eps_dis_work = (recv_frames_.data[1] & 0x10) != 0;
          msg.veh_fb_eps_over_current = (recv_frames_.data[1] & 0x20) != 0;
          msg.veh_fb_ehb_ecu_fault = (recv_frames_.data[2] & 0x10) != 0;
          msg.veh_fb_ehb_dis_on_line = (recv_frames_.data[2] & 0x20) != 0;
          msg.veh_fb_ehb_work_model_fault = (recv_frames_.data[2] & 0x40) != 0;
          msg.veh_fb_ehb_dis_en = (recv_frames_.data[2] & 0x80) != 0;
          msg.veh_fb_ehb_anguler_fault = (recv_frames_.data[3] & 0x01) != 0;
          msg.veh_fb_ehb_ot = (recv_frames_.data[3] & 0x02) != 0;
          msg.veh_fb_ehb_power_fault = (recv_frames_.data[3] & 0x04) != 0;
          msg.veh_fb_ehb_sensor_abnomal = (recv_frames_.data[3] & 0x08) != 0;
          msg.veh_fb_ehb_motor_fault = (recv_frames_.data[3] & 0x10) != 0;
          msg.veh_fb_ehb_oil_press_sensor_fault = (recv_frames_.data[3] & 0x20) != 0;
          msg.veh_fb_ehb_oil_fault = (recv_frames_.data[3] & 0x40) != 0;
          msg.veh_fb_drv_mcu_dis_on_line = (recv_frames_.data[4] & 0x01) != 0;
          msg.veh_fb_drv_mcu_ot = (recv_frames_.data[4] & 0x02) != 0;
          msg.veh_fb_drv_mcu_ov = (recv_frames_.data[4] & 0x04) != 0;
          msg.veh_fb_drv_mcu_uv = (recv_frames_.data[4] & 0x08) != 0;
          msg.veh_fb_drv_mcu_short = (recv_frames_.data[4] & 0x10) != 0;
          msg.veh_fb_drv_mcu_scram = (recv_frames_.data[4] & 0x20) != 0;
          msg.veh_fb_drv_mcu_hall = (recv_frames_.data[4] & 0x40) != 0;
          msg.veh_fb_drv_mcu_mosf_ef = (recv_frames_.data[4] & 0x80) != 0;
          msg.veh_fb_aux_bms_dis_on_line = (recv_frames_.data[5] & 0x10) != 0;
          msg.veh_fb_aux_scram = recv_frames_.data[5] & 0x20;
          msg.veh_fb_aux_remote_close = recv_frames_.data[5] & 0x40;
          msg.veh_fb_aux_remote_dis_on_line = recv_frames_.data[5] & 0x80;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.veh_diag_fb = msg;
          }

          break;
        }

          // ultrasonic
          static unsigned short ultra_data[8] = {0};
        case 0x98C4E8EF:
        {
          ultra_data[0] = (unsigned short)((recv_frames_.data[1] & 0x0f) << 8 | recv_frames_.data[0]);
          ultra_data[1] = (unsigned short)(recv_frames_.data[2] << 4 | ((recv_frames_.data[1] & 0xf0) >> 4));

          ultra_data[2] = (unsigned short)((recv_frames_.data[4] & 0x0f) << 8 | recv_frames_.data[3]);
          ultra_data[3] = (unsigned short)(recv_frames_.data[5] << 4 | ((recv_frames_.data[4] & 0xf0) >> 4));
          break;
        }

        case 0x98C4E9EF:
        {
          ultra_data[4] = (unsigned short)((recv_frames_.data[1] & 0x0f) << 8 | recv_frames_.data[0]);
          ultra_data[5] = (unsigned short)(recv_frames_.data[2] << 4 | ((recv_frames_.data[1] & 0xf0) >> 4));

          ultra_data[6] = (unsigned short)((recv_frames_.data[4] & 0x0f) << 8 | recv_frames_.data[3]);
          ultra_data[7] = (unsigned short)(recv_frames_.data[5] << 4 | ((recv_frames_.data[4] & 0xf0) >> 4));

          yhs_msgs::Ultrasonic ultra_msg;

          sensor_msgs::LaserScan scan;
          scan.header.stamp = ros::Time::now();
          scan.header.frame_id = "ul_link";
          scan.angle_min = -3.14;
          scan.angle_max = 3.14;
          scan.angle_increment = 0.175;
          scan.scan_time = 1.0 / 10;
          scan.time_increment = 1 / 10 / 36;
          scan.range_min = 0.0;
          scan.range_max = 5.0;

          scan.ranges.resize(36);
          scan.intensities.resize(36);

          for (unsigned int i = 0; i < 36; ++i)
          {
            scan.ranges[i] = std::numeric_limits<float>::infinity();
            scan.intensities[i] = 0.0;
          }

          for (unsigned int i = 0; i < ul_pose_.size(); ++i)
          {
            unsigned char yaw_index = ul_pose_[i].yaw / 360 * 36;
            // unsigned char yaw_index = (std::atan2( ul_pose_[i].y, ul_pose_[i].x ) + 3.14) / 6.28 * 36;

            unsigned char d_index = ul_pose_[i].d_index;
            unsigned char p_index = ul_pose_[i].p_index;

            float range = std::hypot(ul_pose_[i].x, ul_pose_[i].y) + (float)ultra_data[d_index] / 1000;

            if (range > ul_pose_[i].max + std::hypot(ul_pose_[i].x, ul_pose_[i].y))
              range = std::numeric_limits<float>::infinity();

            if (range < ul_pose_[i].min + std::hypot(ul_pose_[i].x, ul_pose_[i].y))
              range = std::numeric_limits<float>::infinity();

            scan.ranges[yaw_index] = range;

            if (p_index == 0)
            {
              ultra_msg.front_right = ultra_data[d_index];
            }
            else if (p_index == 1)
            {
              ultra_msg.front_left = ultra_data[d_index];
            }
            else if (p_index == 2)
            {
              ultra_msg.left_front = ultra_data[d_index];
            }
            else if (p_index == 3)
            {
              ultra_msg.left_rear = ultra_data[d_index];
            }
            else if (p_index == 4)
            {
              ultra_msg.rear_left = ultra_data[d_index];
            }
            else if (p_index == 5)
            {
              ultra_msg.rear_right = ultra_data[d_index];
            }
            else if (p_index == 6)
            {
              ultra_msg.right_rear = ultra_data[d_index];
            }
            else if (p_index == 7)
            {
              ultra_msg.right_front = ultra_data[d_index];
            }
          }
        }
        default:
          break;
        }
      }
    }
  }

  void MkCanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Quaternion quaternion;
    tf2::fromMsg(imu_data_msg->orientation, quaternion);

    imu_yaw_ = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void MkCanControl::OdomPub(const float velocity, const float steering)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    double x_mid = 0.0;
    double y_mid = 0.0;

    static tf2_ros::TransformBroadcaster odom_broadcaster;

    static ros::Time last_time = ros::Time::now();
    ros::Time current_time;

    double vx = velocity;
    double vth = vx * tan(steering) / wheel_base_;

    current_time = ros::Time::now();

    std::lock_guard<std::mutex> lock(mutex_);

    // 使用IMU的角度
    th = imu_yaw_;

    double dt = (current_time - last_time).toSec();

    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    // th += delta_th;

    x_mid = x + wheel_base_ / 2 * cos(th);
    y_mid = y + wheel_base_ / 2 * sin(th);

    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrame_;
    odom_trans.child_frame_id = baseFrame_;

    odom_trans.transform.translation.x = x_mid;
    odom_trans.transform.translation.y = y_mid;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // 是否发布tf转换
    if (tfUsed_)
      odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odomFrame_;

    // set the position
    odom.pose.pose.position.x = x_mid;
    odom.pose.pose.position.y = y_mid;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = baseFrame_;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[35] = 0.2;

    odom.pose.covariance[14] = 1e10;
    odom.pose.covariance[21] = 1e10;
    odom.pose.covariance[28] = 1e10;

    // publish the message
    odom_pub_.publish(odom);

    last_time = current_time;
  }

  bool MkCanControl::Run()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
      ROS_ERROR("Failed to open socket: %s", strerror(errno));
      return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
      ROS_ERROR("Failed to get interface index: %s ==> %s", strerror(errno), if_name_.c_str());
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      ROS_ERROR("Failed to bind socket: %s", strerror(errno));
      return false;
    }

    thread_ = boost::thread(&MkCanControl::RecvData, this);

    return true;
  }

  void MkCanControl::Stop()
  {
    if (can_socket_ >= 0)
    {
      close(can_socket_);
      can_socket_ = -1;
    }

    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  MkCanControl::~MkCanControl()
  {
  }
}