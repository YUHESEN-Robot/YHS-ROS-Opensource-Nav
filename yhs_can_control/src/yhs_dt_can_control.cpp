#include "yhs_dt_can_control.h"

namespace can_control
{

  DgtCanControl::DgtCanControl()
  {
    ros::NodeHandle private_node("~");

    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
    private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
    private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
    private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));

    ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::DgtCtrlCmd>("ctrl_cmd", 5, &DgtCanControl::CtrlCmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_msgs::DgtIoCmd>("io_cmd", 5, &DgtCanControl::IoCmdCallBack, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smoother_cmd_vel", 5, &DgtCanControl::CmdCallBack, this);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &DgtCanControl::ImuDataCallBack, this);

    chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::DgtChassisInfoFb>("chassis_info_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

    // 获取超声波参数
    getUlParam();
  }

  bool DgtCanControl::WaitForCanFrame()
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

  void DgtCanControl::IoCmdCallBack(const yhs_msgs::DgtIoCmd::ConstPtr &io_cmd_msg)
  {
    const yhs_msgs::DgtIoCmd msg = *io_cmd_msg;

    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    std::lock_guard<std::mutex> lock(mutex_);

    if (msg.io_cmd_lamp_ctrl)
      sendDataTemp[0] |= 0x01;
    if (msg.io_cmd_unlock)
      sendDataTemp[0] |= 0x02;

    if (msg.io_cmd_lower_beam_headlamp)
      sendDataTemp[1] |= 0x01;
    if (msg.io_cmd_upper_beam_headlamp)
      sendDataTemp[1] |= 0x02;

    if (msg.io_cmd_turn_lamp == 0)
      sendDataTemp[1] |= 0x00;
    if (msg.io_cmd_turn_lamp == 1)
      sendDataTemp[1] |= 0x04;
    if (msg.io_cmd_turn_lamp == 2)
      sendDataTemp[1] |= 0x08;

    if (msg.io_cmd_braking_lamp)
      sendDataTemp[1] |= 0x10;
    if (msg.io_cmd_clearance_lamp)
      sendDataTemp[1] |= 0x20;
    if (msg.io_cmd_fog_lamp)
      sendDataTemp[1] |= 0x40;

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

  void DgtCanControl::CtrlCmdCallBack(const yhs_msgs::DgtCtrlCmd::ConstPtr &ctrl_cmd_msg)
  {
    yhs_msgs::DgtCtrlCmd msg = *ctrl_cmd_msg;
    const short linear = msg.ctrl_cmd_linear * 1000;
    const short angular = msg.ctrl_cmd_angular * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    std::lock_guard<std::mutex> lock(mutex_);

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

    sendDataTemp[1] = (linear >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

    sendDataTemp[3] = (angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

    count++;
    if (count > 15)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D1D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      ROS_ERROR("Send message failed : %s", std::strerror(errno));
    }
  }

  void DgtCanControl::CmdCallBack(const geometry_msgs::Twist::ConstPtr &cmd_msg)
  {

    auto ctrl_cmd_msg = boost::make_shared<yhs_msgs::DgtCtrlCmd>();
    ctrl_cmd_msg->ctrl_cmd_linear = cmd_msg->linear.x;
    ctrl_cmd_msg->ctrl_cmd_angular = cmd_msg->angular.z / 3.14 * 180;
    ctrl_cmd_msg->ctrl_cmd_gear = 3;

    CtrlCmdCallBack(ctrl_cmd_msg);
  }

  void DgtCanControl::RecvData()
  {
    ros::Rate loop(100);

    static yhs_msgs::DgtChassisInfoFb chassis_info_msg;
    while (ros::ok())
    {
      if (!WaitForCanFrame())
        continue;

      if (read(can_socket_, &recv_frames_, sizeof(recv_frames_)) >= 0)
      {
        switch (recv_frames_.can_id)
        {
        case 0x98C4D1EF:
        {
          yhs_msgs::DgtCtrlFb msg;

          msg.ctrl_fb_target_gear = recv_frames_.data[0] & 0x0f;
          msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frames_.data[2] & 0x0f) << 12) | (recv_frames_.data[1] << 4) | ((recv_frames_.data[0] & 0xf0) >> 4)) / 1000.0;
          msg.ctrl_fb_angular = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12) | (recv_frames_.data[3] << 4) | ((recv_frames_.data[2] & 0xf0) >> 4)) / 100.0;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            chassis_info_msg.header.stamp = ros::Time::now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_pub_.publish(chassis_info_msg);

            OdomPub(msg.ctrl_fb_linear * 1.0, msg.ctrl_fb_angular / 180 * 3.14);
          }

          break;
        }

        case 0x98C4D7EF:
        {
          break;
        }

        //
        case 0x98C4D8EF:
        {
          break;
        }

        // io反馈
        case 0x98C4DAEF:
        {
          yhs_msgs::DgtIoFb msg;

          msg.io_fb_lamp_ctrl = (recv_frames_.data[0] & 0x01) != 0;
          msg.io_fb_unlock = (recv_frames_.data[1] & 0x02) != 0;
          msg.io_fb_lower_beam_headlamp = (recv_frames_.data[1] & 0x01) != 0;
          msg.io_fb_upper_beam_headlamp = (recv_frames_.data[1] & 0x02) != 0;
          msg.io_fb_turn_lamp = (recv_frames_.data[1] & 0xc0) >> 2;
          msg.io_fb_braking_lamp = (recv_frames_.data[1] & 0x10) != 0;
          msg.io_fb_clearance_lamp = (recv_frames_.data[1] & 0x20) != 0;
          msg.io_fb_fog_lamp = (recv_frames_.data[1] & 0x40) != 0;
          msg.io_fb_speaker = (recv_frames_.data[2] & 0x01) != 0;
          msg.io_fb_fl_impact_sensor = (recv_frames_.data[3] & 0x01) != 0;
          msg.io_fb_fm_impact_sensor = (recv_frames_.data[3] & 0x02) != 0;
          msg.io_fb_fr_impact_sensor = (recv_frames_.data[3] & 0x04) != 0;
          msg.io_fb_rl_impact_sensor = (recv_frames_.data[3] & 0x08) != 0;
          msg.io_fb_rm_impact_sensor = (recv_frames_.data[3] & 0x10) != 0;
          msg.io_fb_rr_impact_sensor = (recv_frames_.data[3] & 0x20) != 0;
          msg.io_fb_fl_drop_sensor = (recv_frames_.data[4] & 0x01) != 0;
          msg.io_fb_fm_drop_sensor = (recv_frames_.data[4] & 0x02) != 0;
          msg.io_fb_fr_drop_sensor = (recv_frames_.data[4] & 0x04) != 0;
          msg.io_fb_rl_drop_sensor = (recv_frames_.data[4] & 0x08) != 0;
          msg.io_fb_rm_drop_sensor = (recv_frames_.data[4] & 0x10) != 0;
          msg.io_fb_rr_drop_sensor = (recv_frames_.data[4] & 0x20) != 0;
          msg.io_fb_estop = (recv_frames_.data[5] & 0x01) != 0;
          msg.io_fb_joypad_ctrl = (recv_frames_.data[5] & 0x02) != 0;
          msg.io_fb_charge_state = (recv_frames_.data[5] & 0x04) != 0;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.io_fb = msg;
          }

          break;
        }

        // bms反馈
        case 0x98C4E1EF:
        {
          yhs_msgs::DgtBmsFb msg;

          msg.bms_fb_voltage = static_cast<float>(reinterpret_cast<unsigned short &>(recv_frames_.data[0])) / 100;
          msg.bms_fb_current = static_cast<float>(reinterpret_cast<short &>(recv_frames_.data[2])) / 100;
          msg.bms_fb_remaining_capacity = static_cast<float>(reinterpret_cast<unsigned short &>(recv_frames_.data[4])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.bms_fb = msg;
          }

          break;
        }

        // bms_flag反馈
        case 0x98C4E2EF:
        {
          yhs_msgs::DgtBmsFlagFb msg;

          msg.bms_flag_fb_soc = recv_frames_.data[0];
          msg.bms_flag_fb_single_ov = (0x01 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_single_uv = (0x02 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_ov = (0x04 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_uv = (0x08 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_charge_ot = (0x10 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_charge_ut = (0x20 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_discharge_ot = (0x40 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_discharge_ut = (0x80 & recv_frames_.data[1]) != 0;
          msg.bms_flag_fb_charge_oc = (0x01 & recv_frames_.data[2]) != 0;
          msg.bms_flag_fb_discharge_oc = (0x02 & recv_frames_.data[2]) != 0;
          msg.bms_flag_fb_short = (0x04 & recv_frames_.data[2]) != 0;
          msg.bms_flag_fb_ic_error = (0x08 & recv_frames_.data[2]) != 0;
          msg.bms_flag_fb_lock_mos = (0x10 & recv_frames_.data[2]) != 0;
          msg.bms_flag_fb_charge_flag = (0x20 & recv_frames_.data[2]) != 0;

          const float kTemperatureConversionFactor = 0.1;
          msg.bms_flag_fb_hight_temperature = static_cast<float>((static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4))) * kTemperatureConversionFactor;
          msg.bms_flag_fb_low_temperature = static_cast<float>((static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5]))) * kTemperatureConversionFactor;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.bms_flag_fb = msg;
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

  void DgtCanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Quaternion quaternion;
    tf2::fromMsg(imu_data_msg->orientation, quaternion);

    imu_yaw_ = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void DgtCanControl::OdomPub(const float linear, const float angular)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    static double lastYaw = 0;

    static ros::Time last_time = ros::Time::now();
    ros::Time current_time;

    static tf::TransformBroadcaster odom_broadcaster;

    double vx = linear;
    double vth = angular;

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
    //    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrame_;
    odom_trans.child_frame_id = baseFrame_;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    if (tfUsed_)
      odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odomFrame_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

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

    odom_pub_.publish(odom);

    last_time = current_time;
  }

  bool DgtCanControl::Run()
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

    thread_ = boost::thread(&DgtCanControl::RecvData, this);

    return true;
  }

  void DgtCanControl::Stop()
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

  DgtCanControl::~DgtCanControl()
  {
  }

}