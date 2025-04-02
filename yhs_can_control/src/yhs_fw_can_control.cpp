#include "yhs_fw_can_control.h"

namespace can_control
{

  FwCanControl::FwCanControl()
  {
    ros::NodeHandle private_node("~");

    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
    private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
    private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
    private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));

    ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::FwCtrlCmd>("fw_ctrl_cmd", 5, &FwCanControl::CtrlCmdCallBack, this);
    steering_ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::FwSteeringCtrlCmd>("fw_steering_ctrl_cmd", 5, &FwCanControl::SteeringCtrlCmdCallBack, this);    
    io_cmd_sub_ = nh_.subscribe<yhs_msgs::FwIoCmd>("io_cmd", 5, &FwCanControl::IoCmdCallBack, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smoother_cmd_vel", 5, &FwCanControl::CmdCallBack, this);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &FwCanControl::ImuDataCallBack, this);

    chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::FwChassisInfoFb>("chassis_info_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
    angular_pub_ = nh_.advertise<std_msgs::Float32>("angular", 5);
    linear_pub_ = nh_.advertise<std_msgs::Float32>("linear", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_msgs::Ultrasonic>("ultrasonic", 5);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 5);
  }

  bool FwCanControl::WaitForCanFrame()
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

  void FwCanControl::IoCmdCallBack(const yhs_msgs::FwIoCmd::ConstPtr &io_cmd_msg)
  {
    const yhs_msgs::FwIoCmd msg = *io_cmd_msg;

    static unsigned char count = 0;
    std::lock_guard<std::mutex> lock(mutex_);

    unsigned char sendDataTemp[8] = {0};

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

    //		sendDataTemp[3] = msg.io_cmd_wireless_charge;

    count++;
    if (count == 16)
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

  void FwCanControl::CtrlCmdCallBack(const yhs_msgs::FwCtrlCmd::ConstPtr &ctrl_cmd_msg)
  {
    yhs_msgs::FwCtrlCmd msg = *ctrl_cmd_msg;

    const unsigned char gear = msg.ctrl_cmd_gear;
    const short ctrl_cmd_x_linear = msg.ctrl_cmd_x_linear * 1000;
    const short ctrl_cmd_z_angular = msg.ctrl_cmd_z_angular * 100;
    const short ctrl_cmd_y_linear = msg.ctrl_cmd_y_linear * 1000;

    static unsigned char count = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((ctrl_cmd_x_linear & 0x0f) << 4));

    sendDataTemp[1] = (ctrl_cmd_x_linear >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (ctrl_cmd_x_linear >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((ctrl_cmd_z_angular & 0x0f) << 4));

    sendDataTemp[3] = (ctrl_cmd_z_angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (ctrl_cmd_z_angular >> 12));

    sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((ctrl_cmd_y_linear & 0x0f) << 4));

    sendDataTemp[5] = (ctrl_cmd_y_linear >> 4) & 0xff;

    sendDataTemp[6] = sendDataTemp[6] | (0x0f & (ctrl_cmd_y_linear >> 12));

    count++;

    if (count == 16)
      count = 0;

    sendDataTemp[6] = sendDataTemp[6] | (count << 4);

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

  void FwCanControl::SteeringCtrlCmdCallBack(const yhs_msgs::FwSteeringCtrlCmd::ConstPtr &steering_ctrl_cmd)
  {
    yhs_msgs::FwSteeringCtrlCmd msg = *steering_ctrl_cmd;

    const unsigned char gear = msg.ctrl_cmd_gear;
    const short steering_ctrl_cmd_velocity = msg.steering_ctrl_cmd_velocity * 1000;
    const short steering_ctrl_cmd_steering = msg.steering_ctrl_cmd_steering * 100;

    static unsigned char count = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((steering_ctrl_cmd_velocity & 0x0f) << 4));

    sendDataTemp[1] = (steering_ctrl_cmd_velocity >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (steering_ctrl_cmd_velocity >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((steering_ctrl_cmd_steering & 0x0f) << 4));

    sendDataTemp[3] = (steering_ctrl_cmd_steering >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (steering_ctrl_cmd_steering >> 12));

    count ++;

    if (count == 16)
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

  void FwCanControl::CmdCallBack(const geometry_msgs::Twist::ConstPtr &cmd_msg)
  {
    auto ctrl_cmd_msg = boost::make_shared<yhs_msgs::FwCtrlCmd>();
    ctrl_cmd_msg->ctrl_cmd_x_linear = cmd_msg->linear.x;
    ctrl_cmd_msg->ctrl_cmd_z_angular = cmd_msg->angular.z / 3.14 * 180;
    ctrl_cmd_msg->ctrl_cmd_gear = 6;
    ctrl_cmd_msg->ctrl_cmd_y_linear = 0;

    CtrlCmdCallBack(ctrl_cmd_msg);
  }

  void FwCanControl::RecvData()
  {
    static yhs_msgs::FwChassisInfoFb chassis_info_msg;
    while (ros::ok())
    {
      if (!WaitForCanFrame())
        continue;

      if (read(can_socket_, &recv_frames_, sizeof(recv_frames_)) >= 0)
      {
        switch (recv_frames_.can_id)
        {
        //
        case 0x98C4D1EF:
        {
          yhs_msgs::FwCtrlFb msg;

          msg.ctrl_fb_gear = 0x0f & recv_frames_.data[0];

          msg.ctrl_fb_x_linear = static_cast<float>(static_cast<short>((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;

          msg.ctrl_fb_z_angular = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          msg.ctrl_fb_y_linear = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 12 | recv_frames_.data[5] << 4 | (recv_frames_.data[4] & 0xf0) >> 4)) / 1000;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.header.stamp = ros::Time::now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_pub_.publish(chassis_info_msg);

            OdomPub(msg.ctrl_fb_x_linear, msg.ctrl_fb_z_angular / 180 * 3.14, msg.ctrl_fb_gear, msg.ctrl_fb_y_linear / 180 * 3.14);

            std_msgs::Float32 linear_msg;
            linear_msg.data = msg.ctrl_fb_x_linear;
            linear_pub_.publish(linear_msg);
          }

          break;
        }

        //
        case 0x98C4D2EF:
        {
          yhs_msgs::FwSteeringCtrlFb msg;

          msg.steering_ctrl_fb_gear = 0x0f & recv_frames_.data[0];

          msg.steering_ctrl_fb_velocity = (float)((short)((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;

          msg.steering_ctrl_fb_steering = (float)((short)((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.steering_ctrl_fb = msg;
          }

          break;
        }

        // io反馈
        case 0x98C4DAEF:
        {
          yhs_msgs::FwIoFb msg;
          
          msg.io_fb_lamp_ctrl = (0x01 & recv_frames_.data[0]);
          msg.io_fb_unlock = (0x02 & recv_frames_.data[1]);
          msg.io_fb_lower_beam_headlamp = (0x01 & recv_frames_.data[1]);
          msg.io_fb_upper_beam_headlamp = (0x02 & recv_frames_.data[1]);
          msg.io_fb_turn_lamp = (0x0c & recv_frames_.data[1]) >> 2;
          msg.io_fb_braking_lamp = (0x10 & recv_frames_.data[1]);
          msg.io_fb_clearance_lamp = (0x20 & recv_frames_.data[1]);
          msg.io_fb_fog_lamp = (0x40 & recv_frames_.data[1]);
          msg.io_fb_speaker = (0x01 & recv_frames_.data[2]);
          msg.io_fb_fl_impact_sensor = (0x01 & recv_frames_.data[3]);
          msg.io_fb_fm_impact_sensor = (0x02 & recv_frames_.data[3]);
          msg.io_fb_fr_impact_sensor = (0x04 & recv_frames_.data[3]);
          msg.io_fb_rl_impact_sensor = (0x08 & recv_frames_.data[3]);
          msg.io_fb_rm_impact_sensor = (0x10 & recv_frames_.data[3]);
          msg.io_fb_rr_impact_sensor = (0x20 & recv_frames_.data[3]);
          msg.io_fb_fl_drop_sensor = (0x01 & recv_frames_.data[4]);
          msg.io_fb_fm_drop_sensor = (0x02 & recv_frames_.data[4]);
          msg.io_fb_fr_drop_sensor = (0x04 & recv_frames_.data[4]);
          msg.io_fb_rl_drop_sensor = (0x08 & recv_frames_.data[4]);
          msg.io_fb_rm_drop_sensor = (0x10 & recv_frames_.data[4]);
          msg.io_fb_rr_drop_sensor = (0x20 & recv_frames_.data[4]);
          msg.io_fb_estop = (0x01 & recv_frames_.data[5]);
          msg.io_fb_joypad_ctrl = (0x02 & recv_frames_.data[5]);
          msg.io_fb_charge_state = (0x04 & recv_frames_.data[5]);

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            chassis_info_msg.io_fb = msg;
          }

          break;
        }

        //
        case 0x98C4DCEF:
        {
          float front_angle_fb_l = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          float front_angle_fb_r = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {
            std_msgs::Float32 angular_msg;
            angular_msg.data = front_angle_fb_l;
            angular_pub_.publish(angular_msg);
          }

          break;
        }

        // bms反馈
        case 0x98C4E1EF:
        {
          yhs_msgs::FwBmsFb msg;
          msg.bms_fb_voltage = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.bms_fb_current = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          msg.bms_fb_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frames_.data[5] << 8 | recv_frames_.data[4])) / 100;

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
          yhs_msgs::FwBmsFlagFb msg;
          msg.bms_flag_fb_soc = recv_frames_.data[0];

          msg.bms_flag_fb_single_ov = (0x01 & recv_frames_.data[1]);
          msg.bms_flag_fb_single_uv = (0x02 & recv_frames_.data[1]);
          msg.bms_flag_fb_ov = (0x04 & recv_frames_.data[1]);
          msg.bms_flag_fb_uv = (0x08 & recv_frames_.data[1]);
          msg.bms_flag_fb_charge_ot = (0x10 & recv_frames_.data[1]);
          msg.bms_flag_fb_charge_ut = (0x20 & recv_frames_.data[1]);
          msg.bms_flag_fb_discharge_ot = (0x40 & recv_frames_.data[1]);
          msg.bms_flag_fb_discharge_ut = (0x80 & recv_frames_.data[1]);

          msg.bms_flag_fb_charge_oc = (0x01 & recv_frames_.data[2]);
          msg.bms_flag_fb_discharge_oc = (0x02 & recv_frames_.data[2]);
          msg.bms_flag_fb_short = (0x04 & recv_frames_.data[2]);
          msg.bms_flag_fb_ic_error = (0x08 & recv_frames_.data[2]);
          msg.bms_flag_fb_lock_mos = (0x10 & recv_frames_.data[2]);
          msg.bms_flag_fb_charge_flag = (0x20 & recv_frames_.data[2]);

          msg.bms_flag_fb_hight_temperature = static_cast<float>(static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4)) / 10;

          msg.bms_flag_fb_low_temperature = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5])) / 10;

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

            unsigned char d_index = i;//ul_pose_[i].d_index;
            unsigned char p_index = ul_pose_[i].p_index;

            float range = std::hypot(ul_pose_[i].x, ul_pose_[i].y) + (float)ultra_data[p_index] / 1000;

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
          ultrasonic_pub_.publish(ultra_msg);
        }
        default:
          break;
        }
      }
    }
  }

  void FwCanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Quaternion quaternion;
    tf2::fromMsg(imu_data_msg->orientation, quaternion);

    imu_yaw_ = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void FwCanControl::OdomPub(const float linear, const float angular, const unsigned char gear, const float slipangle)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    static double lastYaw = 0;

    static tf2_ros::TransformBroadcaster odom_broadcaster;

    static ros::Time last_time = ros::Time::now();
    ros::Time current_time;

    double vx = linear;
    double vth = angular;

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

		static bool imu_published = false;
		if(!imu_published)
		{
			imu_published = hasPublisher("/imu_data");
		}
		else
		{
			ROS_INFO_ONCE("Get imu_data");
		}
		
    if(gear == 6)
    {
      x += vx * cos(th) * dt;
      y += vx * sin(th) * dt;

      if (!imu_published)
      {
        th += vth * dt;
      }
      else
      {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_;
      }
    }

    else if(gear == 7)
    {
      double theta = th + slipangle;

      if(vx >= 0)
      {
        x += vx * cos(theta) * dt;
        y += vx * sin(theta) * dt;
      }
      else
      {
        theta = th + slipangle + M_PI;
        x += -vx * cos(theta) * dt;
        y += -vx * sin(theta) * dt;
      }

      if (!imu_published)
      {
      }
      else
      {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_;
      }     
    }


    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, th);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

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

    odom_pub_.publish(odom);

    last_time = current_time;
  }

  bool FwCanControl::Run()
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

    thread_ = boost::thread(&FwCanControl::RecvData, this);

    return true;
  }

  void FwCanControl::Stop()
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

  FwCanControl::~FwCanControl()
  {
  }
}
