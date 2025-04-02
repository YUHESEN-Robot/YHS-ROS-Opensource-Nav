#include <recharge/recharge_node.h>

RechargeNode::RechargeNode() : tf2_listener_(tf2_buffer_)
{

  ros::NodeHandle private_nh("/recharge");

  private_nh.param("yaw_delta_little", yaw_delta_little_, 0.0);
  private_nh.param("yaw_delta", yaw_delta_, 0.0);
  private_nh.param("yaw_delta_big", yaw_delta_big_, 0.0);
  private_nh.param("dist_delta_little", dist_delta_little_, 0.0);
  private_nh.param("dist_delta", dist_delta_, 0.0);
  private_nh.param("dist_delta_big", dist_delta_big_, 0.0);
  private_nh.param("angular_vel", angular_vel_, 0.0);
  private_nh.param("linear_vel", linear_vel_, 0.0);
  private_nh.param("slipangle", slipangle_, 0.0);
  private_nh.param("control_frequency", control_frequency_, 33);

  private_nh.param("recharge_dist", recharge_dist_, 0.3);

  private_nh.param("car_half_length", car_half_length_, 0.3);
  private_nh.param("car_half_width", car_half_width_, 0.3);

  private_nh.param("shortest_dist_y", shortest_dist_y_, 0.2);

  private_nh.param("/common/chassis_type", chassis_type_, std::string("FW"));

  if (chassis_type_ == "FW")
  {
    fw_chassis_info_sub_ = nh_.subscribe<yhs_msgs::FwChassisInfoFb>("chassis_info_fb", 10, &RechargeNode::FwChassisCB, this);
  }

  scan1_sub_ = nh_.subscribe("scan1", 1, &RechargeNode::scan1Callback, this);
  scan2_sub_ = nh_.subscribe("scan2", 1, &RechargeNode::scan2Callback, this);

  angular_fb_sub_ = nh_.subscribe("angular", 1, &RechargeNode::angularCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ctrl_cmd_pub_ = nh_.advertise<yhs_msgs::FwSteeringCtrlCmd>("fw_steering_ctrl_cmd", 1);

  recharge_srv_ = nh_.advertiseService("recharge", &RechargeNode::rechargeCallback, this);

  dis_recharge_srv_ = nh_.advertiseService("dis_recharge", &RechargeNode::disRechargeCallback, this);
}

// FW底盘信息
void RechargeNode::FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_charge_done_ = msg->io_fb.io_fb_charge_state;
}

// 角度订阅
void RechargeNode::angularCallback(const std_msgs::Float32::ConstPtr &angular_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  angular_fb_ = angular_msg->data;
}

// 前摄像头
void RechargeNode::scan1Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  bool front_has_obs = false;
  sensor_msgs::PointCloud cloud;
  try
  {
    projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, tf_listener_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("Transform warning: %s", ex.what());
    return;
  }

  for (const auto &point : cloud.points)
  {
    if (point.x <= car_half_length_ + 0.4 && point.x > car_half_length_ && fabs(point.y) <= car_half_width_)
    {
      front_has_obs = true;
      // ROS_WARN("front_has_obs : %d",front_has_obs);
      break;
    }
  }
  front_has_obs_ = front_has_obs;
}

// 后摄像头
void RechargeNode::scan2Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  bool rear_has_obs = false;
  sensor_msgs::PointCloud cloud;
  try
  {
    projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, tf_listener_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("Transform warning: %s", ex.what());
    return;
  }

  for (const auto &point : cloud.points)
  {
    if (point.x >= -car_half_length_ - 0.3 && point.x < -car_half_length_ && fabs(point.y) <= car_half_width_)
    {
      rear_has_obs_ = true;
      break;
    }
  }
  rear_has_obs_ = rear_has_obs;
}

bool RechargeNode::goToTempPoint(const geometry_msgs::PoseStamped temp_point)
{

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = temp_point.pose.position.x;
  goal.target_pose.pose.position.y = temp_point.pose.position.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = temp_point.pose.orientation;

  ROS_INFO("Start Recharge...");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return true;
  }

  return false;
}

bool RechargeNode::getRobotPose()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    // 获取 base_link 到 map 的变换关系
    transformStamped = tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // 提取坐标和偏航角
  robot_pose_.x = transformStamped.transform.translation.x;
  robot_pose_.y = transformStamped.transform.translation.y;

  // 使用四元数转换为欧拉角
  tf2::Quaternion quat;
  tf2::convert(transformStamped.transform.rotation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  robot_pose_.yaw = yaw;

  return true;
}

bool RechargeNode::transformPose(
    const std::string frame,
    const geometry_msgs::PoseStamped &in_pose,
    geometry_msgs::PoseStamped &out_pose)
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_listener_.transformPose(frame, in_pose, out_pose);
    out_pose.header.frame_id = frame;
    return true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

double RechargeNode::distanceToLine(Vector2d A, Vector2d dir, Vector2d B)
{
  Vector2d AB = B - A;
  Vector2d u = dir.normalized();
  double AP = AB.dot(u);
  double dist = sqrt(AB.dot(AB) - AP * AP);
  return dist;
}

double RechargeNode::pointToLineDistance(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) 
{
    // 计算向量AB和AC
    Eigen::Vector2d AB = B - A;
    Eigen::Vector2d AC = C - A;

    // 计算AB的单位向量
    Eigen::Vector2d AB_unit = AB.normalized();

    // 计算AC在AB方向上的投影长度
    double projection_length = AC.dot(AB_unit);

    // 计算投影点P
    Eigen::Vector2d P = A + projection_length * AB_unit;

    // 计算C到P的距离
    double distance = (C - P).norm();
    return distance;
}

void RechargeNode::publishCmdVel(const double linear_vel, const double angular_vel)
{
  // 发布控制指令
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;

  cmd_vel_pub_.publish(cmd_vel);
}

void RechargeNode::publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle)
{
  // 发布控制指令
  yhs_msgs::FwSteeringCtrlCmd msg;

  msg.steering_ctrl_cmd_velocity = linear_vel;
  msg.ctrl_cmd_gear = gear;
  msg.steering_ctrl_cmd_steering = slipangle;

  ctrl_cmd_pub_.publish(msg);
}

bool RechargeNode::disRechargeCallback(recharge::DisRecharge::Request &req, recharge::DisRecharge::Response &res)
{
  int error_code = 1;

  static bool start = false;

  // 是否处于充电中
  if (!is_charge_done_ && !start)
  {
    error_code = -2;
  }
  else
  {
    ROS_INFO("Start DisRecharge...");
    ros::Rate loop_rate(control_frequency_);
    while (ros::ok())
    {
      double linear = 0.1;
      double angular = 0.0;

      // 获取机器人当前坐标
      if (!getRobotPose())
      {
        error_code = -5;
        break;
      }

      double dx = robot_pose_.x - recharge_pose_.x;
      double dy = robot_pose_.y - recharge_pose_.y;
      double temp_dist = std::sqrt(dx * dx + dy * dy);

      // 判断是否脱桩成功
      if (temp_dist > 0.3)
      {
        error_code = 1;
        break;
      }

      // 前面有障碍物
      if (front_has_obs_)
      {
        linear = 0.0;
      }

      start = true;
      publishCmdVel(linear, angular);
      loop_rate.sleep();
    }
  }

  start = false;
  publishCmdVel(0.0, 0.0);
  res.result = error_code;
  if (error_code != 1)
  {
    ROS_ERROR("Disrecharge failed! error code : %d", error_code);
  }
  return true;
}

bool RechargeNode::rechargeCallback(recharge::Recharge::Request &req, recharge::Recharge::Response &res)
{
  // 先复位
  {
    ros::NodeHandle private_nh("/recharge");
    private_nh.param("yaw_delta", yaw_delta_, 0.0);
    private_nh.param("dist_delta", dist_delta_, 0.0);
  }

  ROS_INFO("Recharge Pose:(%.3f,%.3f,%.3f)", req.recharge_x, req.recharge_y, req.recharge_yaw);

  recharge_pose_.x = req.recharge_x;
  recharge_pose_.y = req.recharge_y;
  recharge_pose_.yaw = req.recharge_yaw / 180 * 3.1415;

  // 临时坐标点
  geometry_msgs::PoseStamped temp_point;

  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0, 0, recharge_pose_.yaw);
  temp_point.pose.orientation = tf2::toMsg(tf_quat);

  // 计算开始回充坐标点的位置，距离充点电正前方recharge_dist_m处，方向一致
  temp_point.pose.position.x = recharge_pose_.x + std::cos(recharge_pose_.yaw) * recharge_dist_;
  temp_point.pose.position.y = recharge_pose_.y + std::sin(recharge_pose_.yaw) * recharge_dist_;
  temp_point.pose.position.z = 0;

  bool to_temp_point = goToTempPoint(temp_point);

  int error_code = 1;
  if (to_temp_point)
  {
    sleep(1);

    ros::Rate loop_rate(control_frequency_);
    while (ros::ok())
    {
      // 充电完成，退出
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_charge_done_)
          break;
      }

      // 获取机器人当前坐标
      if (!getRobotPose())
      {
        error_code = -5;
        break;
      }

      // 将回充点的坐标从地图坐标系下转换为机器人坐标系下的坐标
      geometry_msgs::PoseStamped r_p, r_p_t_b_l_p;
      r_p.header.frame_id = "map";
      r_p.pose.position.x = recharge_pose_.x;
      r_p.pose.position.y = recharge_pose_.y;
      r_p.pose.position.z = 0.0;

      r_p.pose.orientation = temp_point.pose.orientation;
      if (!transformPose("base_link", r_p, r_p_t_b_l_p))
      {
        error_code = -4;
        break;
      }

      // 计算机器人到回充点的距离和方向
      double dx = robot_pose_.x - recharge_pose_.x;
      double dy = robot_pose_.y - recharge_pose_.y;
      double temp_dist = std::sqrt(dx * dx + dy * dy);

      double yaw_diff = angles::shortest_angular_distance(robot_pose_.yaw, recharge_pose_.yaw);

      // Vector2d A(recharge_pose_.x, recharge_pose_.y);
      // double angle = recharge_pose_.yaw;
      // Vector2d dir(cos(angle), sin(angle));
      // Vector2d B(robot_pose_.x, recharge_pose_.y);
      // double shortest_dist = distanceToLine(A, dir, B);
      
      Vector2d A1(recharge_pose_.x, recharge_pose_.y);
      Vector2d A2(temp_point.pose.position.x, temp_point.pose.position.y);
      Vector2d A3(robot_pose_.x, robot_pose_.y);
      double shortest_dist = pointToLineDistance(A1, A2, A3);

      double linear_vel = -linear_vel_;
      double angular_vel = 0.0;
      unsigned char gear = 7;
      double slipangle = 0.0;

      if (chassis_type_ == "DT")
        gear = 3;

      // 调整过程
      {
        // ROS_INFO("(robot_pose_.yaw,recharge_pose_.yaw) (%.3f,%.3f)  yaw_diff: %.3f  yaw_delta_: %.3f",
        //  robot_pose_.yaw, recharge_pose_.yaw, yaw_diff, yaw_delta_);

        // 如果大于yaw_delta，停下来原地旋转
        if (fabs(yaw_diff) >= yaw_delta_)
        {
          yaw_delta_ = yaw_delta_little_;
          angular_vel = yaw_diff > 0 ? angular_vel_ : -angular_vel_;
          linear_vel = 0.0;

          if (chassis_type_ == "FW")
            gear = 6;
        }

        // 往左偏了
        else if (r_p_t_b_l_p.pose.position.y > 0 && shortest_dist > dist_delta_)
        {
          dist_delta_ = dist_delta_little_;
          linear_vel = -linear_vel_;
          angular_vel = angular_vel_;
          slipangle = -slipangle_;
        }

        // 往右偏了
        else if (r_p_t_b_l_p.pose.position.y < 0 && shortest_dist > dist_delta_)
        {
          dist_delta_ = dist_delta_little_;
          linear_vel = -linear_vel_;
          angular_vel = -angular_vel_;
          slipangle = slipangle_;
        }
        else
        {
        }

        // 让底盘不反复调整
        if (fabs(yaw_diff) < yaw_delta_)
        {
          yaw_delta_ = yaw_delta_big_;
        }

        // 让底盘不反复调整
        if (shortest_dist <= dist_delta_)
        {
          dist_delta_ = dist_delta_big_;
        }
      }

      // 障碍物判断
      {
        static ros::Time last_time = ros::Time::now();

        if (!rear_has_obs_)
        {
          last_time = ros::Time::now();
        }
        else
        {
          linear_vel = 0.0;
          angular_vel = 0.0;
          slipangle = 0.0;
        }

        if ((ros::Time::now() - last_time).toSec() > 5.0)
        {
          error_code = -3;
          break;
        }
      }

      // 位置异常判断
      {
        if (shortest_dist > shortest_dist_y_)
        {
          error_code = -2;
          break;
        }
      }

      // FW底盘轮子回正
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (chassis_type_ == "FW" && fabs(angular_fb_) > 13 && (fabs(linear_vel) > 0.01 || fabs(slipangle) > 3))
        {
          linear_vel = 0.0;
          angular_vel = 0.0;
        }
      }

      // 发布控制指令
      if (gear != 7)
        publishCmdVel(linear_vel, angular_vel);
      else
        publishCtrlCmd(linear_vel, 0.0, gear, slipangle);

      loop_rate.sleep();
    } // while end
  }

  publishCmdVel(0.0, 0.0);

  res.result = error_code;

  if (error_code != 1)
  {
    ROS_ERROR("Recharge failed! error code: %d .", error_code);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recharge_node");
  RechargeNode recharge_node;

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  // ros::spin();
  return 0;
}
