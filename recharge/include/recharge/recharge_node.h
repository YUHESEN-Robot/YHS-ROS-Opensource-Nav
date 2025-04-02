#ifndef RECHARGE_NODE_H
#define RECHARGE_NODE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <recharge/Recharge.h>
#include <recharge/DisRecharge.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <angles/angles.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "yhs_msgs/DgtChassisInfoFb.h"
#include "yhs_msgs/FwChassisInfoFb.h"
#include "yhs_msgs/FwSteeringCtrlCmd.h"
#include "yhs_msgs/MkChassisInfoFb.h"
#include "yhs_msgs/FrChassisInfoFb.h"

using namespace Eigen;

// 机器人实时坐标和方向
struct RobotPose
{
  double x;
  double y;
  double yaw;
};

// 充电坐标和方向
struct RechargePose
{
  double x;
  double y;
  double yaw;
};

class RechargeNode
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

public:
  RechargeNode();

private:
  ros::NodeHandle nh_;
  ros::Subscriber scan1_sub_;
  ros::Subscriber scan2_sub_;
  ros::Subscriber fw_chassis_info_sub_;
  ros::Subscriber angular_fb_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher ctrl_cmd_pub_;

  ros::ServiceServer recharge_srv_, dis_recharge_srv_;
  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection projector_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::mutex mutex_;
  double angular_fb_;

  // 机器人实时位置
  RobotPose robot_pose_;

  // 充电位置
  RechargePose recharge_pose_;

  std::string chassis_type_;

  double recharge_dist_;
  bool is_charge_failed_{false};
  bool is_charge_done_{false};

  bool front_has_obs_{false};
  bool rear_has_obs_{false};

  double car_half_length_;
  double car_half_width_;

  double shortest_dist_y_;

  double yaw_delta_little_;
  double yaw_delta_;
  double yaw_delta_big_;
  double dist_delta_little_;
  double dist_delta_;
  double dist_delta_big_;
  double angular_vel_;
  double linear_vel_;
  double slipangle_;
  int control_frequency_;

  void FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr &msg);
  void angularCallback(const std_msgs::Float32::ConstPtr &angular_msg);
  void scan1Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  void scan2Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  bool rechargeCallback(recharge::Recharge::Request &req, recharge::Recharge::Response &res);
  bool disRechargeCallback(recharge::DisRecharge::Request &req, recharge::DisRecharge::Response &res);

  bool goToTempPoint(const geometry_msgs::PoseStamped temp_point);
  bool getRobotPose();
  bool transformPose(const std::string frame, const geometry_msgs::PoseStamped &in_pose, geometry_msgs::PoseStamped &out_pose);
  double distanceToLine(Vector2d A, Vector2d dir, Vector2d B);
  double pointToLineDistance(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C);


  void publishCmdVel(const double linear_vel, const double angular_vel);
  void publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle);

};

#endif // RECHARGE_NODE_H
