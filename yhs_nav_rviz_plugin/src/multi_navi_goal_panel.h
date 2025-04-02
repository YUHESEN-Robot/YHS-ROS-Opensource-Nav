#ifndef NAVI_TOOLS_RVIZ_PLUGIN_H
#define NAVI_TOOLS_RVIZ_PLUGIN_H

#include <string>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/master.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <XmlRpcValue.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLabel>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

struct Data
{
  double x;
  double y;
  double yaw;
};

namespace navi_tools_rviz_plugin
{

  class NaviGoalsPathsPanel : public rviz::Panel
  {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    Q_OBJECT
  public:
    explicit NaviGoalsPathsPanel(QWidget *parent = 0);

  public Q_SLOTS:

    void markPose(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void deleteMark();

  protected Q_SLOTS:

    void initPoseTable(); // initialize the pose table
    void startNavi();     // start navigate for the first pose
    void cancelNavi();

    void recordSavePath();

    void loadPathStartNavi();

    void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose); // goal count sub callback function

    void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); // status sub callback function

    void checkCycle();

    void completeNavi(); // after the first pose, continue to navigate the rest of poses
    void cycleNavi();

    bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list); // check whether arrived the goal

    void startSpin(); // spin for sub

    std::string doubleToString(double value);
    
    bool isNodeRunning(const std::string& node_name)
		{
		  XmlRpc::XmlRpcValue args, result, payload;
		  args[0] = ros::this_node::getName();  // Name of the node making the request
		  if (ros::master::execute("getSystemState", args, result, payload, true))
		  {
		      XmlRpc::XmlRpcValue& nodes = payload[2];  // payload[2] contains the list of all nodes
		      for (int i = 0; i < nodes.size(); ++i)
		      {
		          for (int j = 0; j < nodes[i][1].size(); ++j)
		          {
		              std::string name = nodes[i][1][j];
		              if (name == node_name)
		              {
		                  return true;
		              }
		          }
		      }
		  }
		  return false;
		}

  protected:
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    QLabel *status_label_, *navi_pose_size_label_, *navi_run_pose_label_,
        *robot_current_position_label_, *circle_num_label_;
    QPushButton *output_reset_button_, *output_startNavi_button_, *output_cancel_button_,
        *start_record_and_save_path_button_, *load_path_and_start_navi_button_;
    QCheckBox *cycle_checkbox_;

    // The ROS node handle.
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_, cancel_pub_, marker_pub_,
        record_path_pub_, nav_type_pub_;
    ros::Subscriber goal_sub_, status_sub_;

    int NumGoal_;
    int curGoalIdx_ = 0, cycleCnt_ = 0;
    bool permit_ = false, cycle_ = false, arrived_ = false;
    geometry_msgs::PoseArray pose_array_;

    actionlib_msgs::GoalStatus cur_goal_status_;

    QTimer *output_timer_;
    bool move_base_ready_ = false;

    bool already_start_record_path_ = false;
    bool start_load_record_path_ = false;

    bool is_record_path_nav_mode_ = false;

    std::vector<Data> record_path_;
    nav_msgs::Path record_path_msg_;
    double robot_x_, robot_y_;

    ros::Time navi_run_time_;
  };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // NAVI_TOOLS_RVIZ_PLUGIN_H
