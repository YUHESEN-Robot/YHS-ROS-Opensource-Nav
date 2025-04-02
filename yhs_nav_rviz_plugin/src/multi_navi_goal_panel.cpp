#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

#include "multi_navi_goal_panel.h"

namespace navi_tools_rviz_plugin
{

  NaviGoalsPathsPanel::NaviGoalsPathsPanel(QWidget *parent)
      : rviz::Panel(parent), nh_(), tf2_listener_(tf2_buffer_)
  {

    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 1,
                                                          boost::bind(&NaviGoalsPathsPanel::goalCntCB, this, _1));

    status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 1,
                                                                 boost::bind(&NaviGoalsPathsPanel::statusCB, this,
                                                                             _1));

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    record_path_pub_ = nh_.advertise<nav_msgs::Path>("record_path", 1);

    nav_type_pub_ = nh_.advertise<std_msgs::Int32>("/nav_type", 1);

    QVBoxLayout *root_layout = new QVBoxLayout;

    QHBoxLayout *nav_status_layout = new QHBoxLayout;
    nav_status_layout->addWidget(new QLabel("导航状态："));
    status_label_ = new QLabel("未知");
    nav_status_layout->addWidget(status_label_);

    QHBoxLayout *nav_pose_size_layout = new QHBoxLayout;
    nav_pose_size_layout->addWidget(new QLabel("导航点总数："));
    navi_pose_size_label_ = new QLabel("0");
    nav_pose_size_layout->addWidget(navi_pose_size_label_);

    QHBoxLayout *nav_run_pose_layout = new QHBoxLayout;
    nav_run_pose_layout->addWidget(new QLabel("正在前往的导航点："));
    navi_run_pose_label_ = new QLabel("");
    nav_run_pose_layout->addWidget(navi_run_pose_label_);

    QHBoxLayout *robot_current_position_layout = new QHBoxLayout;
    robot_current_position_layout->addWidget(new QLabel("机器人当前位置(x,y,yaw)："));
    robot_current_position_label_ = new QLabel("");
    robot_current_position_layout->addWidget(robot_current_position_label_);

    cycle_checkbox_ = new QCheckBox("循环");

    QHBoxLayout *circle_layout = new QHBoxLayout;
    circle_layout->addWidget(cycle_checkbox_);
    circle_layout->addWidget(new QLabel("已循环次数："));
    circle_num_label_ = new QLabel("");
    circle_layout->addWidget(circle_num_label_);

    root_layout->addLayout(nav_status_layout);
    root_layout->addLayout(nav_pose_size_layout);
    root_layout->addLayout(nav_run_pose_layout);
    root_layout->addLayout(robot_current_position_layout);
    root_layout->addLayout(circle_layout);

    initPoseTable();

    // creat a manipulate layout
    QHBoxLayout *manipulate_layout = new QHBoxLayout;
    output_reset_button_ = new QPushButton("重置");
    manipulate_layout->addWidget(output_reset_button_);

    output_cancel_button_ = new QPushButton("取消");
    manipulate_layout->addWidget(output_cancel_button_);

    output_startNavi_button_ = new QPushButton("开始导航");
    manipulate_layout->addWidget(output_startNavi_button_);

    root_layout->addLayout(manipulate_layout);

    QHBoxLayout *record_path_layout = new QHBoxLayout;
    start_record_and_save_path_button_ = new QPushButton("开始记录路径");
    record_path_layout->addWidget(start_record_and_save_path_button_);

    load_path_and_start_navi_button_ = new QPushButton("载入路径开始导航");
    record_path_layout->addWidget(load_path_and_start_navi_button_);

    root_layout->addLayout(record_path_layout);

    setLayout(root_layout);
    // set a Qtimer to start a spin for subscriptions
    output_timer_ = new QTimer(this);
    output_timer_->start(100);

    output_cancel_button_->setEnabled(false);
    output_startNavi_button_->setEnabled(false);

    connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
    connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
    connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
    connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));

    connect(start_record_and_save_path_button_, SIGNAL(clicked()), this, SLOT(recordSavePath()));
    connect(load_path_and_start_navi_button_, SIGNAL(clicked()), this, SLOT(loadPathStartNavi()));

    connect(output_timer_, SIGNAL(timeout()), this, SLOT(startSpin()));
  }

  // initialize the table of pose
  void NaviGoalsPathsPanel::initPoseTable()
  {
    ROS_INFO("Initialize");
    curGoalIdx_ = 0, cycleCnt_ = 0;
    permit_ = false, cycle_ = false;

    NumGoal_ = 0;

    pose_array_.poses.clear();

    deleteMark();

    cycle_checkbox_->setCheckState(Qt::Unchecked);
    navi_pose_size_label_->setText(QString::number(NumGoal_));
  }

  // delete marks in the map
  void NaviGoalsPathsPanel::deleteMark()
  {
    visualization_msgs::Marker marker_delete;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);
  }

  // call back function for counting goals
  void NaviGoalsPathsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose)
  {
    NumGoal_++;

    pose_array_.poses.push_back(pose->pose);
    pose_array_.header.frame_id = pose->header.frame_id;

    markPose(pose);

    // 更新导航点总数
    navi_pose_size_label_->setText(QString::number(NumGoal_));
  }

  std::string NaviGoalsPathsPanel::doubleToString(double value)
  {
    std::ostringstream stream;
    double truncated_value = std::floor(value * 100) / 100; // Keep two decimal places
    stream << truncated_value;
    return stream.str();
  }

  // when setting a Navi Goal, it will set a mark on the map
  void NaviGoalsPathsPanel::markPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
  {
    if (ros::ok())
    {
      visualization_msgs::Marker arrow;
      visualization_msgs::Marker number;
      arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
      arrow.ns = "navi_point_arrow";
      number.ns = "navi_point_number";
      arrow.action = number.action = visualization_msgs::Marker::ADD;
      arrow.type = visualization_msgs::Marker::ARROW;
      number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      arrow.pose = number.pose = pose->pose;
      number.pose.position.z += 1.0;
      arrow.scale.x = 1.0;
      arrow.scale.y = 0.2;

      number.scale.z = 0.3;

      arrow.color.r = 0.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0;

      number.color.r = 1.0f;
      number.color.g = 0.0f;
      number.color.b = 0.0f;
      number.color.a = 1.0;

      arrow.id = number.id = pose_array_.poses.size();

      number.text = std::to_string(pose_array_.poses.size()) + " (" +
                    doubleToString(pose->pose.position.x) + ", " + doubleToString(pose->pose.position.y) +
                    ", " + doubleToString(tf2::getYaw(pose->pose.orientation) * 180.0 / 3.14) + ")";

      marker_pub_.publish(arrow);
      marker_pub_.publish(number);
    }
  }

  // check whether it is in the cycling situation
  void NaviGoalsPathsPanel::checkCycle()
  {
    cycle_ = cycle_checkbox_->isChecked();
  }

  // start to navigate, and only command the first goal
  void NaviGoalsPathsPanel::startNavi()
  {
    curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
    if (!pose_array_.poses.empty() && curGoalIdx_ < NumGoal_)
    {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_);
      goal_pub_.publish(goal);

      ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);

      curGoalIdx_ += 1;
      permit_ = true;
    }
    else
    {
      ROS_ERROR("Something Wrong");
    }
  }

  void NaviGoalsPathsPanel::recordSavePath()
  {
    if (!already_start_record_path_)
    {
      already_start_record_path_ = true;
      start_record_and_save_path_button_->setText("停止并保存路径");
      load_path_and_start_navi_button_->setEnabled(false);
    }
    else
    {
      already_start_record_path_ = false;
      start_record_and_save_path_button_->setText("开始记录路径");
      load_path_and_start_navi_button_->setEnabled(true);
      start_record_and_save_path_button_->setEnabled(true);
    }
  }

  void NaviGoalsPathsPanel::loadPathStartNavi()
  {
    start_load_record_path_ = true;
    is_record_path_nav_mode_ = true;
  }

  // complete the remaining goals
  void NaviGoalsPathsPanel::completeNavi()
  {
    if (curGoalIdx_ < pose_array_.poses.size())
    {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_);
      goal_pub_.publish(goal);
      ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);

      curGoalIdx_ += 1;
      permit_ = true;
    }
    else
    {
      ROS_ERROR("All goals are completed");
      permit_ = false;
    }
  }

  // command the goals cyclically
  void NaviGoalsPathsPanel::cycleNavi()
  {
    if (permit_)
    {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
      goal_pub_.publish(goal);
      ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
               cycleCnt_ + 1);

      curGoalIdx_ += 1;
      cycleCnt_ = (curGoalIdx_ - 1) / pose_array_.poses.size();
    }
  }

  // cancel the current command
  void NaviGoalsPathsPanel::cancelNavi()
  {
    if (cur_goal_status_.status == 1)
    {
      cancel_pub_.publish(cur_goal_status_.goal_id);
      ROS_ERROR("Navigation have been canceled");
    }
  }

  // call back for listening current state
  void NaviGoalsPathsPanel::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses)
  {
    bool arrived_pre = arrived_;
    arrived_ = checkGoal(statuses->status_list);

    if (arrived_ && arrived_ != arrived_pre && ros::ok() && permit_)
    {
      if (cycle_ && pose_array_.poses.size() > 1)
        cycleNavi();
      else
        completeNavi();
    }
  }

  // check the current state of goal
  bool NaviGoalsPathsPanel::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list)
  {
    bool done;
    if (!status_list.empty())
    {

      // for (auto &i : status_list)
      // {
      cur_goal_status_ = status_list.back();
      // 导航完成
      if (cur_goal_status_.status == 3)
      {
        done = true;
        ROS_INFO("completed Goal%d", curGoalIdx_);
      }
      // 导航失败
      else if (cur_goal_status_.status == 4)
      {
        done = true;
        ROS_ERROR("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
      }
      // 导航点改变处理中
      else if (cur_goal_status_.status == 0)
      {
        done = true;
        status_label_->setText("导航点改变处理中...");
      }
      // 导航中
      else if (cur_goal_status_.status == 1)
      {
        // cur_goalid_ = i.goal_id;
        done = false;
      }
      // 导航取消
      else if (cur_goal_status_.status == 2)
      {
        done = false;
      }
      else
        done = false;
      // }
    }
    else
    {
      ROS_INFO_ONCE("Please input the Navi Goal");
      done = false;
    }
    return done;
  }

  // spin for subscribing
  void NaviGoalsPathsPanel::startSpin()
  {
    if (!move_base_ready_ && isNodeRunning("/move_base"))
    {
      output_timer_->stop();

      MoveBaseClient ac("move_base", true);

      // 等待action服务器启动
      ROS_INFO("等待move_base action服务器启动...");
      if (!ac.waitForServer(ros::Duration(2.0)))
      {
        ROS_ERROR("无法连接到move_base action服务器");
      }
      else
      {
        move_base_ready_ = true;
        status_label_->setText("move_base就绪");
      }
      output_timer_->start(100);
    }

    if (!move_base_ready_)
      return;

    if (ros::ok())
    {
      ros::spinOnce();

      static int status_pre = -8;

      if (status_pre != cur_goal_status_.status || (ros::Time::now() - navi_run_time_).toSec() > 1.0)
      {
        navi_run_time_ = ros::Time::now();
        status_pre = cur_goal_status_.status;

        // 导航中
        if (cur_goal_status_.status == 1)
        {
          status_label_->setText("导航中...");

          // 录制路径模式
          // if (is_record_path_nav_mode_)
          // {
          start_record_and_save_path_button_->setEnabled(false);
          load_path_and_start_navi_button_->setEnabled(false);
          // }

          output_cancel_button_->setEnabled(true);
          output_reset_button_->setEnabled(false);
          output_startNavi_button_->setEnabled(false);
        }

        // 导航完成
        else if (cur_goal_status_.status == 3)
        {
          status_label_->setText("导航完成");

          // 录制路径模式
          if (is_record_path_nav_mode_)
          {
            // static int robot_at_which_pose = 0;
            // robot_at_which_pose++;
            double to_start_dist = std::sqrt(std::pow(robot_x_ - record_path_.front().x, 2) +
                                             std::pow(robot_y_ - record_path_.front().y, 2));
            double to_end_dist = std::sqrt(std::pow(robot_x_ - record_path_.back().x, 2) +
                                           std::pow(robot_y_ - record_path_.back().y, 2));
            // 到达第一个点，接着下发最后一个点
            if (to_start_dist < 0.2)
            {
              // 发布导航类型
              std_msgs::Int32 nav_type_msg;
              nav_type_msg.data = 1;
              nav_type_pub_.publish(nav_type_msg);

              //  清除多点导航数据
              initPoseTable();

              // 当做在rviz上下发路径的最后一个点
              goalCntCB(boost::make_shared<const geometry_msgs::PoseStamped>(record_path_msg_.poses.back()));

              // 开始导航到路径最后一个点
              startNavi();
              ROS_ERROR("to record path second pose");
            }

            // 到达最后一个点
            if (to_end_dist < 0.2)
            {
              //  清除多点导航数据
              initPoseTable();
              start_record_and_save_path_button_->setEnabled(true);
              load_path_and_start_navi_button_->setEnabled(true);
              is_record_path_nav_mode_ = false;
              // robot_at_which_pose = 0;
              ROS_ERROR("get record path second pose");
            }
          }

          else
          {
            output_cancel_button_->setEnabled(false);
            output_reset_button_->setEnabled(true);

            if (!pose_array_.poses.empty())
            {
              output_startNavi_button_->setEnabled(true);
            }
            else
            {
              output_startNavi_button_->setEnabled(false);
            }
            start_record_and_save_path_button_->setEnabled(true);
            load_path_and_start_navi_button_->setEnabled(true);
          }
        }

        // 导航取消
        else if (cur_goal_status_.status == 2)
        {
          status_label_->setText("导航取消");

          // 录制路径模式
          if (is_record_path_nav_mode_)
          {
            // 清除
            initPoseTable();

            is_record_path_nav_mode_ = false;
          }

          if (curGoalIdx_ > 0)
          {
            curGoalIdx_ -= 1;
          }

          output_cancel_button_->setEnabled(false);
          output_reset_button_->setEnabled(true);

          if (!pose_array_.poses.empty())
          {
            output_startNavi_button_->setEnabled(true);
          }
          else
          {
            output_startNavi_button_->setEnabled(false);
          }

          start_record_and_save_path_button_->setEnabled(true);
          load_path_and_start_navi_button_->setEnabled(true);
        }

        // 导航失败
        else if (cur_goal_status_.status == 4)
        {
          status_label_->setText("导航失败");

          // 录制路径模式
          if (is_record_path_nav_mode_)
          {
            // 清除
            initPoseTable();

            start_record_and_save_path_button_->setEnabled(true);
            load_path_and_start_navi_button_->setEnabled(true);
            is_record_path_nav_mode_ = false;
          }

          output_cancel_button_->setEnabled(false);
          output_reset_button_->setEnabled(true);

          if (!pose_array_.poses.empty())
          {
            output_startNavi_button_->setEnabled(true);
          }
          else
          {
            output_startNavi_button_->setEnabled(false);
          }
        }

        // 导航没开始
        else
        {
          if (!pose_array_.poses.empty())
          {
            output_startNavi_button_->setEnabled(true);
            output_reset_button_->setEnabled(true);
            output_cancel_button_->setEnabled(false);
          }
          else
          {
            output_startNavi_button_->setEnabled(false);
            output_reset_button_->setEnabled(true);
            output_cancel_button_->setEnabled(false);
          }
        }

        // 记录路径中
        if (already_start_record_path_)
        {
          output_startNavi_button_->setEnabled(false);
          output_reset_button_->setEnabled(false);
          output_cancel_button_->setEnabled(false);
          load_path_and_start_navi_button_->setEnabled(false);
        }
      }

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
          return;
        }

        // 提取坐标和偏航角
        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;
        robot_x_ = x;
        robot_y_ = y;

        // 使用四元数转换为欧拉角
        tf2::Quaternion quat;
        tf2::convert(transformStamped.transform.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        QString text = QString::fromStdString("(" + doubleToString(x) + ", " +
                                              doubleToString(y) + ", " +
                                              doubleToString(yaw * 180.0 / 3.14) +
                                              ")");
        // 机器人当前位置
        robot_current_position_label_->setText(text);

        // 正在前往curGoalIdx_点
        if (pose_array_.poses.size() > 0)
        {
          int index = curGoalIdx_ % pose_array_.poses.size();
          if (index == 0)
            index = pose_array_.poses.size();
          navi_run_pose_label_->setText(QString::number(index));
        }

        // 循环次数
        circle_num_label_->setText(QString::number(cycleCnt_));

        // 录制路径
        {
          std::string file_path = ros::package::getPath("navi_tools_rviz_plugin") + "/data/path.txt";
          static std::ofstream outfile;
          static double last_x = -10000;
          static double last_y = -10000;
          // 记录
          {
            if (already_start_record_path_)
            {
              if (!outfile.is_open())
              {
                outfile.open(file_path);

                if (!outfile.is_open())
                {
                  ROS_ERROR("Failed to open file %s", file_path.c_str());
                  return;
                }
              }
              // 处于打开状态
              else
              {
                double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

                if (distance > 0.2)
                {
                  outfile << x << "," << y << "," << yaw << std::endl;
                  last_x = x;
                  last_y = y;
                }
              }
            }
            // 保存
            else
            {
              if (outfile.is_open())
              {
                outfile << x << "," << y << "," << yaw << std::endl;
                last_x = -10000;
                last_y = -10000;

                outfile.close();
              }
            }
          } // end 记录路径

          // 载入路径
          {
            if (start_load_record_path_)
            {
              record_path_.clear();
              std::ifstream infile(file_path);
              if (!infile.is_open())
              {
                ROS_ERROR("Failed to open file %s", file_path.c_str());
                is_record_path_nav_mode_ = false;
                return;
              }

              std::string line;
              while (std::getline(infile, line))
              {
                std::stringstream ss(line);
                std::string x_str, y_str, yaw_str;
                if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, yaw_str, ','))
                {
                  Data data;
                  data.x = std::stod(x_str);
                  data.y = std::stod(y_str);
                  data.yaw = std::stod(yaw_str);
                  record_path_.push_back(data);
                }
              }
              infile.close();

              if (record_path_.size() > 10)
              {
                record_path_msg_.poses.clear();
                record_path_msg_.header.frame_id = "map";
                record_path_msg_.header.stamp = ros::Time::now();

                for (const auto &data : record_path_)
                {
                  geometry_msgs::PoseStamped pose;
                  pose.header.frame_id = "map";
                  pose.header.stamp = ros::Time::now();
                  pose.pose.position.x = data.x;
                  pose.pose.position.y = data.y;
                  pose.pose.position.z = 0.0;

                  // 使用tf2创建四元数
                  tf2::Quaternion quat;
                  quat.setRPY(0, 0, data.yaw);
                  pose.pose.orientation = tf2::toMsg(quat);

                  record_path_msg_.poses.push_back(pose);
                }

                // 清除多点导航数据
                initPoseTable();

                // 当做在rviz上下发路径的第一个点
                goalCntCB(boost::make_shared<const geometry_msgs::PoseStamped>(record_path_msg_.poses.front()));

                // 开始导航到路径第一个点
                startNavi();

                ROS_ERROR("to record path first pose");

                record_path_pub_.publish(record_path_msg_);
              }

              start_load_record_path_ = false;
            }

          } // end 载入路径
        }
      }
    }
  }

} // end namespace navi_tools_rviz_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_tools_rviz_plugin::NaviGoalsPathsPanel, rviz::Panel)
