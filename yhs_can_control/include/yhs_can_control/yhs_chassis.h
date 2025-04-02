#ifndef __YHS_CHASSIS_H__
#define __YHS_CHASSIS_H__

#include "ros/ros.h"
#include "ros/topic.h"
#include <XmlRpcValue.h>
#include <vector>

namespace yhs_chassis
{
  // 定义超声波数据结构体
  struct UlPoseParams
  {
      float x;
      float y;
      float z;
      float d_index;
      float p_index;
      float yaw;
      float min;
      float max;
  };

  class CanControl
  {
    public:

      explicit CanControl() {}

      virtual ~CanControl() {}

      virtual bool Run() = 0;

      virtual void Stop() = 0;
      
      std::vector<UlPoseParams> ul_pose_;
      
      void getUlParam()
      {
        ros::NodeHandle nh("~");

        XmlRpc::XmlRpcValue ul_param;
        if (nh.getParam("/ultrasonic", ul_param))
        {
          if (ul_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            for (XmlRpc::XmlRpcValue::iterator it = ul_param.begin(); it != ul_param.end(); ++it)
            {
              UlPoseParams pose;
              XmlRpc::XmlRpcValue &values = it->second;
              if (values.size() != 8 || values.getType() != XmlRpc::XmlRpcValue::TypeArray)
              {
                  ROS_WARN_STREAM("Invalid value for key " << it->first << ", skipping...");
                  continue;
              }

              pose.x = static_cast<double>(values[0]);
              pose.y = static_cast<double>(values[1]);
              pose.z = static_cast<double>(values[2]);
              pose.d_index = static_cast<double>(values[3]);
              pose.p_index = static_cast<double>(values[4]);
              pose.yaw = static_cast<double>(values[5]);
              pose.min = static_cast<double>(values[6]);
              pose.max = static_cast<double>(values[7]);
              ul_pose_.push_back(pose);
            }

            // 打印所有解析后的值
            for (const auto &pose : ul_pose_)
            {
              // ROS_INFO_STREAM("Pose: [x: " << pose.x << ", y: " << pose.y << ", z: " << pose.z << ", d_index: " << pose.d_index
              //                        << ", p_index: " << pose.p_index << ", yaw: " << pose.yaw << ", max: " << pose.max << "]");
            }
          }
          else
          {
            ROS_ERROR("Parameter '/ultrasonic' is not of type 'Struct'");
          }
        }
        else
        {
          ROS_ERROR("Failed to retrieve parameter '/ultrasonic'");
        }
      }
      
      bool hasPublisher(std::string topic_name)
      {
				ros::master::V_TopicInfo topics;
				ros::master::getTopics(topics);

				bool has_publisher = false;
				for (const auto& topic : topics) 
				{
					if (topic.name == topic_name)
					{
						has_publisher = true;
						break;
					}
				}
				
				return has_publisher;
      }
  };
}

#endif
