#ifndef __YHS_MK_CAN_CONTROL_H__
#define __YHS_Mk_CAN_CONTROL_H__

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_msgs/MkCtrlCmd.h"
#include "yhs_msgs/MkIoCmd.h"
#include "yhs_msgs/MkChassisInfoFb.h"

#include "yhs_msgs/Ultrasonic.h"

#include "yhs_can_control/yhs_chassis.h"

namespace can_control
{
  class MkCanControl : public yhs_chassis::CanControl
  {

  public:
    MkCanControl();
    ~MkCanControl();

    bool Run() override;
    void Stop() override;

  private:
    std::mutex mutex_;
    std::string odomFrame_, baseFrame_;
    bool tfUsed_;
    std::string if_name_;
    int can_socket_;
    double wheel_base_;
    can_frame recv_frames_;
    boost::thread thread_;
    double imu_roll_, imu_pitch_, imu_yaw_;

    ros::NodeHandle nh_;
    ros::Publisher chassis_info_fb_pub_;
    ros::Publisher ultrasonic_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher odom_pub_;
    ros::Subscriber ctrl_cmd_sub_;
    ros::Subscriber io_cmd_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber imu_sub_;

    bool WaitForCanFrame();
    void IoCmdCallBack(const yhs_msgs::MkIoCmd::ConstPtr &io_cmd_msg);
    void CtrlCmdCallBack(const yhs_msgs::MkCtrlCmd::ConstPtr &ctrl_cmd_msg);
    void CmdCallBack(const geometry_msgs::Twist::ConstPtr &cmd_msg);
    void OdomPub(const float velocity, const float steering);
    void RecvData();
    void ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg);
  };
}

#endif