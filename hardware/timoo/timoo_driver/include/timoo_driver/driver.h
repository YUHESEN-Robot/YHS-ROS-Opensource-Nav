// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef timoo_DRIVER_DRIVER_H
#define timoo_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>

#include <timoo_driver/input.h>
#include <timoo_driver/timooNodeConfig.h>
#include <sensor_msgs/Imu.h>

namespace timoo_driver
{
  
struct gpsInfo{
  bool gps_status;
  ros::Time gps_time;
  // Latitu and so on
};

class timooDriver
{
public:
  timooDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh,
                 std::string const & node_name = ros::this_node::getName());
  ~timooDriver() {}

  bool poll(void);
  bool statusPoll(void);

private:
  // Callback for dynamic reconfigure
  void callback(timoo_driver::timooNodeConfig &config,
              uint32_t level);
 
  void diagTimerCallback(const ros::TimerEvent&event);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<timoo_driver::
              timooNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each timoo time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;
  }
  config_;

  union Float4Byte{
    uint8_t bytes[4];
    float value;
  };

  typedef struct{
    int16_t IMU_acceleration_x;
    int16_t IMU_acceleration_y;
    int16_t IMU_acceleration_z;
    int16_t IMU_angular_velocity_x; 
    int16_t IMU_angular_velocity_y; 
    int16_t IMU_angular_velocity_z; 
  }IMU_DATA;

  typedef struct{
    uint32_t IMU_Time_Start;
    uint32_t IMU_Time_End;
    uint16_t IMU_Temp;
    IMU_DATA IMU_data[10];
  }IMU_dpkt;
  
 
  // int16_t imu_data[6];
  boost::shared_ptr<Input> input_;
  boost::shared_ptr<Input> status_input_;
  gpsInfo gps_info;
  std::mutex gps_info_mutex;
  bool gps_time_;
  ros::Publisher output_;
  ros::Publisher status_output_;
  ros::Publisher ImuData_output_;
  int last_azimuth_;

  /* diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

}  // namespace timoo_driver

#endif  // timoo_DRIVER_DRIVER_H
