// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
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

/** \file
 *
 *  ROS driver implementation for the timoo 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <timoo_msgs/timooScan.h>
#include <timoo_msgs/timooStatus.h>

#include "timoo_driver/driver.h"
#include <time.h>       /* time_t, struct tm, time, mktime */
#include <stdio.h>  
#include <math.h>

namespace timoo_driver
{

timooDriver::timooDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh,
                               std::string const & node_name)
  : diagnostics_(node, private_nh, node_name)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("timoo"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("C16"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
 
 if (config_.model == "C16")
    {
      packet_rate = 848;             
      model_full_name = "TM-C16";
    }
  else
    {
      ROS_ERROR_STREAM("unknown timoo LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("timoo ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  // if we are timestamping based on the first or last packet in the scan
  private_nh.param("timestamp_first_packet", config_.timestamp_first_packet, false);
  if (config_.timestamp_first_packet)
    ROS_INFO("Setting timoo scan start time to timestamp of first packet");
 
  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < (2*M_PI))
  {
      ROS_INFO_STREAM("Cut at specific angle feature activated. " 
        "Cutting timoo points always at " << cut_angle << " rad.");
  }
  else
  {
    ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
    << "between 0.0 and 2*PI or negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree, 
  // which is used in timoo packets
  config_.cut_angle = int((cut_angle*360/(2*M_PI))*100);

  int udp_port, udp_status_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);
  private_nh.param("status_port", udp_status_port, (int) STATUS_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<timoo_driver::
    timooNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<timoo_driver::timooNodeConfig>::
    CallbackType f;
  f = boost::bind (&timooDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("timoo_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));
  diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &timooDriver::diagTimerCallback,this);

  config_.enabled = true;

  private_nh.param("gps_time", gps_time_, false);
  // open timoo input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new timoo_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new timoo_driver::InputSocket(private_nh, udp_port));
      status_input_.reset(new timoo_driver::InputStatusSocket(private_nh, udp_status_port));
    }

  // raw packet output topic
  output_ = node.advertise<timoo_msgs::timooScan>("timoo_packets", 10);
  
  status_output_ = node.advertise<timoo_msgs::timooStatus>("timoo_status", 10);

  //imu data
  ImuData_output_ = node.advertise<sensor_msgs::Imu>("timoo_imu", 10);

  last_azimuth_ = -1;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool timooDriver::poll(void)
{
  if (!config_.enabled) {
    // If we are not enabled exit once a second to let the caller handle
    // anything it might need to, such as if it needs to exit.
    ros::Duration(1).sleep();
    return true;
  }

  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  timoo_msgs::timooScanPtr scan(new timoo_msgs::timooScan);

  if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    timoo_msgs::timooPacket tmp_packet;
    while(true)
    {
      while(true)
      {
        // std::lock_guard<std::mutex> gps_info_lock(gps_info_mutex);
        tmp_packet.gps_status = gps_info.gps_status;
        tmp_packet.data_stamp = gps_info.gps_time;
        int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      // Extract base rotation of first block in packet
      std::size_t azimuth_data_pos = 100*0+2;
      int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      //if first packet in scan, there is no "valid" last_azimuth_
      if (last_azimuth_ == -1) {
      	 last_azimuth_ = azimuth;
      	 continue;
      }
      if((last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth)
      	 || ( config_.cut_angle <= azimuth && azimuth < last_azimuth_)
      	 || (azimuth < last_azimuth_ && last_azimuth_ < config_.cut_angle))
      {
        last_azimuth_ = azimuth;
        break; // Cut angle passed, one full revolution collected
      }
      last_azimuth_ = azimuth;
    }
  }
  else // standard behaviour
  {
  // Since the timoo delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
    scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          // std::lock_guard<std::mutex> gps_info_lock(gps_info_mutex);
          scan->packets[i].gps_status = gps_info.gps_status;
          scan->packets[i].data_stamp = gps_info.gps_time;
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
          timoo_msgs::timooPacket status_packet;
          if (rc == 0) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
        }
    }
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full timoo scan.");
  if (config_.timestamp_first_packet){
    scan->header.stamp = scan->packets.front().stamp;
  }
  else{
    scan->header.stamp = scan->packets.back().stamp;
  }

  scan->header.frame_id = config_.frame_id;
  // std::cout << "++++++"  << std::endl;
  // std::cout << "++++ stamp_sec:" << scan->header.stamp.sec << "  stamp_nsec:" << scan->header.stamp.nsec << std::endl;
  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

bool timooDriver::statusPoll(void)
{
  if (!config_.enabled) {
    // If we are not enabled exit once a second to let the caller handle
    // anything it might need to, such as if it needs to exit.
    ros::Duration(1).sleep();
    return true;
  }

  while (true)
    {
      // keep reading until full packet received
      timoo_msgs::timooPacket status_packet;
      timoo_msgs::timooStatusPtr status_msg(new timoo_msgs::timooStatus);
      int rc = status_input_->getPacket(&status_packet, config_.time_offset);
      // int rc = status_input_->getPacket(&status_packet, config_.time_offset, &status_msg);
      
      // if (rc == 0) break;       // got a full packet?
      if (rc < 0) return false; // end of file reached?
      // GPS time: pkt->data[36]
      // GPS tag: pkt->data[1000] 

      // std::cout << "____________________________" << std::endl;
      // std::lock_guard<std::mutex> gps_info_lock(gps_info_mutex);
    
      float ChannelToAngleValueTM16[16] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};

      int AngleNoToChannelC16[16] = {0,2,4,6,8,10,12,14,1,3,5,7,9,11,13,15};


      for(int i = 0; i < 16; i++)
      {
        Float4Byte float4byte;
        for(int j = 0 ; j < 4 ; j++){
          float4byte.bytes[j] = status_packet.data[834 + 4*i + j];
        }
        if(float4byte.value >= -16 + i*2 && float4byte.value <= -14 + i*2){
          status_msg->vertical_angle_list[AngleNoToChannelC16[i]] = float4byte.value;    
        }
        else
        {
          status_msg->vertical_angle_list[i] = ChannelToAngleValueTM16[i];
        }
      }

      uint8_t buf[130];
      for(int i= 0 ;i< 130 ;i++){
        buf[i] = status_packet.data[1019+ i];
      }
      IMU_dpkt* imu_dpkt = (IMU_dpkt*)buf;
      
      if((ntohl(imu_dpkt->IMU_Time_End) - ntohl(imu_dpkt->IMU_Time_Start)) > 0){
        double group_timestamp = status_packet.stamp.toSec();
      
        // ros::Time group_time() = pkt->stamp;
        double imu_time_diff = (ntohl(imu_dpkt->IMU_Time_End) - ntohl(imu_dpkt->IMU_Time_Start)) * 1e-6 / 4;
        
        for(int i = 0 ; i < 4 ; i++){          
          sensor_msgs::Imu ros_msg;
          ros_msg.header.stamp.sec = (uint32_t)floor(group_timestamp +imu_time_diff * i);
          ros_msg.header.stamp.nsec = (uint32_t)round(((group_timestamp +imu_time_diff * i) - ros_msg.header.stamp.sec) * 1e9 );

          ros_msg.header.frame_id = config_.frame_id;
          ros_msg.linear_acceleration.x = 9.8 * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_acceleration_x) * (6 / 32768.0);
          ros_msg.linear_acceleration.y = 9.8 * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_acceleration_y) * (6 / 32768.0);
          ros_msg.linear_acceleration.z = 9.8 * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_acceleration_z) * (6 / 32768.0);
          ros_msg.angular_velocity.x  = (M_PI / 180) * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_angular_velocity_x) * (1000 / 32768.0);
          ros_msg.angular_velocity.y  = (M_PI / 180) * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_angular_velocity_y) * (1000 / 32768.0);
          ros_msg.angular_velocity.z  = (M_PI / 180) * (int16_t)ntohs(imu_dpkt->IMU_data[i].IMU_angular_velocity_z) * (1000 / 32768.0);

          ImuData_output_.publish(ros_msg);
        } 
      }

     

      status_msg->header.frame_id = config_.frame_id;
      status_msg->header.stamp = status_packet.stamp;
      // status)msg->data = status_packet.data;  //***********
      gps_info.gps_time = status_packet.stamp;

      // Publish
      status_output_.publish(status_msg);

    }
  
  return true;

}

void timooDriver::callback(timoo_driver::timooNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  if (level & 1)
  {
    config_.time_offset = config.time_offset;
  }
  if (level & 2)
  {
    config_.enabled = config.enabled;
  }
}

void timooDriver::diagTimerCallback(const ros::TimerEvent &event)
{
  (void)event;
  // Call necessary to provide an error when no timoo packets are received
  diagnostics_.update();
}

} // namespace timoo_driver
