// Copyright (C) 2007, 2009, 2010, 2012, 2015Yaxin Liu, Patrick Beeson, Austin Robot Technology, Jack O'Quin
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

/** @file
 *
 *  timoo 3D LIDAR data input classes
 *
 *    These classes provide raw timoo LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     timoo::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     timoo::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     timoo::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef timoo_DRIVER_INPUT_H
#define timoo_DRIVER_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

#include <ros/ros.h>
#include <timoo_msgs/timooPacket.h>
#include <timoo_msgs/timooStatus.h>
namespace timoo_driver
{

static uint16_t DATA_PORT_NUMBER = 2368;      // default data port
static uint16_t STATUS_PORT_NUMBER = 8603;  // default lidar status port
static uint16_t POSITION_PORT_NUMBER = 8308;  // default position port

/** @brief timoo input base class */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);
  virtual ~Input() {}

  /** @brief Read one timoo packet.
   *
   * @param pkt points to timooPacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(timoo_msgs::timooPacket *pkt,
                        const double time_offset) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  bool gps_time_;
};

/** @brief Live timoo input from socket. */
class InputSocket: public Input
{
public:
  InputSocket(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(timoo_msgs::timooPacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  int sockfd_;
  in_addr devip_;
};

/** @brief Live timoo input from socket(For lidar/gps status). */
class InputStatusSocket: public Input
{
public:
  InputStatusSocket(ros::NodeHandle private_nh,
              uint16_t port = STATUS_PORT_NUMBER);
  virtual ~InputStatusSocket();

  // virtual int getPacket(timoo_msgs::timooPacket *pkt,
  //                       const double time_offset,timoo_msgs::timooStatus *status_msg);
  virtual int getPacket(timoo_msgs::timooPacket *pkt,
                        const double time_offset);
                        
  void setDeviceIP(const std::string& ip);

private:
  int sockfd_;
  in_addr devip_;
};


/** @brief timoo input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, timoo's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~InputPCAP();

  virtual int getPacket(timoo_msgs::timooPacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace timoo_driver

#endif  // timoo_DRIVER_INPUT_H
