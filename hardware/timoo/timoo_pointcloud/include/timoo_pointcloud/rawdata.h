// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
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
 *  @brief Interfaces for interpreting raw packets from the timoo 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef timoo_POINTCLOUD_RAWDATA_H
#define timoo_POINTCLOUD_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <timoo_msgs/timooScan.h>

#include <timoo_msgs/timooStatus.h>
#include <timoo_pointcloud/calibration.h>
#include <timoo_pointcloud/datacontainerbase.h>
#include <string>
#include <cstdio>
#include <typeinfo>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


void split1(const std::string &s, std::vector<std::string> &elems, char delim = ',');
namespace timoo_rawdata
{
/**
 * Raw timoo packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for TM16 support **/
static const int TM16_FIRINGS_PER_BLOCK = 2;
static const int TM16_SCANS_PER_FIRING = 16;
static const float TM16_BLOCK_TDURATION = 98.304f;  // [µs]
static const float TM16_DSR_TOFFSET = 3.072f;        // [µs]
static const float TM16_FIRING_TOFFSET = 49.152f;    // [µs]

/** \brief Raw timoo data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw timoo packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;



//string spllit
std::vector<std::string> split(const std::string& str, const std::string& delim );
typedef struct tm_point
{
  float x;
  float y;
  float z;
  uint16_t ring;
  uint16_t azimuth;
  float distance;
  float intensity;
  float time;
  tm_point(){}
  tm_point(float _x, float _y, float _z, uint16_t _ring, uint16_t _azimuth, float _distance, float _intensity, float _time)
  :x(_x), y(_y), z(_z), ring(_ring), azimuth(_azimuth), distance(_distance),intensity(_intensity),time(_time)
  {
  }
}tm_point_t;

/** \brief timoo data conversion class */
class RawData
{
public:
  RawData()
  {    
  }
  RawData(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~RawData()
  {
  }

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns an optional calibration
   */
  boost::optional<timoo_pointcloud::Calibration> setup(ros::NodeHandle private_nh);


  /** \brief Set up for data processing offline.
   * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
   * this method is useful if unpacking data directly from bag files, without passing
   * through a communication overhead.
   *
   * @param calibration_file path to the calibration file
   * @param max_range_ cutoff for maximum range
   * @param min_range_ cutoff for minimum range
   * @returns 0 if successful;
   *           errno value for failure
   */
  int setupOffline(std::string calibration_file, double max_range_, double min_range_);

  void unpack(const timoo_msgs::timooPacket& pkt, DataContainerBase& data,
              const ros::Time& scan_start_time);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int scansPerPacket() const;
  
  void unpackDifop(const timoo_msgs::timooStatus& dpkt);

  bool compareLine(int);
private:
  /** configuration parameters */
  typedef struct
  {
    std::string model;
    std::string calibrationFile;  ///< calibration file name
    double max_range;             ///< maximum range to publish
    double min_range;             ///< minimum range to publish
    int left_min_angle;                ///< minimum angle to publish
    int left_max_angle;                ///< maximum angle to publish
    int right_min_angle;                ///< minimum angle to publish
    int right_max_angle;                ///< maximum angle to publish
    std::string hideline;

    double tmp_min_angle;
    double tmp_max_angle;
  }
  Config;
  Config config_;
  
  std::vector<int > hidelineNum;
  /**
   * Calibration file
   */
  timoo_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  // timing offset lookup table
  std::vector< std::vector<float> > timing_offsets;

 //去噪，转速、去噪参数，去噪阈值
  std::vector<int> filter_threshold;
  std::vector<std::vector<tm_point>> filter_pcloud;


  /** \brief setup per-point timing offsets
   * 
   *  Runs during initialization and determines the firing time for each point in the scan
   * 
   *  NOTE: Does not support all sensors yet (TM16, TM32, and hdl32 are currently supported)
   */
  bool buildTimings();

  /** add private function to handle the TM16 **/
  void unpack_tm16(const timoo_msgs::timooPacket& pkt, DataContainerBase& data,
                    const ros::Time& scan_start_time);

  bool filter_tm16(const tm_point& src_point , tm_point& dst_point) ;  //去噪，转速、去噪参数，去噪阈值

};

}  // namespace timoo_rawdata

#endif  // timoo_POINTCLOUD_RAWDATA_H
