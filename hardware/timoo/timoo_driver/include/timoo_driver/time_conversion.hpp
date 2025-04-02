// Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
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

#ifndef timoo_DRIVER_TIME_CONVERSION_HPP
#define timoo_DRIVER_TIME_CONVERSION_HPP

#include <ros/ros.h>
#include <ros/time.h>
#include <timoo_driver/driver.h>
struct gpsInfo{
  bool gps_status;
  double gps_time;
  // Latitu and so on
};

typedef struct{
    uint32_t imu_start_time;
}imu_start_data;

typedef struct{
    uint32_t ros_time;
}ROS_DATA;

union 
{
    uint8_t bytes[4];
    float data;
}bytestofloat;


/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. timoo only returns time since the top of the hour, so if the computer clock
 * and the timoo clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 * 
 * @param stamp timestamp recovered from timoo
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from timoo, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
ros::Time resolveHourAmbiguity(const ros::Time &stamp, const ros::Time &nominal_stamp) {
    const int HALFHOUR_TO_SEC = 1800;
    ros::Time retval = stamp;
    if (nominal_stamp.sec > stamp.sec) {
        if (nominal_stamp.sec - stamp.sec > HALFHOUR_TO_SEC) {
            retval.sec = retval.sec + 2*HALFHOUR_TO_SEC;
        }
    } else if (stamp.sec - nominal_stamp.sec > HALFHOUR_TO_SEC) {
        retval.sec = retval.sec - 2*HALFHOUR_TO_SEC;
    }
    return retval;
}


ros::Time rosTimeFromGpsTimestamp(const uint8_t * const data, const bool gps_status, const ros::Time gps_time) {
 #ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
  #endif
    const int HOUR_TO_SEC = 3600;
    uint8_t buf[4];
    for(int i = 0; i< 4;i++){
        buf[i] = data[i];
    } 
    ROS_DATA* ros_stamp = (ROS_DATA *)buf;

    ros::Time stamp;
   // Get GPS year-month-day-hour
    uint32_t gps_hour = (int)floor(gps_time.toSec()) / HOUR_TO_SEC;
    // GPS Time + Data min-sec[microseconds form MSOP]


    stamp = ros::Time((gps_hour * HOUR_TO_SEC) + ((ros_stamp->ros_time) / 1000000.0),
                                ((ros_stamp->ros_time) % 1000000) * 1000);      
                    
    ros::Time time_nom = ros::Time::now(); // use this to recover the hour
    uint32_t cur_hour = time_nom.sec / HOUR_TO_SEC;

    
    return stamp;
}

ros::Time imuTimeFromGpsTimestamp(timoo_msgs::timooPacket *pkt) {
 time_t gps_time = 0;

 #ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
  #endif
    const int HOUR_TO_SEC = 3600;
 
    uint8_t buf[4];
    for(int i = 0; i< 4;i++){
        buf[i] = pkt->data[1019 + i];
    } 
    imu_start_data* imu_data = (imu_start_data *)buf;
    ros::Time stamp;
   
    if(pkt->data[1000] == 0xaa && pkt->data[1001] == 0x55)
    {
        int year = (uint8_t)pkt->data[36] + 2000;
        int month = (uint8_t)pkt->data[37];
        int day = (uint8_t)pkt->data[38];
        int hour = (uint8_t)pkt->data[39];
        int min = (uint8_t)pkt->data[40];
        int sec = (uint8_t)pkt->data[41];
        
        struct tm timeinfo;
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        bool enable_local_time = false;
        
        if(enable_local_time)
        {
            // Transform to local time if necessary
            struct tm * local_timeinfo;
            time_t local_time;
            time(&local_time);
            local_timeinfo = localtime(&local_time);
            timeinfo.tm_hour = local_timeinfo->tm_hour; // local time
        }
        else
        {
            timeinfo.tm_hour = hour; // UTM
        }
        timeinfo.tm_min = min;
        // timeinfo.tm_sec = sec + 1; // Add 1 sec
        timeinfo.tm_sec = sec; // Add 1 sec
        gps_time = mktime ( &timeinfo );

        uint32_t gps_hour = (int)floor(gps_time) / HOUR_TO_SEC;
       
        stamp = ros::Time((gps_hour * HOUR_TO_SEC) + (ntohl(imu_data->imu_start_time) / 1000000.0)- static_cast<double>(timezone),
                                (ntohl(imu_data->imu_start_time) % 1000000) * 1000);
    }else{
        struct tm timeinfo;
        timeinfo.tm_year = 70;
        timeinfo.tm_mon = 0;
        timeinfo.tm_mday = 1;
        timeinfo.tm_hour = 0;
        timeinfo.tm_min = 0;
        timeinfo.tm_sec = 0;
        gps_time = mktime ( &timeinfo );
        uint32_t gps_hour = (int)floor(gps_time) / HOUR_TO_SEC;

        stamp = ros::Time((gps_hour * HOUR_TO_SEC) + (ntohl(imu_data->imu_start_time) / 1000000.0)- static_cast<double>(timezone),
                                (ntohl(imu_data->imu_start_time) % 1000000) * 1000);
    }
    return stamp;
}



#endif //timoo_DRIVER_TIME_CONVERSION_HPP
