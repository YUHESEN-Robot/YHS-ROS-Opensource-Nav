/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Kaarta Inc, Shawn Hanna
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  timoo 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw timoo LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw timoo data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Shawn Hanna
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <timoo_pointcloud/rawdata.h>

namespace timoo_rawdata
{
inline float SQR(float val) { return val*val; }

std::vector<std::string> split(const std::string& str, const std::string& delim )
{
   std::vector<std::string> res;
    if("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型
    char * strs = new char[str.length() + 1] ; //不要忘了
    strcpy(strs, str.c_str());
  
    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());
  
    char *p = strtok(strs, d);
    while(p) {
        std::string s = p; //分割得到的字符串转换为string类型
        res.push_back(s); //存入结果数组
        p = strtok(NULL, d);
    }
  
    return res;
}

  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

 RawData::RawData(ros::NodeHandle node, ros::NodeHandle private_nh) {

    std::string str;
    private_nh.param<std::string>("filter_threshold",str," 0,0,0,0");
    std::vector<std::string> res  = split(str,",");
    for(int i = 0; i< res.size(); ++i)
    {
        filter_threshold.push_back(stoi(res[i]));
        //  printf("res[%d]=%d",i,filter_threshold[i] );
    }
    printf("\n");
    filter_pcloud.resize(16);
}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    // //converting angle parameters into the timoo reference (rad)
    // config_.tmp_min_angle = view_direction + view_width/2;
    // config_.tmp_max_angle = view_direction - view_width/2;
    
    // //computing positive modulo to keep theses angles into [0;2*M_PI]
    // config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    // config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    // //converting into the hardware timoo ref (negative yaml and degrees)
    // //adding 0.5 perfomrs a centered double to int conversion 
    // config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    // config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    // if (config_.min_angle == config_.max_angle)
    // {
    //   //avoid returning empty cloud if min_angle = max_angle
    //   config_.min_angle = 0;
    //   config_.max_angle = 36000;
    // }
  }

  int RawData::scansPerPacket() const
  {
    if( calibration_.num_lasers == 16)
    {
      return BLOCKS_PER_PACKET * TM16_FIRINGS_PER_BLOCK *
          TM16_SCANS_PER_FIRING;
    }
    else{
      return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
    }
  }

void split1(const std::string &s, std::vector<std::string> &elems, char delim)
{
	std::stringstream ss;
	s.substr(0);
	ss.str(s);
	std::string iteam;
	while (std::getline(ss, iteam, ','))
	{
		elems.push_back(iteam);
	}
}

void getNumberFromString(std::string msg, std::vector<int>& result)
{
	std::vector<std::string> temp;
  char a ;
	split1(msg,temp,a);

	size_t num_size = temp.size();
	result.clear();
	result.resize(num_size);
	for (int i = 0; i<num_size; i++)
	{
		int num = std::atoi(temp[i].c_str());
		result[i] = num;
	}
}

  /**
   * Build a timing table for each block/firing. Stores in timing_offsets vector
   */
  bool RawData::buildTimings(){
    // TM16    
    if (config_.model == "TM16"){
      // timing table calculation, from timoo user manual
      timing_offsets.resize(12);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(32);
      }
      // constants
      double full_firing_cycle = 49.152 * 1e-6; // seconds
      double single_firing = 3.072 * 1e-6; // seconds 
      double single_packet_cycle;
      double dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (dual_mode){
            dataBlockIndex = (x - (x % 2)) + (y / 16);
            single_packet_cycle = 589.824 * 1e-6; 
          }
          else{
            dataBlockIndex = (x * 2) + (y / 16);
            single_packet_cycle = 1179.648 * 1e-6;
          }
          dataPointIndex = y % 16;
          //timing_offsets[block][firing]
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * (dataPointIndex + 1)) - single_packet_cycle;
        }
      }
    }
    else{
      timing_offsets.clear();
      ROS_WARN("Timings not supported for model %s", config_.model.c_str());
    }

    if (timing_offsets.size()){
      // ROS_INFO("timoo TIMING TABLE:");
      // for (size_t x = 0; x < timing_offsets.size(); ++x){
      //   for (size_t y = 0; y < timing_offsets[x].size(); ++y){
      //     printf("%04.3f ", timing_offsets[x][y] * 1e6);
      //   }
      //   printf("\n");
      // }
      return true;
    }
    else{
      ROS_WARN("NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED timoo SENSOR?");
    }
    return false;
  }

  /** Set up for on-line operation. */
  boost::optional<timoo_pointcloud::Calibration> RawData::setup(ros::NodeHandle private_nh)
  {
    private_nh.param("model", config_.model, std::string("TM16"));
  
    private_nh.param<int>("left_min_angle", config_.left_min_angle, 0);
    private_nh.param<int>("left_max_angle", config_.left_max_angle, 180);
    private_nh.param<int>("right_min_angle", config_.right_min_angle, 180);
    private_nh.param<int>("right_max_angle", config_.right_max_angle, 360);
    private_nh.param<std::string>("hide_line",config_.hideline,"");
    
    getNumberFromString(config_.hideline,hidelineNum);
    
  //  std::cout<<config_.left_min_angle<<" "<<config_.left_max_angle<<" "<<config_.right_min_angle<<" "<<config_.right_max_angle<<std::endl;
          
    if (config_.left_min_angle<0 || config_.left_min_angle > 360 || config_.left_max_angle<0||config_.left_min_angle>360||config_.left_min_angle == config_.left_max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.left_min_angle = 0;
      config_.left_max_angle = 36000;
    }else{
      config_.left_min_angle  = config_.left_min_angle *100 ;
      config_.left_max_angle = config_.left_max_angle * 100 ;
    }
     if (config_.right_min_angle<0 || config_.right_min_angle > 360 || config_.right_max_angle<0||config_.right_min_angle>360||config_.right_min_angle == config_.right_max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.right_min_angle = 0;
      config_.right_max_angle = 36000;
    }else{
      config_.right_min_angle  = config_.right_min_angle *100 ;
      config_.right_max_angle = config_.right_max_angle * 100 ;
    }


    buildTimings();

    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
      {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("timoo_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
      }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);
    
    calibration_.read(config_.calibrationFile);
    
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " <<
          config_.calibrationFile);
      return boost::none;
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
   return calibration_;
  }

  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

    config_.max_range = max_range_;
    config_.min_range = min_range_;
    ROS_INFO_STREAM("data ranges to publish: ["
      << config_.min_range << ", "
      << config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
      return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
      }
    return 0;
  }


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const timoo_msgs::timooPacket &pkt, DataContainerBase& data, const ros::Time& scan_start_time)
  {
    using timoo_pointcloud::LaserCorrection;
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    /** special parsing for the TM16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_tm16(pkt, data, scan_start_time);
      return;
    }

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();
    
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old timoo_common implementation

      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
        const uint8_t laser_number  = j + bank_origin;

        const LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

        /** Position Calculation */
        const raw_block_t &block = raw->blocks[i];
        union two_bytes tmp;
        tmp.bytes[0] = block.data[k];
        tmp.bytes[1] = block.data[k+1];
        if (tmp.bytes[0]==0 &&tmp.bytes[1]==0 ) //no laser beam return
        {
          continue;
        }

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((block.rotation >= config_.left_min_angle
             && block.rotation <= config_.left_max_angle
             && config_.left_min_angle < config_.left_max_angle)
             ||(config_.left_min_angle > config_.left_max_angle 
             && (raw->blocks[i].rotation <= config_.left_max_angle 
             || raw->blocks[i].rotation >= config_.left_min_angle))   ||
             (block.rotation >= config_.right_min_angle
             && block.rotation <= config_.right_max_angle
             && config_.right_min_angle < config_.right_max_angle)
             ||(config_.right_min_angle > config_.right_max_angle 
             && (raw->blocks[i].rotation <= config_.right_max_angle 
             || raw->blocks[i].rotation >= config_.right_min_angle))){

          float distance = tmp.uint * calibration_.distance_resolution_m;
          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[block.rotation] * cos_rot_correction +
            sin_rot_table_[block.rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[block.rotation] * cos_rot_correction -
            cos_rot_table_[block.rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the timoo manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
  
          intensity = raw->blocks[i].data[k+2];
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * 
            SQR(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          float time = 0;
          if (timing_offsets.size())
            time = timing_offsets[i][j] + time_diff_start_to_this_packet;

          data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[i].rotation, distance, intensity, time);
        }
      }
      data.newLine();
    }
  }
  
  void RawData::unpackDifop(const timoo_msgs::timooStatus& dpkt){
    for(int i = 0; i < 16; i++){
      timoo_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[i];
      // int AngleNoToChannelC16[16] = {0,2,4,6,8,10,12,14,1,3,5,7,9,11,13,15};
      calibration_.laser_corrections[i].vert_correction = dpkt.vertical_angle_list[i] * M_PI /180;

      calibration_.laser_corrections[i].cos_vert_correction = cosf(calibration_.laser_corrections[i].vert_correction);
      calibration_.laser_corrections[i].sin_vert_correction = sinf(calibration_.laser_corrections[i].vert_correction);
    }    
  }

  /** @brief convert raw TM16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  bool RawData::compareLine(int a)
  {
      // printf("num == %d\n",hidelineNum.size());
      for(int i = 0 ; i < hidelineNum.size();i++)
      {      
       
       if(a == hidelineNum[i])
       return true;
      }
       return false;
  }


  void RawData::unpack_tm16(const timoo_msgs::timooPacket &pkt, DataContainerBase& data, const ros::Time& scan_start_time)
  {

    float azimuth;
    float azimuth_diff;
    int raw_azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid TM-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
	raw_azimuth_diff = raw->blocks[block+1].rotation - raw->blocks[block].rotation;
        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
	// some packets contain an angle overflow where azimuth_diff < 0 
	if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
	  {
	    //ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
	    // if last_azimuth_diff was not zero, we can assume that the timoo's speed did not change very much and use the same difference
	    if(last_azimuth_diff > 0){
	      azimuth_diff = last_azimuth_diff;
	    }
	    // otherwise we are not able to use this data
	    // TODO: we might just not use the second 16 firings
	    else{
	      continue;
	    }
	  }
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < TM16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < TM16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          timoo_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];
          
          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*TM16_DSR_TOFFSET) + (firing*TM16_FIRING_TOFFSET)) / TM16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          

       /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ( (
               compareLine(corrections.laser_ring) &&(((
                 azimuth_corrected >= config_.left_min_angle 
               && azimuth_corrected <= config_.left_max_angle 
               && config_.left_min_angle < config_.left_max_angle)
               ||(config_.left_min_angle > config_.left_max_angle 
               && (azimuth_corrected <= config_.left_max_angle 
               || azimuth_corrected >= config_.left_min_angle)) )||
             ((azimuth_corrected >= config_.right_min_angle 
               && azimuth_corrected <= config_.right_max_angle 
               && config_.right_min_angle < config_.right_max_angle)
               ||(config_.right_min_angle > config_.right_max_angle 
               && (azimuth_corrected <= config_.right_max_angle 
               || azimuth_corrected >= config_.right_min_angle)))))){
                 
              
              
            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * calibration_.distance_resolution_m;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the timoo manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
              SQR(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
            float time = 0;
            if (timing_offsets.size())
              time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

            
            //filter
            tm_point src_point(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time);
            tm_point dst_point;

             if( filter_tm16(src_point, dst_point))
                 data.addPoint(dst_point.x, dst_point.y, dst_point.z, dst_point.ring,dst_point.azimuth, dst_point.distance, dst_point.intensity, dst_point.time);
           }
        }
        data.newLine();
      }
    }
  }





bool RawData::filter_tm16(const tm_point& src_point , tm_point& dst_point)   //去噪，转速、去噪参数，去噪阈值   //去噪，转速、去噪参数，去噪阈值
{

  bool return_flag = false;
  if(filter_pcloud[src_point.ring].size() == 3)
  {
      //printf("ring :%d  filter_pcloud size =  %d \n", src_point.ring,filter_pcloud[src_point.ring].size() );
      float delta1 = fabs(filter_pcloud[src_point.ring][0].distance - filter_pcloud[src_point.ring][1].distance) * 200;
      float delta2 = fabs(filter_pcloud[src_point.ring][2].distance - filter_pcloud[src_point.ring][1].distance) * 200;
      if(src_point.distance < 2.56)
      {
        if(delta1 < 8 * 0.1* filter_threshold[0] && delta2 < 8 * 0.1 * filter_threshold[0])
        {
            //printf("1 : %lf  2 : %lf   3:%lf\n",filter_pcloud[src_point.ring][0].distance,filter_pcloud[src_point.ring][1].distance,filter_pcloud[src_point.ring][2].distance);
            dst_point = filter_pcloud[src_point.ring][1];
            return_flag = true;
        }
      }
      else if(src_point.distance < 5.12)
      {
        if(delta1 < 26 * 0.1* filter_threshold[1] && delta2 < 26 * 0.1* filter_threshold[1])
        {
            dst_point = filter_pcloud[src_point.ring][1];
            return_flag = true;
        }
      }else if(src_point.distance < 10.24)
      {
         if(delta1 < 48 * 0.1* filter_threshold[2] && delta2 < 48 * 0.1*filter_threshold[2])
        {
            dst_point = filter_pcloud[src_point.ring][1];
            return_flag = true;
        }
      }else
      {
         if(delta1 < 1024 * 0.1* filter_threshold[3] && delta2 < 1024 * 0.1* filter_threshold[3])
        {
            dst_point = filter_pcloud[src_point.ring][1];
            return_flag = true;
        }
      }
      filter_pcloud[src_point.ring].erase(filter_pcloud[src_point.ring].begin()); 
  }
  filter_pcloud[src_point.ring].push_back(src_point);

  return return_flag;
}




} // namespace timoo_rawdata
