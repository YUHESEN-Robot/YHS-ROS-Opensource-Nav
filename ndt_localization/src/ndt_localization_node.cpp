#include "ndt_localization_node.hpp"

NdtLocalization::NdtLocalization() : tf2_listener_(tf2_buffer_)
{
  initializeParameters();
  initializeTopics();
  initializeRegistration();

  timer_ = nh_.createTimer(ros::Duration(0.5), &NdtLocalization::init, this);
}

NdtLocalization::~NdtLocalization()
{
}

void NdtLocalization::initializeParameters()
{
  ROS_INFO("initializeParameters");

  ros::NodeHandle private_nh("/ndt_localization");

  private_nh.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh.param("registration_method", registration_method_, std::string("NDT"));
  private_nh.param("ndt_resolution", ndt_resolution_, 0.0);
  private_nh.param("ndt_step_size", ndt_step_size_, 0.0);
  private_nh.param("transform_epsilon", transform_epsilon_, 0.0);
  private_nh.param("max_iterations", max_iterations_, 0.0);
  private_nh.param("num_threads", num_threads_, 0);
  private_nh.param("lidar_voxel_leaf_size", lidar_voxel_leaf_size_, 0.0);
  private_nh.param("pcd_voxel_leaf_size", pcd_voxel_leaf_size_, 0.0);
  private_nh.param("lidar_z_min", lidar_z_min_, 0.0);
  private_nh.param("lidar_z_max", lidar_z_max_, 0.0);

  private_nh.param("lidar_topic", lidar_topic_, std::string("timoo_points"));
  private_nh.param("imu_topic", imu_topic_, std::string("imu_data"));
  private_nh.param("odom_topic", odom_topic_, std::string("odom"));

  private_nh.param("gnss_odom_topic", gnss_odom_topic_, std::string("/odometry/gps"));
  private_nh.param("heading_topic", heading_topic_, std::string("/gps/rpy"));

  private_nh.param("lidar_max_range", lidar_max_range_, 0.0);
  private_nh.param("lidar_min_range", lidar_min_range_, 0.0);
  private_nh.param("use_pcd_map", use_pcd_map_, false);
  private_nh.param("map_path", map_path_, std::string("/home/yhs/Downloads/LOAM/cloudGlobal.pcd"));
  private_nh.param("set_initial_pose", set_initial_pose_, false);
  private_nh.param("initial_pose_x", initial_pose_x_, 0.0);
  private_nh.param("initial_pose_y", initial_pose_y_, 0.0);
  private_nh.param("initial_pose_z", initial_pose_z_, 0.0);
  private_nh.param("initial_pose_qx", initial_pose_qx_, 0.0);
  private_nh.param("initial_pose_qy", initial_pose_qy_, 0.0);
  private_nh.param("initial_pose_qz", initial_pose_qz_, 0.0);
  private_nh.param("initial_pose_qw", initial_pose_qw_, 0.0);
  private_nh.param("use_odom", use_odom_, false);
  private_nh.param("use_imu", use_imu_, false);
  private_nh.param("enable_debug", enable_debug_, false);

  ROS_INFO("global_frame_id: %s", global_frame_id_.c_str());
  ROS_INFO("odom_frame_id: %s", odom_frame_id_.c_str());
  ROS_INFO("base_frame_id: %s", base_frame_id_.c_str());
  ROS_INFO("registration_method: %s", registration_method_.c_str());
  ROS_INFO("ndt_resolution: %lf", ndt_resolution_);
  ROS_INFO("ndt_step_size: %lf", ndt_step_size_);
  ROS_INFO("transform_epsilon: %lf", transform_epsilon_);
  ROS_INFO("max_iterations: %lf", max_iterations_);
  ROS_INFO("num_threads: %d", num_threads_);
  ROS_INFO("lidar_voxel_leaf_size: %lf", lidar_voxel_leaf_size_);
  ROS_INFO("pcd_voxel_leaf_size: %lf", pcd_voxel_leaf_size_);
  ROS_INFO("lidar_z_min: %lf", lidar_z_min_);
  ROS_INFO("lidar_z_max: %lf", lidar_z_max_);

  ROS_INFO("lidar_topic: %s", lidar_topic_.c_str());
  ROS_INFO("imu_topic: %s", imu_topic_.c_str());
  ROS_INFO("odom_topic: %s", odom_topic_.c_str());

  ROS_INFO("gnss_odom_topic: %s", gnss_odom_topic_.c_str());
  ROS_INFO("heading_topic: %s", heading_topic_.c_str());

  ROS_INFO("lidar_max_range: %lf", lidar_max_range_);
  ROS_INFO("lidar_min_range: %lf", lidar_min_range_);
  ROS_INFO("use_pcd_map: %d", use_pcd_map_);
  ROS_INFO("map_path: %s", map_path_.c_str());
  ROS_INFO("set_initial_pose: %d", set_initial_pose_);
  ROS_INFO("use_odom: %d", use_odom_);
  ROS_INFO("use_imu: %d", use_imu_);
  ROS_INFO("enable_debug: %d", enable_debug_);
}

void NdtLocalization::initializeTopics()
{
  // 话题订阅发布
  initial_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("initial_map", 5);
  cloud_sub_ = nh_.subscribe(lidar_topic_, 1, &NdtLocalization::cloudCallback, this);
  initial_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &NdtLocalization::initialPoseReceived, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &NdtLocalization::odomReceived, this);

  gnss_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(gnss_odom_topic_, 1, &NdtLocalization::gnssOdomReceived, this);

  gnss_yaw_sub_ = nh_.subscribe<nmea_ros_driver::GPSRPY>(heading_topic_, 1, &NdtLocalization::gnssRpyReceived, this);

}

void NdtLocalization::initializeRegistration()
{
  ROS_INFO("initializeRegistration");

  boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt->setStepSize(ndt_step_size_);
  ndt->setResolution(ndt_resolution_);
  ndt->setTransformationEpsilon(transform_epsilon_);
  ndt->setMaximumIterations(max_iterations_);
  // 线程数
  ndt->setNumThreads(num_threads_);
  registration_ = ndt;
}

void NdtLocalization::init(const ros::TimerEvent &)
{
  if (set_initial_pose_)
  {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> msg(new geometry_msgs::PoseWithCovarianceStamped());

    msg->header.stamp = ros::Time::now();
    ;
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    initialPoseReceived(msg);
  }

  if (use_pcd_map_)
  {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, *map_cloud_ptr) == -1)
    {
      ROS_ERROR("Failed to load %s point cloud data!", map_path_.c_str());
      ros::shutdown();
      return;
    }

    ROS_INFO("PointCloud width: %d, height: %d, points: %ld", map_cloud_ptr->width, map_cloud_ptr->height, map_cloud_ptr->points.size());

    // 降采样
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(map_cloud_ptr);
    voxel_grid.setLeafSize(pcd_voxel_leaf_size_, pcd_voxel_leaf_size_, pcd_voxel_leaf_size_); // 设置体素的大小
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_grid.filter(*downsampled_cloud);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*downsampled_cloud, map_msg);
    map_msg.header.frame_id = global_frame_id_;
    map_msg.header.stamp = ros::Time::now();

    initial_map_pub_.publish(map_msg);

    ROS_INFO("Initil Map Published");

    registration_->setInputTarget(downsampled_cloud);

    map_recieved_ = true;
  }

  timer_.stop();
}

void NdtLocalization::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (msg->header.frame_id != global_frame_id_)
  {
    ROS_WARN("initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_received_ = true;

  std::lock_guard<std::mutex> lock(mtx_);

  use_init_pose_ = true;

  init_pose_stamped_.header = msg->header;
  init_pose_stamped_.pose = msg->pose.pose;
  init_pose_stamped_.pose.position.z = 0;

  ROS_INFO("initialPoseReceived : (%.3f  %.3f  %.3f  %.3f)", init_pose_stamped_.pose.position.x, init_pose_stamped_.pose.position.y,
           init_pose_stamped_.pose.position.z, tf2::getYaw(init_pose_stamped_.pose.orientation));
}

void NdtLocalization::publishTf(const geometry_msgs::TransformStamped &tf_pose)
{
  try
  {
/*        const geometry_msgs::TransformStamped base_to_odom_tf = tf2_buffer_.lookupTransform(
            base_frame_id_, odom_frame_id_, ros::Time(0));

        // 计算 map 到 odom 的变换
        geometry_msgs::TransformStamped map_to_odom_tf;
        map_to_odom_tf.header.frame_id = global_frame_id_;
        map_to_odom_tf.child_frame_id = odom_frame_id_;
        map_to_odom_tf.header.stamp = ros::Time::now();

        // 将 base_link 到 odom 的变换与 map 到 base_link 的变换相乘得到 map 到 odom 的变换
        tf2::doTransform(base_to_odom_tf, map_to_odom_tf, tf_pose);*/

    tf2_broadcaster_.sendTransform(tf_pose);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Please publish TF %s to %s", odom_frame_id_.c_str(), base_frame_id_.c_str());
    return;
  }
}

void NdtLocalization::gnssRpyReceived(const nmea_ros_driver::GPSRPY::ConstPtr &msg)
{
  // //0~90 => 90~0   90~270 => 0~-180    270~360 => 180~90
  // double yaw = msg->y;
  // if(yaw <= 90)
  // {
  //   gnss_yaw_ = (-yaw + 90) / 180.0 * 3.1415926;
  // }
  // else if(yaw > 90 && yaw <= 270)
  // {
  //   gnss_yaw_ = (90 - yaw) / 180.0 * 3.1415926;
  // }
  // else
  // {
  //   gnss_yaw_ = (270 - yaw + 180) / 180.0 * 3.1415926;
  // }

  //180~0 => 0~180   360~180 => -180~0    
  double yaw = msg->y;
  if(yaw <= 180 && yaw >= 0)
  {
    gnss_yaw_ = (-yaw + 180) / 180.0 * 3.1415926;
  }
  else
  {
    gnss_yaw_ = (-yaw + 180) / 180.0 * 3.1415926;
  }
  
//  ROS_WARN(" %.3f  %.3f  %.3f  %.3f", msg->y, gnss_init_yaw_,yaw,gnss_yaw_);
}

void NdtLocalization::gnssOdomReceived(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(match_core_ < 0.3) 
    return;

  static ros::Time last_time = ros::Time::now();
  ros::Time now_time = ros::Time::now();
  if( (now_time - last_time).toSec() < 3.0) return;
  last_time = ros::Time::now();

  ROS_INFO("Received Gnss Initial Pose");

  init_pose_stamped_.header.stamp = ros::Time::now();
  init_pose_stamped_.header.frame_id = "map";
  init_pose_stamped_.pose = msg->pose.pose;

  init_pose_stamped_.pose.position.z = 0.0;


  static int i = -2;

  tf2::Quaternion quat;

  double yaw_tem = gnss_yaw_ + i * 0.088;

  if(yaw_tem > 3.1415) yaw_tem = 6.28 - yaw_tem;
  if(yaw_tem < -3.1415) yaw_tem = 6.28 + yaw_tem;

  quat.setRPY(0,0,yaw_tem);
  geometry_msgs::Quaternion msg_quat;
  tf2::convert(quat,msg_quat);

  init_pose_stamped_.pose.orientation = msg_quat;

  i ++;
  if(i > 2) i = -2;

  std::lock_guard<std::mutex> lock(mtx_);
  initialpose_received_ = true;
  use_init_pose_ = true;
}

void NdtLocalization::odomReceived(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Check if use_odom_ is true
  if (!use_odom_)
  {
    return;
  }

  static int8_t del_num = 0;
  del_num++;
  if (del_num < 2)
  {
    return;
  }
  if (del_num == 2)
  {
    del_num = 0;
  }

  // Calculate time difference
  double current_odom_received_time = msg->header.stamp.sec +
                                      msg->header.stamp.nsec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */ || dt_odom < 0.0 /* [sec] */)
  {
    ROS_WARN("Odom time interval is invalid");
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  tf2::Quaternion cur_q;
  double cur_roll, cur_pitch, cur_yaw;
  tf2::fromMsg(corrent_pose_stamped_.pose.orientation, cur_q);
  tf2::Matrix3x3(cur_q).getRPY(cur_roll, cur_pitch, cur_yaw);

  tf2::Quaternion odom_q;
  tf2::fromMsg(msg->pose.pose.orientation, odom_q);
  tf2::Matrix3x3(odom_q).getRPY(odom_roll_, odom_pitch_, odom_yaw_);

  static double last_yaw = 0.0;

  double del_yaw = odom_yaw_ - last_yaw;

  last_yaw = odom_yaw_;

  double del_x = msg->twist.twist.linear.x * dt_odom;

  double offset_odom_x = 0.0;
  double offset_odom_y = 0.0;
  double offset_odom_z = 0.0;
  
  offset_odom_x = std::cos(-odom_pitch_) * std::cos(del_yaw) * del_x;
  offset_odom_y = std::cos(-odom_pitch_) * std::sin(del_yaw) * del_x;
  offset_odom_z = std::sin(-odom_pitch_) * del_x;

  tf2::Quaternion pre_q;
  pre_q.setRPY(odom_roll_,odom_pitch_,cur_yaw + del_yaw);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(pre_q);

//  RCLCPP_WARN(this->get_logger(),"yaw  : %.3f",cur_yaw + del_yaw);

  corrent_pose_stamped_.pose.position.x += offset_odom_x;
  corrent_pose_stamped_.pose.position.y += offset_odom_y;
  corrent_pose_stamped_.pose.position.z += offset_odom_z;
  corrent_pose_stamped_.pose.orientation = quat_msg;

  // Publish transformed pose
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = base_frame_id_;
  transform_stamped.transform.translation.x = corrent_pose_stamped_.pose.position.x;
  transform_stamped.transform.translation.y = corrent_pose_stamped_.pose.position.y;
  //  transform_stamped.transform.translation.z = corrent_pose_stamped_.pose.position.z;
  transform_stamped.transform.translation.z = 0;
  transform_stamped.transform.rotation = corrent_pose_stamped_.pose.orientation;

  //  publishTf(transform_stamped);
}

void NdtLocalization::fromROSMsg_MY(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // Get the field structure of this point cloud
  uint32_t pointBytes = cloud_msg->point_step;
  uint32_t offset_x = 0;
  uint32_t offset_y = 0;
  uint32_t offset_z = 0;
  uint32_t offset_int = 0;
  for (uint32_t f = 0; f < cloud_msg->fields.size(); ++f)
  {
    if (cloud_msg->fields[f].name == "x")
      offset_x = cloud_msg->fields[f].offset;
    if (cloud_msg->fields[f].name == "y")
      offset_y = cloud_msg->fields[f].offset;
    if (cloud_msg->fields[f].name == "z")
      offset_z = cloud_msg->fields[f].offset;
    if (cloud_msg->fields[f].name == "intensity")
      offset_int = cloud_msg->fields[f].offset;
  }

  // Populate point cloud object
  for (uint32_t p = 0; p < cloud_msg->width * cloud_msg->height; ++p)
  {
    pcl::PointXYZI newPoint;

    newPoint.x = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_x);
    newPoint.y = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_y);
    newPoint.z = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_z);
    newPoint.intensity = *(unsigned char *)(&cloud_msg->data[0] + (pointBytes * p) + offset_int);

    cloud->points.push_back(newPoint);
  }
  cloud->height = cloud_msg->height;
  cloud->width = cloud_msg->width;
}

// 点云数据回调函数
void NdtLocalization::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!map_recieved_ || !initialpose_received_)
  {
    return;
  }
  ROS_INFO_ONCE("cloudReceived");

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // fromROSMsg_MY(msg, cloud_ptr);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  const std::string lidar_frame = msg->header.frame_id;
  geometry_msgs::TransformStamped lidar_to_base_tf;
  try
  {
    lidar_to_base_tf = tf2_buffer_.lookupTransform(base_frame_id_, lidar_frame, ros::Time(0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Please publish TF %s to %s", base_frame_id_.c_str(), lidar_frame.c_str());
    return;
  }

  const Eigen::Affine3d base_to_lidar_affine = tf2::transformToEigen(lidar_to_base_tf);
  const Eigen::Matrix4f base_to_lidar_matrix = base_to_lidar_affine.matrix().cast<float>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr to_base_link_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_ptr, *to_base_link_cloud_ptr, base_to_lidar_matrix);

  // Voxel Grid Downsampling
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(to_base_link_cloud_ptr);
  voxel_grid.setLeafSize(lidar_voxel_leaf_size_, lidar_voxel_leaf_size_, lidar_voxel_leaf_size_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid.filter(*downsampled_cloud);

  // Z-axis Clipping
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(downsampled_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(lidar_z_min_, lidar_z_max_);
  pass.filter(*downsampled_cloud);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto &p : downsampled_cloud->points)
  {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (lidar_min_range_ < r && r < lidar_max_range_)
    {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  Eigen::Affine3d affine;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (use_init_pose_)
    {
      corrent_pose_stamped_ = init_pose_stamped_;
      use_init_pose_ = false;
    }
    tf2::fromMsg(corrent_pose_stamped_.pose, affine);
  }

  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ros::Time time_align_start = ros::Time::now();
  registration_->align(*output_cloud, init_guess);
  ros::Time time_align_end = ros::Time::now();
  if (!registration_->hasConverged())
  {
    ROS_WARN("The registration didn't converge.");
    return;
  }

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_eig);

  {
    std::lock_guard<std::mutex> lock(mtx_);

    double cur_yaw = tf2::getYaw(quat_msg);
    tf2::Quaternion cal_q;
    cal_q.setRPY(odom_roll_, odom_pitch_, cur_yaw);
    geometry_msgs::Quaternion cal_msg = tf2::toMsg(cal_q);

    corrent_pose_stamped_.header.stamp = msg->header.stamp;
    corrent_pose_stamped_.pose.position.x = static_cast<double>(final_transformation(0, 3));
    corrent_pose_stamped_.pose.position.y = static_cast<double>(final_transformation(1, 3));
    corrent_pose_stamped_.pose.position.z = static_cast<double>(final_transformation(2, 3));
    corrent_pose_stamped_.pose.orientation = cal_msg;
    // pose_pub_.publish(corrent_pose_stamped_);

    // ROS_INFO("%.3f  %.3f  %.3f  %.3f", corrent_pose_stamped_.pose.position.x, corrent_pose_stamped_.pose.position.y,
            //  corrent_pose_stamped_.pose.position.z, tf2::getYaw(corrent_pose_stamped_.pose.orientation));

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = global_frame_id_;
    transform_stamped.child_frame_id = base_frame_id_;
    transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
    transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
    transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation = cal_msg;

    publishTf(transform_stamped);
  }

  // path_.poses.push_back(corrent_pose_stamped_);
  // path_pub_.publish(path_);

  match_core_ = registration_->getFitnessScore();

  if (enable_debug_)
  {
    std::cout << "number of filtered cloud points: " << downsampled_cloud->size() << std::endl;
    std::cout << "align time: " << (time_align_end - time_align_start).toSec() << " [sec]" << std::endl;
    std::cout << "has converged: " << (registration_->hasConverged() ? "true" : "false") << std::endl;
    std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;

    double init_cos_angle = 0.5 *
                            (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
                       (final_transformation.coeff(0, 0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle: " << delta_angle * 180 / M_PI << " [deg]" << std::endl;
  }
}
