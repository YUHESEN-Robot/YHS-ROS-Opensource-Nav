#ifndef NDT_LOCALIZATION_HPP
#define NDT_LOCALIZATION_HPP

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "pclomp/ndt_omp.h"

#include "nmea_ros_driver/GPSRPY.h"

class NdtLocalization
{
public:
    NdtLocalization();
    ~NdtLocalization();

private:
    ros::NodeHandle nh_;
    ros::Publisher initial_map_pub_;

    ros::Subscriber initial_pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gnss_odom_sub_;
    ros::Subscriber gnss_yaw_sub_;
    ros::Subscriber cloud_sub_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> registration_;

    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string registration_method_;
    double ndt_resolution_;
    double ndt_step_size_;
    double transform_epsilon_;
    double max_iterations_;
    int num_threads_;
    double lidar_voxel_leaf_size_;
    double pcd_voxel_leaf_size_;
    double lidar_z_min_;
    double lidar_z_max_;

    std::string lidar_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string gnss_odom_topic_;
    std::string heading_topic_;

    double lidar_max_range_;
    double lidar_min_range_;
    bool use_pcd_map_;
    bool use_init_pose_;
    std::string map_path_;
    bool set_initial_pose_;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_z_;
    double initial_pose_qx_;
    double initial_pose_qy_;
    double initial_pose_qz_;
    double initial_pose_qw_;
    bool use_odom_;
    bool use_imu_;
    bool enable_debug_;

    bool map_recieved_;
    bool initialpose_received_;

    double odom_roll_{0.0};
    double odom_pitch_{0.0};
    double odom_yaw_{0.0};

    double last_odom_received_time_;

    double last_yaw_,cur_yaw_;

    double match_core_;
    double gnss_yaw_;

    geometry_msgs::PoseStamped corrent_pose_stamped_;
    geometry_msgs::PoseStamped init_pose_stamped_;

    std::mutex mtx_;

    ros::Timer timer_;

    void initializeParameters();
    void initializeTopics();
    void initializeRegistration();
    void init(const ros::TimerEvent&);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void publishTf(const geometry_msgs::TransformStamped &tf_pose);
    void odomReceived(const nav_msgs::Odometry::ConstPtr &msg);
    void gnssOdomReceived(const nav_msgs::Odometry::ConstPtr &msg);
    void gnssRpyReceived(const nmea_ros_driver::GPSRPY::ConstPtr &msg);
    void fromROSMsg_MY(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

};

#endif // NDT_LOCALIZATION_HPP
