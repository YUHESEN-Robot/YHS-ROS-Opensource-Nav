#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
    pcl::fromROSMsg(*msg , pointCloudIn);
    int cloudSize = pointCloudIn.points.size();
    for(int i=0;i<cloudSize;i++)
    {
        ROS_INFO("[i= %d] ( %.2f , %.2f , %.2f)", 
        i , 
        pointCloudIn.points[i].x, 
        pointCloudIn.points[i].y, 
        pointCloudIn.points[i].z);
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tm_lidar_data_node");
    ROS_INFO("tm_lidar_data_node start");
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/timoo_points", 1 , callbackPC);
    ros::spin();
    return 0;
}
