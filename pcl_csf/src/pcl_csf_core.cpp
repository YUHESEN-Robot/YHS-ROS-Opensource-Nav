#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "CSF.h"

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

bool bSloopSmooth;
double cloth_resolution;
int rigidness;
double time_step;
double class_threshold;
int interations;

std::string pcd_path;
double resolution;
double z_min;
double z_max;
int MeanK;
double StddevMulThresh;

void clothSimulationFilter(const vector<csf::Point> &pc, vector<int> &groundIndexes, vector<int> &offGroundIndexes)
{
  // step 1 read point cloud
  CSF csf;
  csf.setPointCloud(pc); // or csf.readPointsFromFile(pointClouds_filepath);
  // pc can be vector< csf::Point > or PointCloud defined in point_cloud.h

  // step 2 parameter settings
  // Among these paramters:
  // time_step  interations class_threshold can remain as defualt in most cases.
  csf.params.bSloopSmooth = bSloopSmooth;
  csf.params.cloth_resolution = cloth_resolution;
  csf.params.rigidness = rigidness;

  csf.params.time_step = time_step;
  csf.params.class_threshold = class_threshold;
  csf.params.interations = interations;

  // step 3 do filtering
  // result stores the index of ground points or non-ground points in the original point cloud

  csf.do_filtering(groundIndexes, offGroundIndexes);
  // csf.do_filtering(groundIndexes, offGroundIndexes,true);
  // if the third parameter is set as true, then a file named "cloth_nodes.txt" will be created,
  // it respresents the cloth nodes.By default, it is set as false
}

void addPointCloud(const vector<int> &index_vec, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
{
  auto &points = cloud_filtered->points;
  const auto &pointclouds = cloud->points;

  for_each(index_vec.begin(), index_vec.end(), [&](const auto &index)
           {
		pcl::PointXYZI pc;
		pc.x = pointclouds[index].x;
		pc.y = pointclouds[index].y;
		pc.z = pointclouds[index].z;
		pc.intensity = pointclouds[index].intensity;

		points.push_back(pc); });

  cloud_filtered->height = 1;
  cloud_filtered->width = cloud_filtered->points.size();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_csf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("bSloopSmooth", bSloopSmooth, true);
  private_nh.param("cloth_resolution", cloth_resolution, 0.5);
  private_nh.param("rigidness", rigidness, 3);
  private_nh.param("time_step", time_step, 0.65);
  private_nh.param("class_threshold", class_threshold, 0.01);
  private_nh.param("interations", interations, 500);

  private_nh.param("pcd_path", pcd_path, std::string("/home/xwqf/Downloads/LOAM/"));
  private_nh.param("resolution", resolution, resolution);
  private_nh.param("z_min", z_min, -2.0);
  private_nh.param("z_max", z_max, 2.0);
  private_nh.param("MeanK", MeanK, 60);
  private_nh.param("StddevMulThresh", StddevMulThresh, resolution);

  // 打印参数
  ROS_INFO("bSloopSmooth: %s", bSloopSmooth ? "true" : "false");
  ROS_INFO("cloth_resolution: %f", cloth_resolution);
  ROS_INFO("rigidness: %d", rigidness);
  ROS_INFO("time_step: %f", time_step);
  ROS_INFO("class_threshold: %f", class_threshold);
  ROS_INFO("interations: %d", interations);

  ROS_INFO("pcd_path: %s", pcd_path.c_str());
  ROS_INFO("resolution: %f", resolution);
  ROS_INFO("z_min: %f", z_min);
  ROS_INFO("z_max: %f", z_max);
  ROS_INFO("MeanK: %d", MeanK);
  ROS_INFO("StddevMulThresh: %f", StddevMulThresh);

  ros::Rate loop_rate(1.0);
  ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

  // Generate pointcloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  std::cout << "正在加载 " << pcd_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read that pcd file\n");
    return (-1);
  }
  std::cout << "加载完成，进行转换，pcd地图越大，转换时间越长，请等待转换完成。。。" << std::endl;

  nav_msgs::OccupancyGrid msg;

  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = resolution;

  double x_min, x_max, y_min, y_max;

  for (int i = 0; i < cloud->points.size() - 1; i++)
  {
    if (i == 0)
    {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }

  vector<csf::Point> pc;
  const auto &pointclouds = cloud->points;
  pc.resize(cloud->size());
  transform(pointclouds.begin(), pointclouds.end(), pc.begin(), [&](const auto &p) -> csf::Point
            {
		csf::Point pp;
		pp.x = p.x;
		pp.y = p.y;
		pp.z = p.z;
		return pp; });

  std::vector<int> groundIndexes, offGroundIndexes;
  clothSimulationFilter(pc, groundIndexes, offGroundIndexes);

  // addPointCloud(groundIndexes, cloud, cloud_filtered);
  // cloud_filtered->points.clear();

  addPointCloud(offGroundIndexes, cloud, cloud_filtered);

  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cloud_filtered);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(MeanK);
  sor.setStddevMulThresh(StddevMulThresh);
  sor.filter(*cloud_filtered);

  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(MeanK);
  sor.setStddevMulThresh(StddevMulThresh);
  sor.filter(*cloud_filtered);

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = int((x_max - x_min) / resolution);
  msg.info.height = int((y_max - y_min) / resolution);

  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (int iter = 0; iter < cloud_filtered->points.size(); iter++)
  {
    int i = int((cloud_filtered->points[iter].x - x_min) / resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud_filtered->points[iter].y - y_min) / resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;

    msg.data[i + j * msg.info.width] = 100;
  }

  std::cout << "转换栅格地图完成，可以保存地图了。。。" << std::endl;
  while (ros::ok())
  {
    map_topic_pub.publish(msg);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
