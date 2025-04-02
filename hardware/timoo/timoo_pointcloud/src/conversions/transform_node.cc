/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node transforms raw timoo LIDAR packets to PointCloud2
    in the /map frame of reference.

*/

#include <ros/ros.h>
#include "timoo_pointcloud/transform.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "timoo_transform_node");

  // create conversion class, which subscribes to raw data
  timoo_pointcloud::Transform transform(ros::NodeHandle(),
                                           ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
