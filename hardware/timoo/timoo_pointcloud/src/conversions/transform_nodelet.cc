/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet transforms raw timoo 3D LIDAR packets to a
    PointCloud2 in the /map frame.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "timoo_pointcloud/transform.h"

namespace timoo_pointcloud
{
  class TransformNodelet: public nodelet::Nodelet
  {
  public:

    TransformNodelet() {}
    ~TransformNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Transform> tf_;
  };

  /** @brief Nodelet initialization. */
  void TransformNodelet::onInit()
  {
    tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

} // namespace timoo_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(timoo_pointcloud::TransformNodelet, nodelet::Nodelet)
