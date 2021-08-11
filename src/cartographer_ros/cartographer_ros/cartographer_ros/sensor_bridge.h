/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <memory>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
);
using PointXYZIRT = VelodynePointXYZIRT;

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, 
  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
  (uint16_t, ring, ring)(double, timestamp, timestamp))


namespace cartographer_ros {

template<class T>
bool isnan(const T& point){
  return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
}
template<class T>
bool isinf(const T& point){
  return std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
}

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
 public:
  explicit SensorBridge(
      int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
      double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
      ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
      const nav_msgs::Odometry::ConstPtr& msg);
  void HandleOdometryMessage(const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

  std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
      const sensor_msgs::Imu::ConstPtr& msg);
  void HandleImuMessage(const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg,
                                const std::string& sensor_type);

  const TfBridge& tf_bridge() const;

 private:
  void HandleLaserScan(
      const std::string& sensor_id, ::cartographer::common::Time start_time,
      const std::string& frame_id,
      const ::cartographer::sensor::PointCloudWithIntensities& points);
  void HandleRangefinder(const std::string& sensor_id,
                         ::cartographer::common::Time time,
                         const std::string& frame_id,
                         const ::cartographer::sensor::TimedPointCloud& ranges);

  const int num_subdivisions_per_laser_scan_;
  std::map<std::string, cartographer::common::Time>
      sensor_to_previous_subdivision_time_;
  const TfBridge tf_bridge_;
  ::cartographer::mapping::TrajectoryBuilderInterface* const
      trajectory_builder_;

  ::cartographer::common::optional<::cartographer::transform::Rigid3d>
      ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
