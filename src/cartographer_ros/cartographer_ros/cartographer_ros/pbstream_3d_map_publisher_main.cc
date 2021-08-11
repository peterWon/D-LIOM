/*
 * Copyright 2018 The Cartographer Authors
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

// Publishes a frozen nav_msgs/OccupancyGrid map from serialized submaps.

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "cartographer_ros/urdf_reader.h"
#include "cartographer_ros/msg_conversion.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(bag_filename, "", "Bag file");
DEFINE_string(urdf_filename, "", "URDF file");
DEFINE_string(tracking_frame, "base_link", "Frame of tracking.");
DEFINE_double(rate, 100, "Publishing rate.");

namespace cartographer_ros {
namespace {
using namespace cartographer;
using namespace cartographer::common;
namespace carto = ::cartographer;
constexpr float kPointCloudComponentFourMagic = 1.;
sensor_msgs::PointCloud2 PreparePointCloud2Message(const ros::Time timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}


std::string CheckNoLeadingSlash(const std::string& frame_id) {
  std::string frame_id_out = frame_id;
  if (frame_id.size() > 0) {
    if(frame_id[0] == '/'){
      // LOG(WARNING)<< "The frame_id " << frame_id
      //             << " should not start with a /. See 1.7 in "
      //                 "http://wiki.ros.org/tf2/Migration.";
      if(frame_id.size() > 1){
        frame_id_out = frame_id.substr(1);
      }else{
        LOG(ERROR)<< "The frame_id " << frame_id
                  << " should not start with a /. See 1.7 in "
                      "http://wiki.ros.org/tf2/Migration.";
      }
    } 
  }
  return frame_id_out;
}

void Run(const std::string& pbstream_filename, 
         const std::string& bag_filename,
         const std::string& urdf_filename,
         const std::string& tracking_frame) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
  
  ::ros::NodeHandle node_handle("");
  ::ros::Publisher pub_cloud = node_handle.advertise<sensor_msgs::PointCloud2>(
      "map_cloud", 1);
  ::ros::Publisher pub_traj = node_handle.advertise<nav_msgs::Path>(
      "map_trajectory", 1);

  LOG(INFO) << "Loading map from serialized data.";
  ::cartographer::mapping::proto::SerializedData proto;
  nav_msgs::Path map_traj;
  sensor_msgs::PointCloud2 point_cloud;


  const auto& pose_graph = deserializer.pose_graph();
  size_t trajectory_id = 0;
  
  const carto::mapping::proto::Trajectory& trajectory_proto =
      pose_graph.trajectory(trajectory_id);
  if (trajectory_proto.node_size() == 0) {
    return;
  }
  tf2_ros::Buffer tf_buffer;
  if (!urdf_filename.empty()) {
    ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
  }

  const carto::transform::TransformInterpolationBuffer 
    transform_interpolation_buffer(trajectory_proto);
  if(transform_interpolation_buffer.empty()){
    LOG(ERROR) << "Empty transforms!";
    return;
  }

  ros::Rate r(FLAGS_rate);
  rosbag::Bag bag;
  bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);
  Eigen::Vector3f tp;
  
  for(rosbag::MessageInstance const m : view){
    if(m.isType<sensor_msgs::PointCloud2>()){
      sensor_msgs::PointCloud2::Ptr s 
        = m.instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<RsPointXYZIRT> pcl_point_cloud;
      pcl::fromROSMsg(*s, pcl_point_cloud);
      auto msg = PreparePointCloud2Message(ros::Time::now(), 
        "map", pcl_point_cloud.points.size());
      
      ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
      for (const auto& point : pcl_point_cloud.points) {
        const carto::common::Time time = FromRos(s->header.stamp);
          + FromSeconds(point.timestamp - s->header.stamp.toSec());
        if (!transform_interpolation_buffer.Has(time)) {
          continue;
        }
        const carto::transform::Rigid3f tracking_to_map =
            transform_interpolation_buffer.Lookup(time).cast<float>();
    
        carto::transform::Rigid3f sensor_to_tracking =
        ToRigid3d(tf_buffer.lookupTransform(
            tracking_frame, CheckNoLeadingSlash(s->header.frame_id), 
            ToRos(time))).cast<float>();
        const carto::transform::Rigid3f sensor_to_map = 
          tracking_to_map * sensor_to_tracking;
        
        tp << point.x, point.y, point.z;
        tp = sensor_to_map * tp;
        
        stream.next(tp.x());
        stream.next(tp.y());
        stream.next(tp.z());
        stream.next(kPointCloudComponentFourMagic);
      }
    
      pub_cloud.publish(msg);
      ::ros::spinOnce();
      r.sleep();
    }
  }
  
  ::ros::shutdown();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  CHECK(!FLAGS_urdf_filename.empty()) << "-urdf_filename is missing.";
  CHECK(!FLAGS_tracking_frame.empty()) << "-tracking_frame is missing.";

  ::ros::init(argc, argv, "cartographer_pbstream_map_publisher");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  ::cartographer_ros::Run(FLAGS_pbstream_filename, 
    FLAGS_bag_filename, FLAGS_urdf_filename, FLAGS_tracking_frame);
}
