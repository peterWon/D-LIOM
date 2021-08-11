/*
 * Copyright 2017 The Cartographer Authors
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
#include "cartographer/mapping/proto/local_slam_range_data.pb.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros/urdf_reader.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"

DEFINE_string(pose_graph_pb_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(range_data_pb_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(urdf_filename, "",
              "URDF of a robot.");
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

void ReadNodesInfo(std::map<int64, ::cartographer::transform::Rigid3d>& node_table){
  CHECK(!FLAGS_pose_graph_pb_filename.empty());
  ::cartographer::io::ProtoStreamReader reader(FLAGS_pose_graph_pb_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
  node_table.clear();
  const auto& pose_graph = deserializer.pose_graph();

  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_node()) {
      const auto& node = proto.node();
      const ::cartographer::mapping::NodeId id{
          node.node_id().trajectory_id(),
          node.node_id().node_index()};
      const ::cartographer::transform::Rigid3d global_node_pose =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .node(id.node_index)
                  .pose());
      node_table.insert({node.node_data().timestamp(), global_node_pose});
    }
  }
  CHECK(reader.eof());
}

void Run(const std::string& pbstream_filename) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::ros::NodeHandle node_handle("");
  ::ros::Publisher pub_cloud = node_handle.advertise<sensor_msgs::PointCloud2>(
      "map_cloud", 1);
  ::ros::Publisher pub_traj = node_handle.advertise<nav_msgs::Path>(
      "map_trajectory", 1);
  tf2_ros::Buffer tf_buffer;
  if (!FLAGS_urdf_filename.empty()) {
    ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  }
  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<int64, ::cartographer::transform::Rigid3d> node_table;
  ReadNodesInfo(node_table);

  ros::Rate r(FLAGS_rate);
  Eigen::Vector3f lp, tp, gp;
  nav_msgs::Path traj_viz;
  
  traj_viz.header.frame_id = "map";
  ::geometry_msgs::PoseStamped stp_pose;
  
  cartographer::mapping::proto::NodeRangeData node_range_data;
  while (reader.ReadProto(&node_range_data)) {
    int64 timestamp = node_range_data.timestamp();
    const auto& local_pose = ::cartographer::transform::ToRigid3(
      node_range_data.local_pose()).cast<float>();
    if(node_table.find(timestamp) == node_table.end()) continue;
    const auto& global_pose = node_table[timestamp].cast<float>();
    
    
    traj_viz.header.stamp = ::ros::Time::now();
    stp_pose.header = traj_viz.header;
    const ::geometry_msgs::Pose node_pose =
          ToGeometryMsgPose(global_pose.cast<double>());
    stp_pose.pose = node_pose;
    traj_viz.poses.push_back(stp_pose);
    pub_traj.publish(traj_viz);

    int points_num = node_range_data.range_data_in_local().returns_size();
    auto msg = PreparePointCloud2Message(ros::Time::now(), "map", points_num);
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for(int k = 0; k < points_num; ++k){
      const auto& pt  = node_range_data.range_data_in_local().returns(k);
      lp << pt.x(), pt.y(), pt.z(); //pt in local
      tp = local_pose.inverse() * lp; //pt in tracking
      gp = global_pose * tp; //pt in global
      stream.next(gp.x());
      stream.next(gp.y());
      stream.next(gp.z());
      stream.next(kPointCloudComponentFourMagic);
    }
    
    pub_cloud.publish(msg);
    ::ros::spinOnce();
    r.sleep();
  }
  CHECK(reader.eof());
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pose_graph_pb_filename.empty()) << "-pose_graph_pb_filename is missing.";
  CHECK(!FLAGS_range_data_pb_filename.empty()) << "-range_data_pb_filename is missing.";
  CHECK(!FLAGS_urdf_filename.empty()) << "-urdf_filename is missing.";
  ::ros::init(argc, argv, "cartographer_pbstream_3d_map_publisher");
  ::ros::start();
  ::cartographer_ros::Run(FLAGS_range_data_pb_filename);
}
