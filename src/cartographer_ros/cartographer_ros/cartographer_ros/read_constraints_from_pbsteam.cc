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
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer_ros/time_conversion.h"
DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(filename, "", "Stem of the output files.");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename, const std::string& filename) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;
  std::ofstream ofs(filename);
  if(!ofs.is_open()){
    LOG(INFO)<<"Open file failed!";
    return;
  }
  
  const auto& pose_graph = deserializer.pose_graph();
  for(int i = 0; i < pose_graph.constraint_size(); ++i){
    const auto& constraint = pose_graph.constraint(i);
    if(constraint.tag() == 0) continue;//intra-constraint
    ofs << "constraint "<< constraint.node_id().trajectory_id()<<" "
        << constraint.node_id().node_index()<<" "
        << constraint.submap_id().trajectory_id() << " " 
        << constraint.submap_id().submap_index() <<"\n";
  }
  for(int i= 0; i < pose_graph.trajectory_size(); ++i){
    const auto& trajectory = pose_graph.trajectory(i);
    for(int j = 0; j < trajectory.node_size(); ++j){
      const auto& node = trajectory.node(j);
      ofs << "node "<<trajectory.trajectory_id()<<" "<< node.node_index() 
          << " " << node.pose().translation().x()
          << " " << node.pose().translation().y()
          << " " << node.pose().translation().z()
          << " " << cartographer_ros::ToRos(cartographer::common::FromUniversal(
      node.timestamp())).toNSec() << "\n";
    }
    for(int k= 0; k < trajectory.submap_size(); ++k){
      const auto& submap = trajectory.submap(k);
      ofs << "submap "<<trajectory.trajectory_id()<<" "<< submap.submap_index() 
          << " " << submap.pose().translation().x()
          << " " << submap.pose().translation().y()
          << " " << submap.pose().translation().z() << "\n";
    }
  }
    
  ofs.close();
  LOG(INFO)<<"Finish reading pose graph from pbstream.";
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_filename.empty()) << "-filename is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_filename);
}
