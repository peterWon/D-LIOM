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
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
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

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to save a kitti trahectory from.");
DEFINE_string(calib_file_dir, "", "Dir of the calibration file.");
DEFINE_string(traj_filestem, "traj", "Stem of the output file.");

namespace cartographer_ros {
namespace {
using namespace std;
void Split(const string& s, vector<string>& tokens, const char& delim = ' ') {
  tokens.clear();
  size_t lastPos = s.find_first_not_of(delim, 0);
  size_t pos = s.find(delim, lastPos);
  while (lastPos != string::npos) {
      tokens.emplace_back(s.substr(lastPos, pos - lastPos));
      lastPos = s.find_first_not_of(delim, pos);
      pos = s.find(delim, lastPos);
  }
}

void ReadTransform(const std::string& calib_file, Eigen::Matrix4d& T){
  std::ifstream ifs(calib_file);
  if(!ifs.is_open()){
    LOG(ERROR)<<"Open calib file failed!";
    return;
  }
  std::string line;
  std::vector<std::string> substrs = {};
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  while(getline(ifs, line)){
    if(line.empty()) continue;
    if(line.at(0) == 'R'){
      Split(line, substrs);
      if(substrs.size() != 10){
        LOG(ERROR)<<"R has invalid size, check it!";
        return;
      }
      for(int i = 1; i < 10; i++){
        istringstream os(substrs[i]);
        double d;
        os >> d;
        R.row((i-1)/3)[(i-1)%3] = d;
      }
    }else if(line.at(0) == 'T'){
      Split(line, substrs);
      if(substrs.size() != 4){
        LOG(ERROR)<<"T has invalid size, check it!";
        return;
      }
      for(int i = 1; i < 4; i++){
        istringstream os(substrs[i]);
        double d;
        os >> d;
        t[i-1] = d;
      }
    }
  }
  ifs.close();
  
  T.block(0,0,3,3) = R;
  T.block(0,3,3,1) = t;
  T.block(3,0,1,4) << 0,0,0,1;
}

void Run(const std::string& pbstream_filename) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  Eigen::Matrix4d H, H_init, T, Pose_imu;
  H = Eigen::Matrix4d::Identity();
  T = Eigen::Matrix4d::Identity();
  H_init = Eigen::Matrix4d::Identity();
  Pose_imu = Eigen::Matrix4d::Identity();
  double length = 0.;
  double max_linear_speed = 0.;
  double max_angular_speed = 0.;
  double x_min = 99999999999.;
  double x_max = -99999999999.;
  double y_min = 99999999999.;
  double y_max = -99999999999.;
  double z_min = 99999999999.;
  double z_max = -99999999999.;

  Eigen::Matrix3d last_R;
  Eigen::Vector3d last_t;
  cartographer::common::Time last_stamp;
  bool inited = false;
  ::cartographer::mapping::proto::SerializedData proto;
  const auto& pose_graph = deserializer.pose_graph();
  
  for(const auto&traj: pose_graph.trajectory()){
    for(const auto& node: traj.node()){
      const ::cartographer::transform::Rigid3d global_pose =
            ::cartographer::transform::ToRigid3(node.pose());
      int64 time = node.timestamp();
      auto carto_time = cartographer::common::FromUniversal(time);
      Eigen::Matrix3d R = global_pose.rotation().toRotationMatrix();
      Eigen::Vector3d t = global_pose.translation();
      
      if(!inited){
        inited = true;
        last_R = R;
        last_t = t;
        last_stamp = carto_time;
        continue;
      }

      Eigen::Matrix3d relative_R = last_R.inverse() * R;
      Eigen::Vector3d relative_t = last_R.inverse() * (t - last_t);
      double relative_time = cartographer::common::ToSeconds(carto_time - last_stamp);
      Eigen::AngleAxisd rotation_vector;
      rotation_vector.fromRotationMatrix(relative_R);
      
      max_linear_speed = std::max(max_linear_speed, 
        (relative_t / relative_time).norm());
      max_angular_speed = std::max(max_angular_speed, std::abs(
        rotation_vector.angle() / relative_time));
      
      x_max = std::max(x_max, t[0]);
      x_min = std::min(x_min, t[0]);
      y_max = std::max(y_max, t[1]);
      y_min = std::min(y_min, t[1]);
      z_max = std::max(z_max, t[2]);
      z_min = std::min(z_min, t[2]);
      
      length += sqrt(relative_t.norm());
      last_R = R;
      last_t = t;
      last_stamp = carto_time;
    }
  }

  LOG(INFO) << "Length: "<<length; 
  LOG(INFO) << "max_linear_speed: "<<max_linear_speed; 
  LOG(INFO) << "max_angular_speed: "<<max_angular_speed; 
  
  LOG(INFO) << "Z variation: "<<z_max - z_min; 
  LOG(INFO) << "Coverage: "<<(x_max-x_min)*(y_max-y_min); 
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename);
}
