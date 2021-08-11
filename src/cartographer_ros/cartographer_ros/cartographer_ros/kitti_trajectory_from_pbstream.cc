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

void Run(const std::string& pbstream_filename, 
         const std::string& calib_file_dir,
         const std::string& traj_filestem) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
  
  std::string calib_imu_to_velo = calib_file_dir+"/calib_imu_to_velo.txt";
  std::string calib_velo_to_cam = calib_file_dir+"/calib_velo_to_cam.txt";
  Eigen::Matrix4d T_imu_to_velo, T_velo_to_cam;
  ReadTransform(calib_imu_to_velo, T_imu_to_velo);
  ReadTransform(calib_velo_to_cam, T_velo_to_cam);

  Eigen::Matrix4d H, H_init, T, Pose_imu;
  H = Eigen::Matrix4d::Identity();
  T = Eigen::Matrix4d::Identity();
  H_init = Eigen::Matrix4d::Identity();
  Pose_imu = Eigen::Matrix4d::Identity();
  
  // Eigen::AngleAxisd rotation_vector(-3.5 * M_PI / 180.0 , Eigen::Vector3d(0,0,1));
  // Eigen::Matrix3d R_init = rotation_vector.matrix();
  // H_init.block(0,0,3,3) = R_init;

  bool init_flag = true;
  T = T_velo_to_cam * T_imu_to_velo;//
  //真值是以相机坐标系为基准的，x:水平向右; y:竖直向下；ｚ:水平向前
  // T << 0,0,1,0,
  //      -1,0,0,0,
  //      0,-1,0,0,
  //      0,0,0,1;
  /********************************************/
  std::ofstream ofs(traj_filestem+".txt");
  if(!ofs.is_open()) {
    LOG(ERROR)<<"Open kitti file failed!";  
    return;
  }
  ofs.setf(std::ios::scientific, std::ios::floatfield);
  ofs.precision(6);
  LOG(INFO) << "Loading trajectory nodes from serialized data.";

  ::cartographer::mapping::proto::SerializedData proto;
  const auto& pose_graph = deserializer.pose_graph();
  for(const auto&traj: pose_graph.trajectory()){
    for(const auto& node: traj.node()){
      const ::cartographer::transform::Rigid3d global_pose =
            ::cartographer::transform::ToRigid3(node.pose());
      Eigen::Matrix3d R = global_pose.rotation().toRotationMatrix();
      Eigen::Vector3d t = global_pose.translation();
      // if (init_flag == true){
      //   H_init.block(0,0,3,3) = R;
      //   H_init.block(0,3,3,1) = t;
      //   H_init.block(3,0,1,4) << 0,0,0,1;
      //   init_flag = false;
      // }
      Pose_imu.block(0,0,3,3) = R;
      Pose_imu.block(0,3,3,1) = t;
      Pose_imu.block(3,0,1,4) << 0,0,0,1;
      H = T * H_init.inverse() * Pose_imu;
      for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 4; ++j){
          if(i==2 && j==3)
            ofs << H.row(i)[j] << "\n" ;	
          else
            ofs << H.row(i)[j] << " " ;
        }
      }
    }
  }

  // CHECK(reader.eof());
  ofs.close();
  LOG(INFO) << "Exported trajectory poses to kitti format."; 
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_calib_file_dir.empty()) << "-calib_file_dir is missing.";
  CHECK(!FLAGS_traj_filestem.empty()) << "-traj_filestem is missing.";

  ::cartographer_ros::Run(
    FLAGS_pbstream_filename, FLAGS_calib_file_dir, FLAGS_traj_filestem);
}
