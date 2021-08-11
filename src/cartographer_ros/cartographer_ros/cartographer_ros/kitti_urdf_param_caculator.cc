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
#include <Eigen/Core>
#include <Eigen/Dense>
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(calib_file_dir, "", "Dir of the calibration file.");

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

void Run(const std::string& calib_file_dir) {  
  std::string calib_imu_to_velo = calib_file_dir+"/calib_imu_to_velo.txt";
  // std::string calib_velo_to_cam = calib_file_dir+"/calib_velo_to_cam.txt";
  Eigen::Matrix4d T_imu_to_velo;
  ReadTransform(calib_imu_to_velo, T_imu_to_velo);
  Eigen::Matrix4d T_velo_to_imu = T_imu_to_velo.inverse();
  Eigen::Vector3d rpy = T_velo_to_imu.block<3,3>(0,0).eulerAngles(0,1,2);
  Eigen::Vector3d t = T_velo_to_imu.block<3,1>(0,3);

  LOG(INFO) << "Translation from velodyne to imu is: "; 
  LOG(INFO) << t[0] << "," << t[1] << "," <<t[2]; 
  LOG(INFO) << "Rotation(roll, pitch, yaw) from velodyne to imu is: "; 
  LOG(INFO) << rpy[0] << "," << rpy[1] << "," << rpy[2]; 
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  using namespace Eigen;
  
  /*newer college*/
  // Eigen::Quaterniond q(0,0,0,-1);
  // Eigen::Matrix3d R(q);
  // Eigen::Vector3d rpy = R.eulerAngles(0,1,2);
  // LOG(INFO) << "Rotation(roll, pitch, yaw) from velodyne to imu is: "; 
  // LOG(INFO) << rpy[0] << "," << rpy[1] << "," << rpy[2]; 
  
  // /*KAIST*/
  // Eigen::Matrix4d T_i2b, T_v2b;
  // T_i2b.setIdentity();
  // T_v2b.setIdentity();
  // // Eigen::Vector3d eulerAngle(yaw,pitch,roll);
  // Eigen::Vector3d eulerAngle(0,0,0);
  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Vector3d::UnitZ())); 
  // R = yawAngle * pitchAngle * rollAngle;
  // Eigen::Vector3d t;
  // t << -0.07, 0, 1.7;
  // T_v2b.block(0,0,3,3)=R;
  // T_v2b.block(0,3,3,1)=t;

  // Eigen::Matrix4d T_b2v = T_v2b.inverse();
  // Eigen::Vector3d rpy_b2v = T_b2v.block<3,3>(0,0).eulerAngles(0,1,2);
  // Eigen::Vector3d t_b2v = T_b2v.block<3,1>(0,3);
  // LOG(INFO)<<t_b2v[0]<<","<<t_b2v[1]<<","<<t_b2v[2];
  // LOG(INFO)<<rpy_b2v[0]<<","<<rpy_b2v[1]<<","<<rpy_b2v[2];
  
  //LIO-SAM campus, lidar to imu
  // Eigen::Matrix3d R;
  // R << -1, 0, 0,  0, 1, 0, 0, 0, -1;
  // Eigen::Vector3d rpy_campus = R.eulerAngles(0,1,2);
  // LOG(INFO)<<rpy_campus[0]<<","<<rpy_campus[1]<<","<<rpy_campus[2];

  // VIRAL
  // Eigen::Matrix3d R;
  // Eigen::Matrix4d T, T_inv;
  // Eigen::Vector3d t;
  // T << -1.0,  0.0,  0.0, -0.55,  0.0, 0.0, 1.0, 0.03,  0.0, 1.0,  0.0, 0.05, 0,0,0,1;
  // t << -0.55, 0.03, 0.05;
  // T.block(0,0,3,3) = R;
  // T.block(0,3,3,1) = t;
  // T.block(3,0,1,4) << 0,0,0,1;
  // T_inv = T.inverse();
  // Eigen::Vector3d rpy_l2i = T_inv.block<3, 3>(0,0).eulerAngles(0,1,2);
  // Eigen::Vector3d t_l2i = T_inv.block<3,1>(0,3);
  // LOG(INFO)<<T_inv;
  // LOG(INFO)<<t_l2i[0]<<","<<t_l2i[1]<<","<<t_l2i[2];
  // LOG(INFO)<<rpy_l2i[0]<<","<<rpy_l2i[1]<<","<<rpy_l2i[2];

  // CHECK(!FLAGS_calib_file_dir.empty()) << "-calib_file_dir is missing.";

  // ::cartographer_ros::Run(FLAGS_calib_file_dir);

  Eigen::Matrix3d R;
  R<<0,1,0,-1,0,0,0,0,1;
  Eigen::Vector3d rpy_l2i = R.eulerAngles(0,1,2);
  LOG(INFO)<<rpy_l2i[0]<<","<<rpy_l2i[1]<<","<<rpy_l2i[2];
}
