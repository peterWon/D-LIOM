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
#include "rosbag/view.h"
#include "cartographer_ros/time_conversion.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(bag_filename, "", "Stem of the rosbag file.");
DEFINE_string(ground_truth_filename, "", "Stem of the output file.");
DEFINE_string(method, "icp", "Using ndt to match.");
DEFINE_string(viewer, "", "Enable viewer debug.");

namespace cartographer_ros {
namespace {



void Run(const std::string& pbstream_filename, 
    const std::string& bag_filename, const std::string& grd_truth_filename){
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;

  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/rslidar_points");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  std::vector<int64_t> stamps_in_bag;
  for(rosbag::MessageInstance const m : view){
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<
      sensor_msgs::PointCloud2>();
    if (msg != NULL){
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud;
      pcl_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
      
      pcl::fromROSMsg(*msg, *pcl_point_cloud);
      std::vector<int> mapping;
      pcl::removeNaNFromPointCloud(*pcl_point_cloud, *pcl_point_cloud, mapping);
      clouds.emplace_back(pcl_point_cloud);
      stamps_in_bag.emplace_back(msg->header.stamp.toNSec());
    }
  }
  std::ofstream ofs(grd_truth_filename);
  if(!ofs.is_open()){
    LOG(INFO)<<"Open file failed!";
    return;
  }
  
  const auto& pose_graph = deserializer.pose_graph();
  
  if(pose_graph.trajectory_size() != 1){
    LOG(ERROR)<<"Not support!";
    return;
  }
  std::vector<cartographer::transform::Rigid3d> poses(
    pose_graph.trajectory(0).node_size()); 
  std::vector<int64_t> timestamps(pose_graph.trajectory(0).node_size()); 
  const auto& trajectory = pose_graph.trajectory(0);
  for(int j = 0; j < trajectory.node_size(); ++j){
    const auto& node = trajectory.node(j);
    // timestamps[j] = cartographer_ros::ToRos(cartographer::common::FromUniversal(
      // node.timestamp())).toNSec();
    // int64_t uts_timestamp = ::cartographer::common::ToUniversal(carto_time);
    int64_t ns_since_unix_epoch =
      (node.timestamp() -
      ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
          10000000ll) *
      100ll;
    timestamps[j] = ns_since_unix_epoch;
    poses[j] = cartographer::transform::ToRigid3(node.pose());
  }
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_pre_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_cur_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter_pre_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter_cur_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  voxel_filter_pre_.setLeafSize(0.2, 0.2, 0.2);
  voxel_filter_cur_.setLeafSize(0.2, 0.2, 0.2);

  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.0);
  ndt_.setMaximumIterations(35);

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  int n = 0;
  for(int i = 0; i < pose_graph.constraint_size(); ++i){
    const auto& constraint = pose_graph.constraint(i);
    if(constraint.tag() == 0) continue;//intra-constraint
    // if(n++ % 5 != 0) continue;
    size_t idx_from = constraint.node_id().node_index();
    size_t idx_to = 0;
    const auto& cur_pose = poses[idx_from];
    double min_dst = std::numeric_limits<double>::max();
    for(int k = 0; k < poses.size(); ++k){
      if(k == idx_from) continue;
      if(abs(timestamps[k] - timestamps[idx_from]) * 1e-9 < 60.) continue;//sec
      double diff_pos = (poses[idx_from].translation()-poses[k].translation()).norm();
      if(diff_pos < min_dst){
        min_dst = diff_pos;
        idx_to = k;
      }
    }
    LOG(INFO)<<idx_from<<","<<idx_to;
    double align_error= 99999.0;
    int64_t ts_to = timestamps[idx_to];
    int64_t ts_from = timestamps[idx_from];
    
    const auto it_to = std::lower_bound(
      stamps_in_bag.begin(), stamps_in_bag.end(), ts_to);
    const auto it_from = std::lower_bound(
      stamps_in_bag.begin(), stamps_in_bag.end(), ts_from);
    
    if(it_to == stamps_in_bag.end() || it_from == stamps_in_bag.end()){
      continue;
    }
    auto ig = (poses[idx_to].inverse() * poses[idx_from]).cast<float>();
    Eigen::Matrix4f gt = Eigen::Matrix4f::Identity();
    gt.block(0,0,3,3) = ig.rotation().toRotationMatrix();
    gt.block(0,3,3,1) = ig.translation();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f diff = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /*******************************************************/
    if(FLAGS_method == "icp" || FLAGS_method == "ICP"){
      static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setMaxCorrespondenceDistance(30);
      icp.setMaximumIterations(100);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setRANSACIterations(0);

      // Align clouds
      icp.setInputSource(clouds[it_from - stamps_in_bag.begin()]);
      icp.setInputTarget(clouds[it_to - stamps_in_bag.begin()]);
      
      icp.align(*output_cloud, gt);
      // icp.align(*output_cloud);
      trans = icp.getFinalTransformation();
      if(!icp.hasConverged()) continue;

      diff = gt.inverse() * trans;
      align_error = icp.getFitnessScore();
      // if(icp.getFitnessScore()>1) continue;
      LOG(INFO) <<icp.getFitnessScore()<<", " <<diff.block(0,3,3,1).norm(); 
    }else{
      ndt_.setInputSource(clouds[it_from - stamps_in_bag.begin()]);
      ndt_.setInputTarget(clouds[it_to - stamps_in_bag.begin()]);

      ndt_.align(*output_cloud, gt);
      
      auto trans = ndt_.getFinalTransformation();
      if(!ndt_.hasConverged()) continue;
      align_error = ndt_.getFitnessScore();
      diff = gt.inverse() * trans;
      LOG(INFO) <<ndt_.getFitnessScore()<<", " <<diff.block(0,3,3,1).norm();
    }
    if(!FLAGS_viewer.empty() && 
        (align_error < 1.0 || diff.block(0,3,3,1).norm() < 1.0)){
      pcl::transformPointCloud(
        *clouds[it_from - stamps_in_bag.begin()], *output_cloud, trans);
      boost::shared_ptr<pcl::visualization::PCLVisualizer>
      viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer_final->setBackgroundColor(0, 0, 0);
      //对目标点云着色（红色）并可视化
      const auto target_cloud = clouds[it_to - stamps_in_bag.begin()];
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
      viewer_final->addPointCloud<pcl::PointXYZ>(
        target_cloud, target_color, "target cloud");
      viewer_final->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
      //对转换后的目标点云着色（绿色）并可视化
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
      viewer_final->addPointCloud<pcl::PointXYZ>(
        output_cloud, output_color, "output cloud");
      viewer_final->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
      // 启动可视化
      viewer_final->addCoordinateSystem(1.0);
      viewer_final->initCameraParameters();
      //等待直到可视化窗口关闭。
      while (!viewer_final->wasStopped()){
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }
    }
    R = trans.block(0,0,3,3).cast<float>();
    t = trans.block(0,3,3,1).cast<float>();
    ofs << ts_from << " " << ts_to << " "<< align_error <<" "
      << R.row(0)[0] << " " << R.row(0)[1] <<" "<< R.row(0)[2] << " "<< t[0] << " "
      << R.row(1)[0] << " " << R.row(1)[1] <<" "<< R.row(1)[2] << " "<< t[1] << " "
      << R.row(2)[0] << " " << R.row(2)[1] <<" "<< R.row(2)[2] << " "<< t[2] << " "
      << diff.block(0,3,3,1).norm() << "\n"; 
  }
  
  ofs.close();
  bag.close();
  LOG(INFO)<<"Finish reading pose graph from pbstream.";
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  CHECK(!FLAGS_ground_truth_filename.empty()) << "-ground_truth_filename is missing.";
  CHECK(!FLAGS_method.empty()) << "-method is missing.";

  ::cartographer_ros::Run(
    FLAGS_pbstream_filename, FLAGS_bag_filename, FLAGS_ground_truth_filename);
}
