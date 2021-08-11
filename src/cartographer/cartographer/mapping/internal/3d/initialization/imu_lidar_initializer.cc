/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 5/20/18.
//

#include "cartographer/mapping/internal/3d/initialization/imu_lidar_initializer.h"
namespace cartographer {
namespace mapping {
using namespace Eigen;

MatrixXd TangentBasis(Vector3d &g0) {
  Vector3d b, c;
  Vector3d a = g0.normalized();
  Vector3d tmp(0, 0, 1);
  if (a == tmp)
    tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

bool ApproximateGravity(
    std::deque<Rigid3dWithVINSPreintegrator> &all_laser_transforms,
    const transform::Rigid3d& transform_lb,
    const double g_norm,
    Vector3d& g) {
  size_t window_size = all_laser_transforms.size();
  int n_state = window_size * 3 + 3; //3g + n*3v
  VectorXd x;
  // Laser in Base(IMU)   
  const Vector3d& tlb = transform_lb.translation();
  const Matrix3d& rlb = transform_lb.rotation().toRotationMatrix();

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();
  
  if (window_size < 7) {
    LOG(WARNING) << ">>>>>>>Initialization window size is not enough <<<<<<<";
    return false;
  }
  for (size_t i = 0; i < window_size - 1; ++i){
    const Rigid3dWithVINSPreintegrator* frame_i = &all_laser_transforms[i];
    const Rigid3dWithVINSPreintegrator* frame_j = &all_laser_transforms[i + 1];
    if(!frame_j->pre_integration) continue;
    
    MatrixXd tmp_A(6, 9);
    tmp_A.setZero();
    VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->pre_integration->deltaTij();
    Matrix3d frame_i_R = frame_i->transform.rotation().toRotationMatrix();
    Matrix3d frame_j_R = frame_j->transform.rotation().toRotationMatrix();
    Vector3d frame_i_T = frame_i->transform.translation();
    Vector3d frame_j_T = frame_j->transform.translation();
  
    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) = frame_i_R.transpose() * dt * dt 
      / 2 * Matrix3d::Identity();
    
    tmp_b.block<3, 1>(0, 0) = frame_j->pre_integration->deltaPij() 
      + frame_i_R.transpose() * frame_j_R * tlb - tlb 
      - frame_i_R.transpose() * (frame_j_T - frame_i_T);
    
    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i_R.transpose() * frame_j_R;
    tmp_A.block<3, 3>(3, 6) = frame_i_R.transpose() * dt * Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j->pre_integration->deltaVij();
   
    Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
    b.tail<3>() += r_b.tail<3>();

    A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
    A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);

  g = x.segment<3>(n_state - 3);
  LOG(INFO) << "g_norm_diff before refining: " <<fabs(g.norm() - g_norm);
  return fabs(g.norm() - g_norm) < 1.0;
}

void RefineGravity(
    std::deque<Rigid3dWithVINSPreintegrator> &all_laser_transforms,
    const transform::Rigid3d &transform_lb,
    const double k_g_norm,
    std::deque<Vector3d> &Bgs,
    std::deque<Vector3d> &Vs,
    Vector3d &g_approx) {

  Vector3d g0 = g_approx.normalized() * k_g_norm;

  // Laser in Base(IMU)   
  const Vector3d& tlb = transform_lb.translation();
  const Matrix3d& rlb = transform_lb.rotation().toRotationMatrix();
  
  Vector3d lx, ly;
  VectorXd x;
  size_t window_size = all_laser_transforms.size();
  int n_state = window_size * 3 + 2;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();
  
  for(int k = 0; k < 4; k++) {
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    for (size_t i = 0; i < window_size - 1; ++i){
      const Rigid3dWithVINSPreintegrator* frame_i = &all_laser_transforms[i];
      const Rigid3dWithVINSPreintegrator* frame_j = &all_laser_transforms[i + 1];
      if(!frame_j->pre_integration) continue;

      double dt = frame_j->pre_integration->deltaTij();
      Matrix3d frame_i_R = frame_i->transform.rotation().toRotationMatrix();
      Matrix3d frame_j_R = frame_j->transform.rotation().toRotationMatrix();
      Vector3d frame_i_T = frame_i->transform.translation();
      Vector3d frame_j_T = frame_j->transform.translation();

      MatrixXd tmp_A(6, 8);
      tmp_A.setZero();
      VectorXd tmp_b(6);
      tmp_b.setZero();

      tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i_R.transpose() 
          * dt * dt / 2 * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(0, 0) = frame_j->pre_integration->deltaPij() 
          + frame_i_R.transpose() * frame_j_R * tlb - tlb 
          - frame_i_R.transpose() * dt * dt / 2 * g0
          - frame_i_R.transpose() * (frame_j_T - frame_i_T);

      tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) = frame_i_R.transpose() * frame_j_R;
      tmp_A.block<3, 2>(3, 6) = frame_i_R.transpose() * dt 
          * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j->pre_integration->deltaVij() 
          - frame_i_R.transpose() * dt * Matrix3d::Identity() * g0;

      Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<2, 2>() += r_A.bottomRightCorner<2, 2>();
      b.tail<2>() += r_b.tail<2>();

      A.block<6, 2>(i * 3, n_state - 2) += r_A.topRightCorner<6, 2>();
      A.block<2, 6>(n_state - 2, i * 3) += r_A.bottomLeftCorner<2, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    VectorXd dg = x.segment<2>(n_state - 2);
    g0 = (g0 + lxly * dg).normalized() * k_g_norm;
  }   
  g_approx = g0;
  for(int i = 0; i < window_size; ++i){
    Vs[i] = x.segment<3>(3 * i);
  }
}

bool Initializer::Initialization(
  std::deque<Rigid3dWithVINSPreintegrator> &all_laser_transforms,
  const transform::Rigid3d& transform_lb,
  const double g_norm,
  std::deque<Vector3d> &Vs,
  std::deque<Vector3d> &Bgs,
  Vector3d &g) {
  if(!ApproximateGravity(all_laser_transforms, transform_lb, g_norm, g)){
    return false;
  }
  
  RefineGravity(all_laser_transforms, transform_lb, g_norm, Bgs, Vs, g);
  
  double g_norm_diff = fabs(g.norm() - g_norm);
  LOG(INFO) << "g_norm_diff refined: " <<g_norm_diff;
  return g_norm_diff < 0.2; 
};

}
}