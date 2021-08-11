#include "cartographer/mapping/internal/3d/gravity_factor/gravity_estimator.h"


namespace cartographer {
namespace mapping {
MatrixXd GravityEstimator::TangentBasis(Vector3d &g0) {
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

bool GravityEstimator::ApproximateGravity(
  const std::deque<Rigid3dWithPreintegrator> &all_laser_transforms,
  const transform::Rigid3d &transform_lb,
  const std::deque<Vector3d> &Vs,
  const double g_norm,
  Vector3d &g) {

  size_t window_size = all_laser_transforms.size();
  int n_state = window_size * 3 + 3; //3g + n*3v
  VectorXd x;
  // Laser in Base(IMU)   
  Vector3d tlb = transform_lb.translation();
  Matrix3d rlb = transform_lb.rotation().toRotationMatrix();

  MatrixXd A{3, 3};
  A.setZero();
  VectorXd b{3};
  b.setZero();
  
  if (window_size < 3) {
    LOG(WARNING) << ">>>>>>>Gravity estimate window size is not enough <<<<<<<";
    return false;
  }
  for (size_t i = 0; i < window_size - 1; ++i){
    const Rigid3dWithPreintegrator* frame_i = &all_laser_transforms[i];
    const Rigid3dWithPreintegrator* frame_j = &all_laser_transforms[i + 1];
    
    if(!frame_j->pre_integration) continue;

    MatrixXd tmp_A(6, 3);
    tmp_A.setZero();
    VectorXd tmp_b(6);
    tmp_b.setZero();
    
    double dt = frame_j->pre_integration->deltaTij();
    Matrix3d frame_i_R = frame_i->transform.rotation().toRotationMatrix();
    Matrix3d frame_j_R = frame_j->transform.rotation().toRotationMatrix();
    Vector3d frame_i_T = frame_i->transform.translation();
    Vector3d frame_j_T = frame_j->transform.translation();
  
    tmp_A.block<3, 3>(0, 0) = frame_i_R.transpose() 
      * dt * dt / 2 * Matrix3d::Identity();
    
    tmp_b.block<3, 1>(0, 0) = 
      frame_j->pre_integration->deltaPij() 
      + frame_i_R.transpose() * frame_j_R * tlb - tlb 
      - frame_i_R.transpose() * (frame_j_T - frame_i_T)
      + dt * Matrix3d::Identity() * Vs[i];
    
    tmp_A.block<3, 3>(3, 0) = frame_i_R.transpose() * dt * Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j->pre_integration->deltaVij() 
      + Matrix3d::Identity() * Vs[i]
      - frame_i_R.transpose() * frame_j_R * Vs[i+1];
   
    Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<3, 3>(0, 0) += r_A;
    b.segment<3>(0) += r_b;
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);

  g = x.segment<3>(0);
  // LOG(INFO) << "g_norm_diff before refining: " <<fabs(g.norm() - g_norm);
  return fabs(g.norm() - g_norm) < 0.5;
}

void GravityEstimator::RefineGravity(
  const std::deque<Rigid3dWithPreintegrator>& all_laser_transforms,
  const transform::Rigid3d &transform_lb,
  const std::deque<Vector3d> &Vs,
  const double k_g_norm,
  Vector3d &g_approx) {

  Vector3d g0 = g_approx.normalized() * k_g_norm;

  // Laser in Base(IMU)   
  Vector3d tlb = transform_lb.translation();
  Matrix3d rlb = transform_lb.rotation().toRotationMatrix();
  
  Vector3d lx, ly;
  VectorXd x;
  size_t window_size = all_laser_transforms.size();

  MatrixXd A{2, 2};
  A.setZero();
  VectorXd b{2};
  b.setZero();
  
  for(int k = 0; k < 4; k++) {
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    for (size_t i = 0; i < window_size - 1; ++i){
      const Rigid3dWithPreintegrator* frame_i = &all_laser_transforms[i];
      const Rigid3dWithPreintegrator* frame_j = &all_laser_transforms[i + 1];
      if(!frame_j->pre_integration) continue;
      double dt = frame_j->pre_integration->deltaTij();
      Matrix3d frame_i_R = frame_i->transform.rotation().toRotationMatrix();
      Matrix3d frame_j_R = frame_j->transform.rotation().toRotationMatrix();
      Vector3d frame_i_T = frame_i->transform.translation();
      Vector3d frame_j_T = frame_j->transform.translation();

      MatrixXd tmp_A(6, 2);
      tmp_A.setZero();
      VectorXd tmp_b(6);
      tmp_b.setZero();

      tmp_A.block<3, 2>(0, 0) = frame_i_R.transpose() 
          * dt * dt / 2 * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(0, 0) = frame_j->pre_integration->deltaPij() 
          + frame_i_R.transpose() * frame_j_R * tlb - tlb 
          - frame_i_R.transpose() * dt * dt / 2 * g0
          - frame_i_R.transpose() * (frame_j_T - frame_i_T)
          + dt * Matrix3d::Identity() * Vs[i];

      tmp_A.block<3, 2>(3, 0) = frame_i_R.transpose() * dt 
          * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j->pre_integration->deltaVij() 
          - frame_i_R.transpose() * dt * Matrix3d::Identity() * g0
          + Matrix3d::Identity() * Vs[i]
          - frame_i_R.transpose() * frame_j_R * Vs[i+1];

      Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<2, 2>(0, 0) += r_A;
      b.segment<2>(0) += r_b;
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    VectorXd dg = x.segment<2>(0);
    g0 = (g0 + lxly * dg).normalized() * k_g_norm;
  }   
  g_approx = g0;
  // for(int i = 0; i < window_size; ++i){
  //   Vs[i] = x.segment<3>(3 * i);
  // }
}

bool GravityEstimator::Estimate(
    const std::deque<Rigid3dWithPreintegrator>& all_laser_transforms,
    const transform::Rigid3d &transform_lb,
    const std::deque<Vector3d> &Vs,
    double g_norm,
    Vector3d &g_approx){

  if(!ApproximateGravity(all_laser_transforms, transform_lb, Vs, g_norm, g_approx)){
    return false;
  }

  RefineGravity(all_laser_transforms, transform_lb, Vs, g_norm, g_approx);
  
  double g_norm_diff = fabs(g_approx.norm() - g_norm);
  // LOG(INFO) << "g_norm_diff refined: " <<g_norm_diff;
  return g_norm_diff < 0.2; 
};

}
}
