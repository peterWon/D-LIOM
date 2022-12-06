//
// Created by wz on 05/29/2021. Adapted from VINS-Mono.
//
#ifndef _CARTOGRAPHER_MAPPING_INTERNAL_3D_INTEGRATION_BASE_H_
#define _CARTOGRAPHER_MAPPING_INTERNAL_3D_INTEGRATION_BASE_H_

#include <eigen3/Eigen/Dense>
#include <vector>

namespace cartographer {
namespace mapping {

using namespace std;
using namespace Eigen;

enum StateOrder { O_R = 0, O_P = 3, O_V = 6, O_BA = 9, O_BG = 12 };
enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };
struct IMUNoise{
   double ACC_N;
   double GYR_N;
   double ACC_W;
   double GYR_W;
};
class IntegrationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IntegrationBase() = delete;
  IntegrationBase(
      const Eigen::Vector3d &_linearized_ba, 
      const Eigen::Vector3d &_linearized_bg,
      const IMUNoise& n_imu):
    linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
    jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, 
    covariance{Eigen::Matrix<double, 15, 15>::Zero()},
    sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, 
    delta_q{Eigen::Quaterniond::Identity()}, 
    delta_v{Eigen::Vector3d::Zero()} {
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) =  (n_imu.ACC_N * n_imu.ACC_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) =  (n_imu.GYR_N * n_imu.GYR_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) =  (n_imu.ACC_N * n_imu.ACC_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) =  (n_imu.GYR_N * n_imu.GYR_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) =  (n_imu.ACC_W * n_imu.ACC_W) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) =  (n_imu.GYR_W * n_imu.GYR_W) 
      * Eigen::Matrix3d::Identity();
  }

  IntegrationBase(const IntegrationBase& other){
    sum_dt = other.sum_dt;
    linearized_ba = other.linearized_ba;
    linearized_bg = other.linearized_bg;
    jacobian = other.jacobian;
    covariance = other.covariance;
    delta_p = other.delta_p;
    delta_q = other.delta_q;
    delta_v = other.delta_v;
    noise = other.noise;
    dt_buf = other.dt_buf;
    acc_buf = other.acc_buf;
    gyr_buf = other.gyr_buf;
  }
  
  void resetIntegration(
      const Eigen::Vector3d &_linearized_ba, 
      const Eigen::Vector3d &_linearized_bg,
      const IMUNoise& n_imu){
    dt = -1.;
    sum_dt = 0.0; 
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;

    jacobian = Eigen::Matrix<double, 15, 15>::Identity(); 
    covariance = Eigen::Matrix<double, 15, 15>::Zero();
    
    delta_p = Eigen::Vector3d::Zero(); 
    delta_q = Eigen::Quaterniond::Identity(); 
    delta_v = Eigen::Vector3d::Zero();
    
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) =  (n_imu.ACC_N * n_imu.ACC_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) =  (n_imu.GYR_N * n_imu.GYR_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) =  (n_imu.ACC_N * n_imu.ACC_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) =  (n_imu.GYR_N * n_imu.GYR_N) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) =  (n_imu.ACC_W * n_imu.ACC_W) 
      * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) =  (n_imu.GYR_W * n_imu.GYR_W) 
      * Eigen::Matrix3d::Identity();
    
    dt_buf.clear();
    acc_buf.clear();
    gyr_buf.clear();
  }

  double deltaTij(){ return sum_dt;}
  Eigen::Vector3d deltaPij(){ return delta_p;}
  Eigen::Vector3d deltaVij(){ return delta_v;}
  Eigen::Matrix3d deltaRij(){ return delta_q.toRotationMatrix();}
  Eigen::Quaterniond deltaQij(){ return delta_q;}
  
  void push_back(
      double _dt, const Eigen::Vector3d &_acc, const Eigen::Vector3d &_gyr) {
    if(dt < 0.){
      dt = 1e-6;
      acc_0 = _acc;
      gyr_0 = _gyr;
      linearized_acc = acc_0;
      linearized_gyr = gyr_0;
      return;
    }
    dt_buf.push_back(_dt);
    acc_buf.push_back(_acc);
    gyr_buf.push_back(_gyr);
    propagate(_dt, _acc, _gyr);
  }

  template <typename Derived>
  Eigen::Quaternion<typename Derived::Scalar> deltaQ(
      const Eigen::MatrixBase<Derived> &theta){
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  void repropagate(const Eigen::Vector3d &_linearized_ba, 
                    const Eigen::Vector3d &_linearized_bg) {
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
      propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
  }

  void midPointIntegration(double _dt, const Eigen::Vector3d &_acc_0, 
      const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1, 
      const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p, 
      const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
      const Eigen::Vector3d &linearized_ba, 
      const Eigen::Vector3d &linearized_bg,
      Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v, Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian) {
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Quaterniond(
        1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;         

    if(update_jacobian){
      Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
      Vector3d a_0_x = _acc_0 - linearized_ba;
      Vector3d a_1_x = _acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x<<0, -w_x(2), w_x(1),
          w_x(2), 0, -w_x(0),
          -w_x(1), w_x(0), 0;
      R_a_0_x<<0, -a_0_x(2), a_0_x(1),
          a_0_x(2), 0, -a_0_x(0),
          -a_0_x(1), a_0_x(0), 0;
      R_a_1_x<<0, -a_1_x(2), a_1_x(1),
          a_1_x(2), 0, -a_1_x(0),
          -a_1_x(1), a_1_x(0), 0;

      MatrixXd F = MatrixXd::Zero(15, 15);
      F.block<3, 3>(0, 0) = Matrix3d::Identity();
      F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() 
        * R_a_0_x * _dt * _dt + -0.25 * result_delta_q.toRotationMatrix() 
        * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
      F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
      F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() 
        + result_delta_q.toRotationMatrix()) * _dt * _dt;
      F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() 
        * R_a_1_x * _dt * _dt * -_dt;
      F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
      F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
      F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() 
        * R_a_0_x * _dt -0.5 * result_delta_q.toRotationMatrix() 
        * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
      F.block<3, 3>(6, 6) = Matrix3d::Identity();
      F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() 
        + result_delta_q.toRotationMatrix()) * _dt;
      F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() 
        * R_a_1_x * _dt * -_dt;
      F.block<3, 3>(9, 9) = Matrix3d::Identity();
      F.block<3, 3>(12, 12) = Matrix3d::Identity();

      MatrixXd V = MatrixXd::Zero(15,18);
      V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
      V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() 
        * R_a_1_x  * _dt * _dt * 0.5 * _dt;
      V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() 
        * _dt * _dt;
      V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
      V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
      V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() 
        * R_a_1_x  * _dt * 0.5 * _dt;
      V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() 
        * _dt;
      V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
      V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

      jacobian = F * jacobian;
      covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
  }

  void propagate(double _dt, const Eigen::Vector3d &_acc_1, 
                 const Eigen::Vector3d &_gyr_1) {
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, 
      delta_p, delta_q, delta_v, linearized_ba, linearized_bg,
      result_delta_p, result_delta_q, result_delta_v,
      result_linearized_ba, result_linearized_bg, 1);
    //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
    //                    linearized_ba, linearized_bg);
    // 积分状态更新
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;  
  }

  /* Eigen::Matrix<double, 15, 1> evaluate(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
    const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, 
    const Eigen::Vector3d &Bgi,const Eigen::Vector3d &Pj, 
    const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, 
    const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    // midPointIntegration中是将bias认为恒定来积分的，因此，在这里加上对bias的线性修正
    Eigen::Quaterniond corrected_delta_q = delta_q *
      deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt 
      * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() 
      * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) 
      - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
  } */

  double dt = -1.0;
  Eigen::Vector3d acc_0, gyr_0;
  Eigen::Vector3d acc_1, gyr_1;

  Eigen::Vector3d linearized_acc, linearized_gyr;
  Eigen::Vector3d linearized_ba, linearized_bg;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 15, 15> step_jacobian;
  Eigen::Matrix<double, 15, 18> step_V;
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> acc_buf;
  std::vector<Eigen::Vector3d> gyr_buf;

  // TODO(wz)
  // Eigen::Vector3d G{0.0, 0.0, 9.8};
};

} 
}
#endif  // _CARTOGRAPHER_MAPPING_INTERNAL_3D_INTEGRATION_BASE_H_

