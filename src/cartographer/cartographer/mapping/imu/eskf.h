#ifndef __IMU_INTEGRATION__
#define __IMU_INTEGRATION__

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/StdVector>
#include <boost/math/distributions/chi_squared.hpp>

#include "cartographer/mapping/imu/types.h"
#include "cartographer/mapping/imu/matrix_utils.h"

namespace cartographer {
namespace mapping {
namespace imu{
using namespace Eigen;

/* Notes to self:
 *		- Noiseparams in calcGNPoseEst
 *		- The thing with the quaternions being inverted
 */
template <typename _S>
class EsKF {
private: 
    noiseParams<_S> noise_params_;
    MSCKFParams<_S> msckf_params_;
    
    //todo,assign parameter to this variable
    Point<_S> p_LI_;
    Quaternion<_S> q_LI_;
    
    imuState<_S> imu_state_;
    laserState<_S> laser_state_;

    Matrix<_S,15,15> imu_covar_;
    // add laser
    Matrix<_S,6,6> laser_covar_;
    Matrix<_S,15,6> imu_laser_covar_;

    std::vector<_S> chi_squared_test_table;
    Vector3<_S> pos_init_;
    Quaternion<_S> quat_init_;

    Matrix<_S,15,15> F_;
    Matrix<_S,15,15> Phi_;
    Matrix<_S,15,12> G_;

    MatrixX<_S> P_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EsKF(){}

    // Initializes the filter state and parameters.
    void initialize(const noiseParams<_S>& noise_params,
                    const MSCKFParams<_S>& msckf_params,
                    const imuState<_S>& imu_state) {
        // Constructor:
        noise_params_ = noise_params;
        msckf_params_ = msckf_params;
        imu_state_ = imu_state;
        pos_init_ = imu_state_.p_I_G;
        imu_state_.p_I_G_null = imu_state_.p_I_G;
        imu_state_.v_I_G_null = imu_state_.v_I_G;
        imu_state_.q_IG_null = imu_state_.q_IG;
        imu_covar_ = noise_params.initial_imu_covar;

        // Initialize the chi squared test table with confidence
        // level 0.95.
        chi_squared_test_table.resize(99);
        for (int i = 1; i < 100; ++i) {
            boost::math::chi_squared chi_squared_dist(i);
            chi_squared_test_table[i-1] = boost::math::quantile(
                chi_squared_dist, 0.05);
        }
    // TODO: Adjust for 0-sized covar?
    }

    // Given an IMU measurement, propagates the latest camera pose to the timestamp in measurement
    // using the acceleration and angular velocity in measurement.
    void propagate(imuReading<_S> &measurement_) {
        calcF(imu_state_, measurement_);
        calcG(imu_state_);

        imuState<_S> imu_state_prop = propogateImuStateRK(
            imu_state_, measurement_);
        F_ *= measurement_.dT;
        Phi_ = F_.exp(); // Matrix exponential,转移矩阵

        // 这里没看懂？？？
        // Apply observability constraints - enforce nullspace of Phi
        // Ref: Observability-constrained Vision-aided Inertial Navigation, Hesch J.
        // et al. Feb, 2012
        Matrix3<_S> R_kk_1(imu_state_.q_IG_null);
        Phi_.template block<3, 3>(0, 0) =
            imu_state_prop.q_IG.toRotationMatrix() * R_kk_1.transpose();

        Vector3<_S> u = R_kk_1 * imu_state_.g;
        RowVector3<_S> s = (u.transpose() * u).inverse() * u.transpose();

        Matrix3<_S> A1 = Phi_.template block<3, 3>(6, 0);
        Vector3<_S> tmp = imu_state_.v_I_G_null - imu_state_prop.v_I_G;
        Vector3<_S> w1 = vectorToSkewSymmetric(tmp) * imu_state_.g;
        Phi_.template block<3, 3>(6, 0) = A1 - (A1 * u - w1) * s;

        Matrix3<_S> A2 = Phi_.template block<3, 3>(12, 0);
        tmp = measurement_.dT * imu_state_.v_I_G_null +
            imu_state_.p_I_G_null - imu_state_prop.p_I_G;
        Vector3<_S> w2 = vectorToSkewSymmetric(tmp) * imu_state_.g;
        Phi_.template block<3, 3>(12, 0) = A2 - (A2 * u - w2) * s;

        Matrix<_S, 15, 15> imu_covar_prop = 
        Phi_ * (imu_covar_ + G_ * noise_params_.Q_imu * G_.transpose() 
        * measurement_.dT) * Phi_.transpose();

        // Apply updates directly
        // 更新IMU的当前状态,由于上一时刻的误差态已经重新置零，因此这里只需要递推名义态及相应的误差协方差
        // 误差态的估计靠测量更新来完成,因此没有测量，则没有修正,仅仅是协方差递推而已
        imu_state_ = imu_state_prop;
        imu_state_.q_IG_null = imu_state_.q_IG;
        imu_state_.v_I_G_null = imu_state_.v_I_G;
        imu_state_.p_I_G_null = imu_state_.p_I_G;

        imu_covar_ = (imu_covar_prop + imu_covar_prop.transpose()) / 2.0;
        // imu_cam_covar_ = Phi_ * imu_cam_covar_;
        // 更新laser
        imu_laser_covar_ = Phi_ * imu_laser_covar_;
    }

    
    inline imuState<_S> getImuState(){
        return imu_state_;
    }
private:
    Quaternion<_S> buildUpdateQuat(const Vector3<_S> &deltaTheta) {
        Vector3<_S> deltaq = 0.5 * deltaTheta;
        Quaternion<_S> updateQuat;
        // Replaced with squaredNorm() ***1x1 result so using sum instead of creating
        // another variable and then referencing the 0th index value***
        _S checkSum = deltaq.squaredNorm();
        if (checkSum > 1) {
            updateQuat.w() = 1;
            updateQuat.x() = -deltaq(0);
            updateQuat.y() = -deltaq(1);
            updateQuat.z() = -deltaq(2);
        } else {
            updateQuat.w() = sqrt(1 - checkSum);
            updateQuat.x() = -deltaq(0);
            updateQuat.y() = -deltaq(1);
            updateQuat.z() = -deltaq(2);
        }

        updateQuat.normalize();

        return updateQuat;
    }
    //公式10中的F和G的计算式，F和G为误差的线性化矩阵（雅克比）
    inline void calcF(const imuState<_S> &imu_state_k,
                    const imuReading<_S> &measurement_k) {
        /* Multiplies the error state in the linearized continuous-time
            error state model */
        F_.setZero();

        Vector3<_S> omegaHat, aHat;
        omegaHat = measurement_k.omega - imu_state_k.b_g;
        aHat = measurement_k.a - imu_state_k.b_a;
        Matrix3<_S> C_IG = imu_state_k.q_IG.toRotationMatrix();

        F_.template block<3, 3>(0, 0) = -vectorToSkewSymmetric(omegaHat);
        F_.template block<3, 3>(0, 3) = -Matrix3<_S>::Identity();
        F_.template block<3, 3>(6, 0) 
          = -C_IG.transpose() * vectorToSkewSymmetric(aHat);
        F_.template block<3, 3>(6, 9) = -C_IG.transpose();
        F_.template block<3, 3>(12, 6) = Matrix3<_S>::Identity();
    }

    inline void calcG(const imuState<_S> &imu_state_k) {
        /* Multiplies the noise std::vector in the linearized continuous-time
            error state model */
        G_.setZero();

        Matrix3<_S> C_IG = imu_state_k.q_IG.toRotationMatrix();

        G_.template block<3, 3>(0, 0) = -Matrix3<_S>::Identity();
        G_.template block<3, 3>(3, 3) = Matrix3<_S>::Identity();
        G_.template block<3, 3>(6, 6) = -C_IG.transpose();
        G_.template block<3, 3>(9, 9) = Matrix3<_S>::Identity();
    }

    // 求取观测模型的雅克比
    // 求取Hf阵的左零空间矩阵，将与特征点相关的Hf矩阵消除，使得残差项只与Hx（相机姿态）有关
    void calcMeasJacobian(const Vector3<_S> &p_f_G,
                        const std::vector<size_t> &camStateIndices,
                        MatrixX<_S> &H_o_j,
                        MatrixX<_S> &A_j) {
        // Calculates H_o_j according to Mourikis 2007

        /* MatrixX<_S> H_f_j = MatrixX<_S>::Zero(2 * camStateIndices.size(), 3);
        MatrixX<_S> H_x_j =
            MatrixX<_S>::Zero(2 * camStateIndices.size(), 15 + 6 * cam_states_.size());

        for (int c_i = 0; c_i < camStateIndices.size(); c_i++) {
            size_t index = camStateIndices[c_i];
            Vector3<_S> p_f_C = cam_states_[index].q_CG.toRotationMatrix() *
            (p_f_G - cam_states_[index].p_C_G);

            _S X, Y, Z;

            X = p_f_C(0);
            Y = p_f_C(1);
            Z = p_f_C(2);

            // cout << "p_f_C: " << p_f_C.transpose() << ". X: " << X << ", Y: " << Y <<
            // ", Z: " << Z << endl;

            Matrix<_S, 2, 3> J_i;
            J_i << 1, 0, -X / Z, 0, 1, -Y / Z;
            J_i *= 1 / Z;

            // Enforce observability constraint, see propagation for citation
            Matrix<_S, 2, 6> A;
            A << J_i * vectorToSkewSymmetric(p_f_C),
            -J_i * cam_states_[index].q_CG.toRotationMatrix();

            Matrix<_S, 6, 1> u = Matrix<_S, 6, 1>::Zero();
            u.head(3) = cam_states_[index].q_CG.toRotationMatrix() * imu_state_.g;
            Vector3<_S> tmp = p_f_G - cam_states_[index].p_C_G;
            u.tail(3) = vectorToSkewSymmetric(tmp) * imu_state_.g;

            Matrix<_S, 2, 6> H_x =
            A - A * u * (u.transpose() * u).inverse() * u.transpose();
            Matrix<_S, 2, 3> H_f = -H_x.template block<2, 3>(0, 3);
            H_f_j.template block<2, 3>(2 * c_i, 0) = H_f;

            // Potential indexing problem zone
            H_x_j.template block<2, 6>(2 * c_i, 15 + 6 * (index)) = H_x;
        }

        int jacobian_row_size = 2 * camStateIndices.size();

        JacobiSVD<MatrixX<_S>> svd_helper(H_f_j, ComputeFullU | ComputeThinV);
        A_j = svd_helper.matrixU().rightCols(jacobian_row_size - 3);

        H_o_j = A_j.transpose() * H_x_j; */ 
    }

    VectorX<_S> calcResidual(
        const Vector3<_S> &p_f_G,
        const std::vector<camState<_S>> &camStates,
        const std::vector<Vector2<_S>, Eigen::aligned_allocator<Vector2<_S>>> &observations) {
        // CALCRESIDUAL Calculates the residual for a feature position

        VectorX<_S> r_j = VectorX<_S>::Constant(2 * camStates.size(),
                            std::numeric_limits<_S>::quiet_NaN());

        int iter = 0;
        //重投影误差=根据状态量当前估计值推算出来的像素坐标-特征点检测或跟踪（测量）得到的归一化像素坐标
        //最小二乘求得的是改正值
        //雅克比阵为当前估计值处的雅克比
        for (auto state_i : camStates) {
            Vector3<_S> p_f_C = state_i.q_CG.toRotationMatrix() 
                              * (p_f_G - state_i.p_C_G);
            Vector2<_S> zhat_i_j = p_f_C.template head<2>() / p_f_C(2);

            r_j.template segment<2>(2 * iter) = observations[iter] - zhat_i_j;
            iter++;
        }

        return r_j;
    }

    void cost(const Isometry3<_S>& T_c0_ci,
            const Vector3<_S>& x, const Vector2<_S>& z,
            _S& e) const {
        // Compute hi1, hi2, and hi3 as Equation (37).
        const _S &alpha = x(0);
        const _S &beta = x(1);
        const _S &rho = x(2);

        Vector3<_S> h = T_c0_ci.linear() * Vector3<_S>(alpha, beta, 1.0) +
            rho * T_c0_ci.translation();
        _S &h1 = h(0);
        _S &h2 = h(1);
        _S &h3 = h(2);

        // Predict the feature observation in ci frame.
        Vector2<_S> z_hat(h1 / h3, h2 / h3);

        // Compute the residual.
        e = (z_hat - z).squaredNorm();
        return;
    }

    //5阶RK公式递推IMU的状态,推算PVQ的名义值
    imuState<_S> propogateImuStateRK(const imuState<_S> &imu_state_k,
                                    const imuReading<_S> &measurement_k) {
        imuState<_S> imuStateProp = imu_state_k;
        const _S dT(measurement_k.dT);

        Vector3<_S> omega_vec = measurement_k.omega - imu_state_k.b_g;
        Matrix4<_S> omega_psi = 0.5 * omegaMat(omega_vec);

        // Note: MSCKF Matlab code assumes quaternion form: -x,-y,-z,w
        //     Eigen quaternion is of form: w,x,y,z
        // Following computation accounts for this change
        
        Vector4<_S> y0, k0, k1, k2, k3, k4, k5, y_t;
        y0(0) = -imu_state_k.q_IG.x();
        y0(1) = -imu_state_k.q_IG.y();
        y0(2) = -imu_state_k.q_IG.z();
        y0(3) = imu_state_k.q_IG.w();

        k0 = omega_psi * (y0);
        k1 = omega_psi * (y0 + (k0 / 4.) * dT);
        k2 = omega_psi * (y0 + (k0 / 8. + k1 / 8.) * dT);
        k3 = omega_psi * (y0 + (-k1 / 2. + k2) * dT);
        k4 = omega_psi * (y0 + (k0 * 3. / 16. + k3 * 9. / 16.) * dT);
        k5 = omega_psi * (y0 + (-k0 * 3. / 7. + k1 * 2. / 7. + k2 * 12. / 7. 
             - k3 * 12. / 7. + k4 * 8. / 7.) * dT);

        y_t = y0 + (7. * k0 + 32. * k2 + 12. * k3 + 32. * k4 + 7. * k5) 
              * dT / 90.;

        Quaternion<_S> q(y_t(3), -y_t(0), -y_t(1), -y_t(2));
        q.normalize();//四元数要归一化才能表示方向

        imuStateProp.q_IG = q;
        Vector3<_S> delta_v_I_G = 
          (((imu_state_k.q_IG.toRotationMatrix()).transpose()) 
              * (measurement_k.a - imu_state_k.b_a) + imu_state_k.g) * dT;

        imuStateProp.v_I_G += delta_v_I_G;
        imuStateProp.p_I_G = imu_state_k.p_I_G + imu_state_k.v_I_G * dT;
        return imuStateProp;
    }
};
}//namespace imu
}//namespace mapping
}//namespace cartographer
#endif