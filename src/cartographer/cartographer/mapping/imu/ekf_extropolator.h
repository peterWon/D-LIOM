#ifndef __EKF_EXTROPOLATOR__
#define __EKF_EXTROPOLATOR__

#include <deque>
#include "cartographer/mapping/imu/eskf.h"
#include "cartographer/mapping/imu/types.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/common/time.h"
/**********************************************
用EKF来推算IMU的状态，进而纠正雷达点云观测值，并为scan-to-map的配准提供初始值，
配准值反过来重置或者更新滤波器.
1) EKF递推
2) 点云观测时刻的插值预测
3) 配准数据的观测更新
4) 回环检测后的模型重置
*************************************************/

namespace cartographer {
namespace mapping {
namespace imu{
const double kStandStillInitilizationTime = 5.0;//sec
class ExtropolatorEKF{
public:
    ExtropolatorEKF(){}
    ~ExtropolatorEKF(){}
    bool Initialize(){
        if(stand_still_time_ < kStandStillInitilizationTime){
            return false;
        }
        Eigen::Vector3f accel_accum;
        Eigen::Vector3f gyro_accum;
        int num_readings = 0;

        accel_accum.setZero();
        gyro_accum.setZero();

        for(const auto& entry : imu_queue_){
            auto imu_time = std::get<0>(entry);
            auto imu_reading = std::get<1>(entry);

            accel_accum += imu_reading.a;
            gyro_accum += imu_reading.omega;
            num_readings++;
        }

        Eigen::Vector3f accel_mean = accel_accum / num_readings;
        Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

        init_imu_state_.b_g = gyro_mean;
        init_imu_state_.g << 0.0, 0.0, -9.81;
        //初始时刻IMU与世界坐标系的转换关系
        init_imu_state_.q_IG = imu::Quaternion<float>::FromTwoVectors(
            -init_imu_state_.g, accel_mean);

        init_imu_state_.b_a = 
            init_imu_state_.q_IG*init_imu_state_.g + accel_mean;

        init_imu_state_.p_I_G.setZero();
        init_imu_state_.v_I_G.setZero();
        const auto q = init_imu_state_.q_IG;

        LOG(ERROR)<<"\nInitial IMU State" <<
            "\n--p_I_G " << init_imu_state_.p_I_G.transpose() <<
            "\n--q_IG " << q.w() << "," << q.x() << "," 
                        << q.y() << "," << q.x() <<
            "\n--v_I_G " << init_imu_state_.v_I_G.transpose() <<
            "\n--b_a " << init_imu_state_.b_a.transpose() <<
            "\n--b_g " << init_imu_state_.b_g.transpose() <<
            "\n--g " << init_imu_state_.g.transpose()<<
            "\n--accel_mean "<< accel_mean[0]
                             <<","<<accel_mean[1]
                             <<","<<accel_mean[2];
        setupEskfParas();
        eskf_.initialize(noise_params_, msckf_params_, init_imu_state_);
        return true;
    };
    
    void AddImuData(const sensor::ImuData& imu_data){
        common::Time cur_imu_time = imu_data.time;
        if(prev_imu_time_ == common::Time::min()){
            stand_still_time_ += common::ToSeconds(
                cur_imu_time - prev_imu_time_);
            prev_imu_time_ = cur_imu_time;
            return;
        }

        imu::imuReading<float> current_imu;

        current_imu.a[0] = imu_data.linear_acceleration[0];
        current_imu.a[1] = imu_data.linear_acceleration[1];
        current_imu.a[2] = imu_data.linear_acceleration[2];

        current_imu.omega[0] = imu_data.angular_velocity[0];
        current_imu.omega[1] = imu_data.angular_velocity[1];
        current_imu.omega[2] = imu_data.angular_velocity[2];

        current_imu.dT = common::ToSeconds(cur_imu_time - prev_imu_time_);

        imu_queue_.emplace_back(cur_imu_time, current_imu);

        stand_still_time_ += common::ToSeconds(
                cur_imu_time - prev_imu_time_);
        prev_imu_time_ = cur_imu_time;
    }
    //
    transform::Rigid3d ExtrapolatePose(common::Time time){
        return transform::Rigid3d();
    } 
private:
    void setupEskfParas(){
        //TODO: setup calibration transforms between LiDAR and imu.
        Eigen::Matrix<float,12,1> Q_imu_vars;
        float w_var=1e-4, dbg_var=3.6733e-5, a_var=1e-2, dba_var=7e-2;
        Q_imu_vars << w_var, w_var, w_var, dbg_var, dbg_var, dbg_var,
                        a_var,	a_var,	a_var, dba_var,dba_var,dba_var;

        Eigen::Matrix<float,15,1> IMUCovar_vars;
        float q_var_init=1e-5, bg_var_init=1e-2, v_var_init=1e-2, 
              ba_var_init=1e-2, p_var_init=1e-12;
        IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                        bg_var_init,bg_var_init,bg_var_init,
                        v_var_init, v_var_init, v_var_init,
                        ba_var_init,ba_var_init,ba_var_init,
                        p_var_init, p_var_init, p_var_init;

        // Setup noise parameters
        noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
        noise_params_.Q_imu = Q_imu_vars.asDiagonal();
        // noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
        // noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);

        // nh.param<float>("max_gn_cost_norm", msckf_params_.max_gn_cost_norm, 11);
        // msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
        msckf_params_.translation_threshold = 0.05;
        msckf_params_.min_rcond = 3e-12;
        msckf_params_.redundancy_angle_thresh = 0.005;
        msckf_params_.redundancy_distance_thresh = 0.05;
        msckf_params_.max_track_length = 1000;
        msckf_params_.min_track_length = 3;
        msckf_params_.max_cam_states = 20;
    };
    EsKF<float> eskf_;
    std::vector<std::tuple<common::Time, imuReading<float>>> imu_queue_;
    common::Time prev_imu_time_ = common::Time::min(); 
    
    double stand_still_time_ = 0.;
    imuState<float> init_imu_state_;

    Matrix3<float> R_imu_laser_;
    Vector3<float> p_imu_laser_;

    Matrix3<float> R_laser_imu_;
    Vector3<float> p_laser_imu_;

    noiseParams<float> noise_params_;
    MSCKFParams<float> msckf_params_;
    bool imu_calibrated_ = false;
};
}//namespace imu
}//namespace mapping
}//namespace cartographer
#endif//__EKF_EXTROPOLATOR__