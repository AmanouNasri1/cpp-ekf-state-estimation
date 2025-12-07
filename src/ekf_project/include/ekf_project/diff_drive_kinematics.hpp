#ifndef EKF_PROJECT__DIFF_DRIVE_KINEMATICS_HPP_
#define EKF_PROJECT__DIFF_DRIVE_KINEMATICS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>

class DiffDriveKinematics {
public:
    DiffDriveKinematics() {
        // 1. Initialize State [x, y, theta] to 0
        state_.setZero();

        // 2. Initialize Covariance P (3x3 Matrix)
        // We start with 0 uncertainty (we know we are at start)
        covariance_.setZero();

        // 3. Initialize Process Noise Q (Tuning Parameter!)
        // This represents how much "slip" we expect per step.
        process_noise_.setIdentity();
        process_noise_(0,0) = 0.005; // X noise
        process_noise_(1,1) = 0.005; // Y noise
        process_noise_(2,2) = 0.002; // Theta noise (Heading is usually hardest to keep)
    }

    void integrate(double v, double w, double dt) {
        // --- A. PREDICT STATE (Dead Reckoning) ---
        // Same math as yesterday
        double theta = state_(2);
        double delta_x = v * std::cos(theta) * dt;
        double delta_y = v * std::sin(theta) * dt;
        double delta_theta = w * dt;

        state_(0) += delta_x;
        state_(1) += delta_y;
        state_(2) += delta_theta;

        // Normalize Theta [-PI, PI]
        if (state_(2) > M_PI) state_(2) -= 2 * M_PI;
        if (state_(2) < -M_PI) state_(2) += 2 * M_PI;

        // --- B. PREDICT COVARIANCE (The EKF Part) ---
        // We calculate the Jacobian F (Linearized motion model)
        // F = [ 1   0   -v*sin(theta)*dt ]
        //     [ 0   1    v*cos(theta)*dt ]
        //     [ 0   0    1               ]
        
        Eigen::Matrix3d F;
        F.setIdentity();
        F(0, 2) = -v * std::sin(theta) * dt;
        F(1, 2) =  v * std::cos(theta) * dt;

        // Prediction Step: P = F * P * F_transpose + Q
        covariance_ = F * covariance_ * F.transpose() + process_noise_;
    }
    void update(double z_range, double z_bearing, double map_x, double map_y) {
        
        // 1. Expected Measurement (h(x))
        // Where do we THINK the landmark is based on our current estimate?
        double dx = map_x - state_(0);
        double dy = map_y - state_(1);
        double dist = std::sqrt(dx*dx + dy*dy);
        
        // Expected bearing = Angle to landmark - Robot Heading
        double expected_bearing = std::atan2(dy, dx) - state_(2);
        
        // Normalize angle [-PI, PI]
        if (expected_bearing > M_PI) expected_bearing -= 2 * M_PI;
        if (expected_bearing < -M_PI) expected_bearing += 2 * M_PI;

        // 2. Innovation (y = z - h(x))
        // The difference between what we saw and what we expected
        Eigen::Vector2d y;
        y(0) = z_range - dist;
        y(1) = z_bearing - expected_bearing;

        // Normalize bearing innovation
        if (y(1) > M_PI) y(1) -= 2 * M_PI;
        if (y(1) < -M_PI) y(1) += 2 * M_PI;

        // 3. Jacobian H (The Geometry Matrix)
        // Derivatives of Range/Bearing w.r.t x, y, theta
        Eigen::Matrix<double, 2, 3> H;
        // Range derivatives
        H(0,0) = -(dx / dist);
        H(0,1) = -(dy / dist);
        H(0,2) = 0; 
        
        // Bearing derivatives
        H(1,0) = (dy / (dist*dist));
        H(1,1) = -(dx / (dist*dist));
        H(1,2) = -1;

        // 4. Measurement Noise R
        // How accurate is our Lidar? 
        Eigen::Matrix2d R;
        R.setIdentity();
        R(0,0) = 0.05; // Range noise variance (e.g., 10cm accuracy)
        R(1,1) = 0.05; // Bearing noise variance (rad)

        // 5. Kalman Gain K = P * Ht * (H * P * Ht + R)^-1
        Eigen::Matrix2d S = H * covariance_ * H.transpose() + R;
        Eigen::Matrix<double, 3, 2> K = covariance_ * H.transpose() * S.inverse();

        // 6. Update State x = x + K * y
        state_ = state_ + K * y;
        
        // Normalize Theta again
        if (state_(2) > M_PI) state_(2) -= 2 * M_PI;
        if (state_(2) < -M_PI) state_(2) += 2 * M_PI;

        // 7. Update Covariance P = (I - K * H) * P
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        covariance_ = (I - K * H) * covariance_;
    }

    // Getters
    Eigen::Vector3d getState() const { return state_; }
    Eigen::Matrix3d getCovariance() const { return covariance_; }

private:
    Eigen::Vector3d state_;      // [x, y, theta]
    Eigen::Matrix3d covariance_; // 3x3 Uncertainty Matrix P
    Eigen::Matrix3d process_noise_; // Q Matrix
};

#endif