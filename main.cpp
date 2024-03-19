#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <memory>

struct TspiFrame {
    uint64_t timestamp = 0;
    double position_x = 0, position_y = 0, position_z = 0;
    double velocity_x = 0, velocity_y = 0, velocity_z = 0;
    double acceleration_x = 0, acceleration_y = 0, acceleration_z = 0;
    double orientation_roll = 0, orientation_pitch = 0, orientation_yaw = 0;
    double angularRate_x = 0, angularRate_y = 0, angularRate_z = 0;
    double angularAcceleration_x = 0, angularAcceleration_y = 0, angularAcceleration_z = 0;
};

struct UniqueID {
    uint32_t site_id;
    uint32_t app_id;
    uint32_t object_id;

    bool operator<(const UniqueID& other) const {
        if (site_id < other.site_id) return true;
        if (site_id > other.site_id) return false;
        if (app_id < other.app_id) return true;
        if (app_id > other.app_id) return false;
        return object_id < other.object_id;
    }
};

class InterpolationStrategy {
public:
    virtual void Interpolate(TspiFrame& currentData, const TspiFrame& newData, double alpha) = 0;
    virtual ~InterpolationStrategy() {}
};


class LinearInterpolation : public InterpolationStrategy {
public:
    void Interpolate(TspiFrame& currentData, const TspiFrame& newData, double alpha) override {
        // Correct implementation of linear interpolation for each field
        // The alpha should be calculated before calling this method, based on timestamps and the desired interpolation logic

        // Linear interpolation for position
        currentData.position_x = (1 - alpha) * currentData.position_x + alpha * newData.position_x;
        currentData.position_y = (1 - alpha) * currentData.position_y + alpha * newData.position_y;
        currentData.position_z = (1 - alpha) * currentData.position_z + alpha * newData.position_z;

        // Linear interpolation for velocity
        currentData.velocity_x = (1 - alpha) * currentData.velocity_x + alpha * newData.velocity_x;
        currentData.velocity_y = (1 - alpha) * currentData.velocity_y + alpha * newData.velocity_y;
        currentData.velocity_z = (1 - alpha) * currentData.velocity_z + alpha * newData.velocity_z;

        // Linear interpolation for acceleration
        currentData.acceleration_x = (1 - alpha) * currentData.acceleration_x + alpha * newData.acceleration_x;
        currentData.acceleration_y = (1 - alpha) * currentData.acceleration_y + alpha * newData.acceleration_y;
        currentData.acceleration_z = (1 - alpha) * currentData.acceleration_z + alpha * newData.acceleration_z;

        // Linear interpolation for orientation (roll, pitch, yaw)
        currentData.orientation_roll = (1 - alpha) * currentData.orientation_roll + alpha * newData.orientation_roll;
        currentData.orientation_pitch = (1 - alpha) * currentData.orientation_pitch + alpha * newData.orientation_pitch;
        currentData.orientation_yaw = (1 - alpha) * currentData.orientation_yaw + alpha * newData.orientation_yaw;

        // Linear interpolation for angular rates
        currentData.angularRate_x = (1 - alpha) * currentData.angularRate_x + alpha * newData.angularRate_x;
        currentData.angularRate_y = (1 - alpha) * currentData.angularRate_y + alpha * newData.angularRate_y;
        currentData.angularRate_z = (1 - alpha) * currentData.angularRate_z + alpha * newData.angularRate_z;

        // Linear interpolation for angular accelerations
        currentData.angularAcceleration_x = (1 - alpha) * currentData.angularAcceleration_x + alpha * newData.angularAcceleration_x;
        currentData.angularAcceleration_y = (1 - alpha) * currentData.angularAcceleration_y + alpha * newData.angularAcceleration_y;
        currentData.angularAcceleration_z = (1 - alpha) * currentData.angularAcceleration_z + alpha * newData.angularAcceleration_z;
    }
};

class KalmanFilterInterpolation : public InterpolationStrategy {
public:
    KalmanFilterInterpolation() {
        int n = 3; // State dimension (x, y, z position)
        int m = 3; // Measurement dimension (x, y, z position)

        A = Eigen::MatrixXd::Identity(n, n); // State transition model
        C = Eigen::MatrixXd::Identity(m, n); // Measurement model
        Q = Eigen::MatrixXd::Identity(n, n) * 0.001; // Process noise covariance
        R = Eigen::MatrixXd::Identity(m, m) * 0.01; // Measurement noise covariance
        P = Eigen::MatrixXd::Identity(n, n); // Estimate error covariance
        I = Eigen::MatrixXd::Identity(n, n); // Identity matrix
    }

    void Interpolate(TspiFrame& currentData, const TspiFrame& newData, double alpha) override {
        Eigen::VectorXd y(3); // Measurement
        y << newData.position_x, newData.position_y, newData.position_z;

        // Prediction
        x = A * x;
        P = A * P * A.transpose() + Q;

        // Measurement Update (Correction)
        Eigen::MatrixXd K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
        x = x + K * (y - C * x);
        P = (I - K * C) * P;

        // Update the current data with the new estimated state
        currentData.position_x = x(0);
        currentData.position_y = x(1);
        currentData.position_z = x(2);
    }

private:
    Eigen::VectorXd x; // State (x, y, z position)
    Eigen::MatrixXd A, C, Q, R, P, I;
};

std::map<UniqueID, TspiFrame> m_tspi_frame_map;

class Plane {
public:
    Plane(uint32_t site_id, uint32_t app_id, uint32_t object_id, InterpolationStrategy* strategy)
            : unique_id_{site_id, app_id, object_id}, current_timestamp_(0), strategy_(strategy) {}

    void Update() {
        auto it = m_tspi_frame_map.find(unique_id_);
        if (it != m_tspi_frame_map.end()) {
            const TspiFrame& newFrame = it->second;
            double alpha = 0.5; // Placeholder for alpha calculation
            if (strategy_) {
                strategy_->Interpolate(tspi_data_, newFrame, alpha);
            }
            current_timestamp_ = newFrame.timestamp; // Update timestamp after interpolation
        }
    }

    void PrintPosition() const {
        std::cout << "Position of plane ID: " << unique_id_.site_id << "-" << unique_id_.app_id << "-" << unique_id_.object_id << " is ("
                  << tspi_data_.position_x << ", " << tspi_data_.position_y << ", " << tspi_data_.position_z << ")" << std::endl;
    }

private:
    UniqueID unique_id_;
    TspiFrame tspi_data_;
    uint64_t current_timestamp_;
    InterpolationStrategy* strategy_;

// Continue the interpolation for acceleration_y and acceleration_z, and add orientation and angular rates/accelerations
    void InterpolateData(const TspiFrame& newFrame) {
    }
};

class Simulation {
public:
    Simulation() : simulation_time_(0), simulation_duration_(1000) {
        // Instantiate interpolation strategies
        linearInterpolation = std::make_unique<LinearInterpolation>();
        // kalmanFilterInterpolation = std::make_unique<KalmanFilterInterpolation>(); // Uncomment if using Kalman Filter

        // Assuming a single strategy for simplicity; you could choose based on other criteria
        currentStrategy = linearInterpolation.get();
    }

    void LoadModels() {
        std::cout << "Loading models..." << std::endl;
        // Use the currentStrategy for all planes for simplicity
        for (int i = 1; i <= 10; ++i) {
            planes_.emplace_back(i, 1, i, currentStrategy);
        }
    }

    void Update() {
        std::cout << "Updating simulation..." << std::endl;
        for (auto& plane : planes_) {
            plane.Update();
            plane.PrintPosition();
        }
        simulation_time_++;
    }

    void Run() {
        std::cout << "Running simulation..." << std::endl;
        LoadModels();
        while (simulation_time_ < simulation_duration_) {
            Update();
        }
    }

private:
    std::vector<Plane> planes_;
    uint64_t simulation_time_;
    uint64_t simulation_duration_;
    std::unique_ptr<LinearInterpolation> linearInterpolation;
    // std::unique_ptr<KalmanFilterInterpolation> kalmanFilterInterpolation; // Uncomment if using Kalman Filter
    InterpolationStrategy* currentStrategy;
};

int main() {
    // Fill in the map with your TspiFrame data...


    // m_tspi_frame_map.try_emplace(...);

    Simulation sim;
    sim.Run();
}

