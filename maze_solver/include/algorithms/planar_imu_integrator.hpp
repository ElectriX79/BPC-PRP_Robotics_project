#pragma once

#include <vector>

namespace algorithms {

    // Integrator of yaw from IMU
    class PlanarImuIntegrator {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        void update(float gyro_z, double dt) {
            float corrected = gyro_z - gyro_offset_;

            // Euler method of integration: new angle = old angle + speed * time
            theta_ += corrected * static_cast<float>(dt);
        }

        // Calibration
        void setCalibration(const std::vector<float>& gyro_samples) {
            if (gyro_samples.empty()) {
                gyro_offset_ = 0.0f;
                return;
            }
            float sum = 0.0f;
            for (float v : gyro_samples) {
                sum += v;
            }
            gyro_offset_ = sum / static_cast<float>(gyro_samples.size());
        }

        [[nodiscard]] float getYaw() const {
            return theta_;
        }


        [[nodiscard]] float getOffset() const {
            return gyro_offset_;
        }

        void reset() {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

        void resetYaw() {
            theta_ = 0.0f;
        }

    private:
        float theta_;       // Integrated yaw angle [rad]
        float gyro_offset_; // bias of IMU [rad/s]
    };

}