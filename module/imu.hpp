#pragma once

#include <array>
#include <mujoco/mujoco.h>

namespace chassis {

struct SensorNames {
    const char* gyro = "imu_gyro";
    const char* accelerometer = "imu_accel";
    const char* front = "range_front";
    const char* down = "range_down";
};

struct Distance {
    double metres = 0;
    bool valid = false;
};

struct ImuReading {
    double time = 0;
    std::array<double, 3> gyro{};          // rad/s, sensor-local XYZ
    std::array<double, 3> acceleration{};  // m/s^2, sensor-local XYZ (specific force)
    double roll = 0;                       // radians, sensor frame relative to level
    double pitch = 0;
    double yaw = 0;                        // radians, relative to initial heading
    Distance front;
    Distance down;
};

class Imu {
 public:
    explicit Imu(const mjModel* model, SensorNames names = {});
    void reset();
    const ImuReading& update(const mjData* data);
    const ImuReading& reading() const { return reading_; }

 private:
    const mjModel* model_;
    int gyro_;
    int accel_;
    int front_;
    int down_;
    bool initialized_ = false;
    ImuReading reading_;
};

}  // namespace chassis
