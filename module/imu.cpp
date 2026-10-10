#include "imu.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace chassis {
namespace {
constexpr double kGravity = 9.81;
constexpr double kTiltTimeConstant = 0.5;

int sensorAddress(const mjModel* model, const char* name, int type, int dimension) {
    const int id = mj_name2id(model, mjOBJ_SENSOR, name);
    if (id < 0) throw std::runtime_error(std::string("Missing chassis sensor: ") + name);
    if (model->sensor_type[id] != type || model->sensor_dim[id] != dimension)
        throw std::runtime_error(std::string("Wrong chassis sensor type/dimension: ") + name);
    return model->sensor_adr[id];
}

Distance readDistance(const mjData* data, int address) {
    const double metres = data->sensordata[address];
    return {std::isfinite(metres) && metres >= 0 ? metres : 0,
            std::isfinite(metres) && metres >= 0};
}
}  // namespace

Imu::Imu(const mjModel* model, SensorNames names)
    : model_(model),
      gyro_(sensorAddress(model, names.gyro, mjSENS_GYRO, 3)),
      accel_(sensorAddress(model, names.accelerometer, mjSENS_ACCELEROMETER, 3)),
      front_(sensorAddress(model, names.front, mjSENS_RANGEFINDER, 1)),
      down_(sensorAddress(model, names.down, mjSENS_RANGEFINDER, 1)) {}

void Imu::reset() {
    initialized_ = false;
    reading_ = {};
}

const ImuReading& Imu::update(const mjData* data) {
    if (!data) throw std::invalid_argument("IMU update requires mjData");
    const double previousTime = reading_.time;
    const bool first = !initialized_ || data->time < previousTime;
    reading_.time = data->time;
    for (int axis = 0; axis < 3; ++axis) {
        reading_.gyro[axis] = data->sensordata[gyro_ + axis];
        reading_.acceleration[axis] = data->sensordata[accel_ + axis];
    }
    reading_.front = readDistance(data, front_);
    reading_.down = readDistance(data, down_);

    const auto& a = reading_.acceleration;
    const double magnitude = std::hypot(a[0], a[1], a[2]);
    const bool levelMeasurement = std::isfinite(magnitude) &&
                                  std::abs(magnitude - kGravity) < 2.0;
    const double measuredRoll = std::atan2(a[1], a[2]);
    const double measuredPitch = std::atan2(-a[0], std::hypot(a[1], a[2]));
    if (first) {
        reading_.roll = levelMeasurement ? measuredRoll : 0;
        reading_.pitch = levelMeasurement ? measuredPitch : 0;
        reading_.yaw = 0;
        initialized_ = true;
        return reading_;
    }

    const double dt = data->time - previousTime;
    if (dt <= 0) return reading_;
    const auto& w = reading_.gyro;
    const double sr = std::sin(reading_.roll), cr = std::cos(reading_.roll);
    const double cp = std::cos(reading_.pitch);
    const double safeCp = std::copysign(std::max(std::abs(cp), 1e-3), cp);
    const double tiltRate = w[1] * sr + w[2] * cr;
    reading_.roll += dt * (w[0] + std::tan(reading_.pitch) * tiltRate);
    reading_.pitch += dt * (w[1] * cr - w[2] * sr);
    reading_.yaw += dt * tiltRate / safeCp;
    if (levelMeasurement) {
        const double alpha = std::exp(-dt / kTiltTimeConstant);
        reading_.roll += (1 - alpha) * std::remainder(measuredRoll - reading_.roll, 2 * mjPI);
        reading_.pitch += (1 - alpha) * (measuredPitch - reading_.pitch);
    }
    return reading_;
}
}  // namespace chassis
