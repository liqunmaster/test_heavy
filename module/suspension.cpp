#include "suspension.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace chassis {

ActiveSuspension::ActiveSuspension(const mjModel* model, SuspensionConfig config)
    : model_(model), config_(config) {
    if (!model_) throw std::invalid_argument("Suspension requires mjModel");
    if (config_.targetHeight <= 0) throw std::invalid_argument("Target height must be positive");
    for (int i = 0; i < 4; ++i) {
        motors_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, config_.motors[i]);
        if (motors_[i] < 0) {
            throw std::runtime_error(std::string("Missing suspension motor: ") + config_.motors[i]);
        }  
        if (model_->actuator_trntype[motors_[i]] != mjTRN_JOINT || !model_->actuator_ctrllimited[motors_[i]]){
            throw std::runtime_error(std::string("Suspension motor needs a limited joint control: ") + config_.motors[i]);
        }
    }
}

void ActiveSuspension::reset() {
    status_ = {};
    havePreviousHeight_ = false;
    previousTime_ = 0;
    previousHeight_ = 0;
    heightRate_ = 0;
}

void ActiveSuspension::setTargetHeight(double height) {
    config_.targetHeight = std::clamp(height, 0.120, 0.500);
}

const SuspensionStatus& ActiveSuspension::apply(mjData* data, const ImuReading& reading) {
    if (!data) throw std::invalid_argument("Suspension update requires mjData");
    if (reading.time < previousTime_) reset();
    status_.heightValid = reading.down.valid && std::isfinite(reading.down.metres);
    double heightCorrection = 0;
    if (status_.heightValid) {
        status_.height = reading.down.metres * std::cos(reading.roll) * std::cos(reading.pitch);
        const double dt = reading.time - previousTime_;
        if (havePreviousHeight_ && dt > 0) {
            const double rawRate = std::clamp((status_.height - previousHeight_) / dt, -1.0, 1.0);
            const double alpha = std::exp(-dt / 0.05);
            heightRate_ = alpha * heightRate_ + (1 - alpha) * rawRate;
        } else {
            heightRate_ = 0;
        }
        previousHeight_ = status_.height;
        previousTime_ = reading.time;
        havePreviousHeight_ = true;
        heightCorrection = -config_.heightP * (config_.targetHeight - status_.height) + config_.heightD * heightRate_;
    } else {
        havePreviousHeight_ = false;
        heightRate_ = 0;
        previousTime_ = reading.time;
    }

    const double rollCorrection = config_.rollP * reading.roll + config_.rollD * reading.gyro[0];
    const double pitchCorrection = config_.pitchP * reading.pitch + config_.pitchD * reading.gyro[1];
    constexpr int xsign[4] = {1, 1, -1, -1};
    constexpr int ysign[4] = {1, -1, 1, -1};
    for (int i = 0; i < 4; ++i) {
        const int motor = motors_[i];
        const double target = heightCorrection + ysign[i] * rollCorrection - xsign[i] * pitchCorrection;
        const double low = model_->actuator_ctrlrange[2 * motor];
        const double high = model_->actuator_ctrlrange[2 * motor + 1];
        status_.targets[i] = std::clamp(target, low, high);
        data->ctrl[motor] = status_.targets[i];
    }
    return status_;
}

}
