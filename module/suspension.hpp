#pragma once

#include <array>
#include <mujoco/mujoco.h>
#include "imu.hpp"

namespace chassis {

struct SuspensionConfig {
    std::array<const char*, 4> motors{"motor_fl", "motor_fr", "motor_rl", "motor_rr"};
    double targetHeight = 0.175;
    double heightP = 0.8;
    double heightD = 0.08;
    double rollP = 0.42;
    double rollD = 0.055;
    double pitchP = 0.32;
    double pitchD = 0.045;
};

struct SuspensionStatus {
    bool heightValid = false;
    double height = 0;
    std::array<double, 4> targets{};
};

class ActiveSuspension {
 public:
    explicit ActiveSuspension(const mjModel* model, SuspensionConfig config = {});
    void reset();
    void setTargetHeight(double height);
    double targetHeight() const { return config_.targetHeight; }
    const SuspensionStatus& apply(mjData* data, const ImuReading& reading);
    const SuspensionStatus& status() const { return status_; }

 private:
    const mjModel* model_;
    SuspensionConfig config_;
    std::array<int, 4> motors_{};
    SuspensionStatus status_;
    bool havePreviousHeight_ = false;
    double previousTime_ = 0;
    double previousHeight_ = 0;
    double heightRate_ = 0;
};

}
