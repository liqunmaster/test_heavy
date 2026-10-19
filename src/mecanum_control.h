#pragma once
#include <array>
#include <cmath>
#include "mecanum_config.h"

namespace mecanum {
struct Velocity {
    double x = 0;
    double y = 0;
    double yaw = 0;
};
struct Command {
    Velocity world;
    std::array<double, 4> wheels;
};

inline Command solve(double referenceX, double referenceY, double spin,
                     double chassisYaw, double referenceYaw = 0) {
    const double cr = std::cos(referenceYaw), sr = std::sin(referenceYaw);
    const double delta = referenceYaw - chassisYaw;
    const double c = std::cos(delta), s = std::sin(delta);
    const double bx = c * referenceX - s * referenceY;
    const double by = s * referenceX + c * referenceY;
    return {{cr * referenceX - sr * referenceY,
             sr * referenceX + cr * referenceY, spin},
            {(bx - by - kLeverArm * spin) / kWheelRadius,
             (bx + by + kLeverArm * spin) / kWheelRadius,
             (bx + by - kLeverArm * spin) / kWheelRadius,
             (bx - by + kLeverArm * spin) / kWheelRadius}};
}

inline Command keyboardCommand(double forward, double left, bool ccw, bool cw,
                               double chassisYaw, double referenceYaw = 0) {
    const double scale = std::fmax(1.0, std::hypot(forward, left));
    return solve(kLinearSpeed * forward / scale, kLinearSpeed * left / scale,
                 kAngularSpeed * (static_cast<int>(ccw) - static_cast<int>(cw)),
                 chassisYaw, referenceYaw);
}
}
