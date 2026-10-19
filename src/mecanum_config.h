#pragma once

namespace mecanum {
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kFootprintHalfSize = 0.295;
inline constexpr double kDeckHalfSize = 0.295;
inline constexpr double kDeckHalfWidth = 0.150;
inline constexpr double kWheelRadius = 0.076;
inline constexpr double kWheelHalfLength = 0.170;
inline constexpr double kWheelHalfTrack = 0.210;
inline constexpr double kSuspensionArmLength = 0.080;
inline constexpr double kSuspensionArmLateral = 0.040;
inline constexpr double kSuspensionRodOffset = 0.065;
inline constexpr double kSuspensionMinHeight = 0.120;
inline constexpr double kSuspensionMaxHeight = 0.500;
inline constexpr double kHeightCommandSpeed = 0.080;
inline constexpr double kYawHoldP = 12.0;
inline constexpr double kYawHoldD = 3.0;
inline constexpr double kLeverArm = kWheelHalfLength + kWheelHalfTrack;

inline constexpr double kLinearSpeed = -2.0;
inline constexpr double kAngularSpeed = 2*kPi;
inline constexpr double kCameraDistance = 3.2;
}
