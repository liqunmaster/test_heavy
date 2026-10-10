#pragma once

// All lengths are in metres. +X forward, +Y left; positive yaw is CCW.
namespace mecanum {
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kFootprintHalfSize = 0.295; // 590 x 590 mm overall envelope
inline constexpr double kDeckHalfSize = 0.295;   // 590 mm deck length
inline constexpr double kDeckHalfWidth = 0.150;  // 300 mm deck width, clear of wheels
inline constexpr double kWheelRadius = 0.076;   // 76 mm radius (half of the previous wheel)
inline constexpr double kWheelHalfLength = 0.170; // wheel centre X = +/-17 cm
inline constexpr double kWheelHalfTrack = 0.210;  // wheel centre Y = +/-21 cm
inline constexpr double kSuspensionArmLength = 0.080; // horizontal pivot-to-wheel reach
inline constexpr double kSuspensionArmLateral = 0.040;
inline constexpr double kSuspensionRodOffset = 0.065;
inline constexpr double kSuspensionMinHeight = 0.120;
inline constexpr double kSuspensionMaxHeight = 0.500;
inline constexpr double kHeightCommandSpeed = 0.080; // m/s while Up/Down is held
inline constexpr double kYawHoldP = 12.0;
inline constexpr double kYawHoldD = 3.0;
inline constexpr double kLeverArm = kWheelHalfLength + kWheelHalfTrack;

// Keyboard sensitivity: edit these two constants and rebuild.
inline constexpr double kLinearSpeed = -2.0;     // m/s
inline constexpr double kAngularSpeed = 2*kPi;    // rad/s = 180 degrees/s
inline constexpr double kCameraDistance = 3.2;
}  // namespace mecanum
