#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "mecanum_config.h"

namespace {
std::string values(std::initializer_list<double> list) {
    std::ostringstream result;
    result << std::fixed << std::setprecision(6);
    for (double value : list) result << value << ' ';
    return result.str();
}

void geom(std::ostream& out, const std::string& name, const std::string& type,
          const std::string& size, const std::string& pos, const std::string& material,
          const std::string& extra = "") {
    out << "      <geom name=\"" << name << "\" type=\"" << type << "\" size=\"" << size
        << "\" pos=\"" << pos << "\" material=\"" << material << "\" " << extra << "/>\n";
}
}

int main(int argc, char** argv) {
    using namespace mecanum;
    const std::filesystem::path path = argc > 1 ? argv[1] : "model/mecanum_wheel.xml";
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) { std::cerr << "Cannot write " << path << '\n'; return 1; }
    out << R"(<?xml version="1.0" encoding="utf-8"?>
<mujoco model="mecanum_chassis">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" iterations="80" ls_iterations="40"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.8 0.8 0.8"/>
    <global azimuth="180" elevation="-35" offwidth="1024" offheight="768"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="checker" type="2d" builtin="checker" rgb1="0.12 0.12 0.12"
             rgb2="0.05 0.05 0.05" width="256" height="256"/>
    <material name="ground" texture="checker" texrepeat="2 2" texuniform="true"
              reflectance="0" specular="0" shininess="0"/>
    <material name="navy" rgba="0.06 0.17 0.27 1" shininess="0.5"/>
    <material name="silver" rgba="0.65 0.72 0.78 1" shininess="0.6"/>
    <material name="rubber" rgba="0.055 0.065 0.08 1" shininess="0.1"/>
    <material name="gold" rgba="0.92 0.53 0.13 1" shininess="0.6"/>
    <material name="cyan" rgba="0.1 0.8 0.88 1" shininess="0.4"/>
  </asset>
  <worldbody>
    <light directional="true" pos="0 -2 4" dir="0 0 -1" diffuse="0.75 0.75 0.75"/>
    <geom name="floor" type="plane" size="12 12 0.1" material="ground"
          contype="1" conaffinity="1" friction="0.01 0.0001 0.0001"/>
    <geom name="single_side_bridge_5cm" type="box" pos="0.90 0.210 0.025"
          size="0.40 0.10 0.025" material="gold" contype="1" conaffinity="1"
          friction="0.8 0.01 0.001" margin="0.002" solref="0.02 1" solimp="0.95 0.99 0.001"/>
    <geom name="step_20cm" type="box" pos="2.40 0 0.100"
          size="0.20 0.34 0.100" material="cyan" contype="1" conaffinity="1"
          friction="0.8 0.01 0.001" margin="0.002" solref="0.02 1" solimp="0.95 0.99 0.001"/>
)";
    out << "    <body name=\"chassis\" pos=\"" << values({0, 0, kWheelRadius + .001}) << "\">\n";
    out << R"(      <inertial pos="0 0 0.06" mass="5" diaginertia="0.08 0.12 0.18"/>
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_y" type="slide" axis="0 1 0"/>
      <joint name="yaw" type="hinge" axis="0 0 1"/>
      <joint name="heave" type="slide" axis="0 0 1" limited="true" range="-0.08 0.08" damping="20"/>
      <joint name="roll" type="hinge" axis="1 0 0" limited="true" range="-0.25 0.25" damping="12" stiffness="18" springref="0"/>
      <joint name="pitch" type="hinge" axis="0 1 0" limited="true" range="-0.25 0.25" damping="12" stiffness="18" springref="0"/>
      <site name="imu_site" pos="0 0 0.155" size="0.008"/>
      <site name="range_front_site" pos="0.330 0 0.098" quat="0.707106781186548 0 0.707106781186548 0" size="0.006"/>
      <site name="range_down_site" pos="0 0 0" quat="0 1 0 0" size="0.006"/>
)";
    geom(out, "lower_deck", "box", values({kDeckHalfSize, kDeckHalfWidth, .015}), "0 0 0.058", "silver");

    const char* labels[] = {"fl", "fr", "rl", "rr"};
    const int xsign[] = {1, 1, -1, -1}, ysign[] = {1, -1, 1, -1};
    const int handedness[] = {-1, 1, 1, -1};
    const double scale = kWheelRadius / .078;
    for (int w = 0; w < 4; ++w) {
        const std::string label = labels[w];
        out << "      <body name=\"susp_" << label << "\" pos=\""
            << values({xsign[w] * kWheelHalfLength, ysign[w] * kWheelHalfTrack, 0}) << "\">\n"
            << "        <joint name=\"susp_" << label
            << "\" type=\"slide\" axis=\"0 0 1\" limited=\"true\" range=\"-0.20 0.20\" damping=\"24\"/>\n"
            << "        <inertial pos=\"0 0 0.06\" mass=\"0.12\" diaginertia=\"0.00025 0.00025 0.00025\"/>\n"
            << "        <geom name=\"link_inner_" << label << "\" type=\"box\" size=\"0.008 0.008 0.10\" material=\"gold\" contype=\"0\" conaffinity=\"0\" pos=\"0 -0.045 "
            << kWheelRadius + 0.10 << "\"/>\n"
            << "        <geom name=\"link_outer_" << label << "\" type=\"box\" size=\"0.008 0.008 0.10\" material=\"gold\" contype=\"0\" conaffinity=\"0\" pos=\"0 0.045 "
            << kWheelRadius + 0.10 << "\"/>\n"
            << "        <geom name=\"rod_guide_inner_" << label << "\" type=\"box\" size=\"0.014 0.014 0.06\" material=\"silver\" contype=\"0\" conaffinity=\"0\" pos=\"0 -0.045 "
            << kWheelRadius + 0.12 << "\"/>\n"
            << "        <geom name=\"rod_guide_outer_" << label << "\" type=\"box\" size=\"0.014 0.014 0.06\" material=\"silver\" contype=\"0\" conaffinity=\"0\" pos=\"0 0.045 "
            << kWheelRadius + 0.12 << "\"/>\n"
            << "        <geom name=\"rod_foot_" << label << "\" type=\"box\" size=\"0.014 0.055 0.012\" material=\"gold\" contype=\"0\" conaffinity=\"0\" pos=\"0 0 "
            << kWheelRadius + 0.012 << "\"/>\n"
            << "        <body name=\"wheel_" << label << "\" pos=\"0 0 0\">\n"
            << "        <joint name=\"w_" << label << "\" type=\"hinge\" axis=\"0 1 0\"/>\n"
            << "        <inertial pos=\"0 0 0\" mass=\"0.25\" diaginertia=\""
            << values({.0005 * scale * scale, .0008 * scale * scale, .0005 * scale * scale}) << "\"/>\n";
        const auto axial = "euler=\"" + values({kPi / 2, 0, 0}) + "\"";
        geom(out, "tire_contact_" + label, "cylinder", values({kWheelRadius, .025}),
             "0 0 0", "rubber", axial + " contype=\"1\" conaffinity=\"1\" friction=\"0.01 0.0001 0.0001\" group=\"3\" rgba=\"0 0 0 0\"");
        geom(out, "hub_" + label, "cylinder", values({.040 * scale, .028 * scale}), "0 0 0", "rubber", axial);
        for (int side : {-1, 1}) {
            geom(out, "rim_" + label + "_" + std::to_string(side), "cylinder",
                 values({.057 * scale, .004 * scale}), values({0, side * .028 * scale, 0}), "gold", axial);
            geom(out, "axle_" + label + "_" + std::to_string(side), "cylinder",
                 values({.014 * scale, .006 * scale}), values({0, side * .035 * scale, 0}), "silver", axial);
        }
        for (int i = 0; i < 12; ++i) {
            const double angle = i * 2 * kPi / 12;
            const double ax = -std::sin(angle) / std::sqrt(2.0);
            const double ay = handedness[w] / std::sqrt(2.0);
            const double az = std::cos(angle) / std::sqrt(2.0);
            const double qw = 1 + az, qx = -ay, qy = ax;
            const double norm = std::sqrt(qw * qw + qx * qx + qy * qy);
            geom(out, "roller_" + label + "_" + std::to_string(i), "ellipsoid",
                 values({.014 * scale, .014 * scale, .039 * scale}),
                 values({std::cos(angle) * .064 * scale, 0, std::sin(angle) * .064 * scale}),
                 "rubber", "quat=\"" + values({qw / norm, qx / norm, qy / norm, 0}) + "\"");
        }
        out << "        </body>\n      </body>\n";
    }
    out << R"(    </body>
  </worldbody>
  <actuator>
    <position name="motor_fl" joint="susp_fl" kp="900" kv="70" ctrllimited="true" ctrlrange="-0.20 0.20" forcelimited="true" forcerange="-220 220"/>
    <position name="motor_fr" joint="susp_fr" kp="900" kv="70" ctrllimited="true" ctrlrange="-0.20 0.20" forcelimited="true" forcerange="-220 220"/>
    <position name="motor_rl" joint="susp_rl" kp="900" kv="70" ctrllimited="true" ctrlrange="-0.20 0.20" forcelimited="true" forcerange="-220 220"/>
    <position name="motor_rr" joint="susp_rr" kp="900" kv="70" ctrllimited="true" ctrlrange="-0.20 0.20" forcelimited="true" forcerange="-220 220"/>
  </actuator>
  <contact>
    <exclude body1="chassis" body2="susp_fl"/>
    <exclude body1="chassis" body2="susp_fr"/>
    <exclude body1="chassis" body2="susp_rl"/>
    <exclude body1="chassis" body2="susp_rr"/>
  </contact>
  <sensor>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_accel" site="imu_site"/>
    <rangefinder name="range_front" site="range_front_site"/>
    <rangefinder name="range_down" site="range_down_site"/>
  </sensor>
</mujoco>
)";
    out.close();
    if (!out) { std::cerr << "Failed to write model\n"; return 1; }
    std::cout << "Generated " << path << "\n";
}
