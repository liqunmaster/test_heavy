// Headless C++ demonstration using the same fixed-reference control as the viewer.
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include "mecanum_model.h"
#include "imu.hpp"
#include "suspension.hpp"

int main(int argc, char** argv) {
    try {
        const char* path = argc > 1 ? argv[1] : "model/mecanum_wheel.xml";
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(path, nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("Cannot allocate simulation data");
        const mecanum::ModelJoints joints(model.get());
        chassis::Imu sensors(model.get());
        chassis::ActiveSuspension suspension(model.get());
        mj_forward(model.get(), data.get());
        sensors.update(data.get());
        const double initialYaw = data->qpos[joints.qpos[2]];
        // duration(s), fixed-reference vx(m/s), vy(m/s), yaw rate(rad/s)
        const std::array<double, 4> segments[] = {
            {2, 1, 0, 0}, {2, 0, 1, 0}, {2, 0, 0, 1},
            {2, std::sqrt(.5), std::sqrt(.5), 0}, {2, 1, 0, .5}
        };
        std::filesystem::create_directories("outputs");
        std::ofstream out("outputs/mecanum_trajectory.csv");
        if (!out) throw std::runtime_error("Cannot write trajectory CSV");
        out << "segment,time,x,y,yaw_deg,wfl,wfr,wrl,wrr,imu_roll_deg,imu_pitch_deg,imu_yaw_deg,range_front_m,range_down_m,height_m,motor_fl_rad,motor_fr_rad,motor_rl_rad,motor_rr_rad\n";
        int index = 0;
        for (const auto& segment : segments) {
            const int steps = static_cast<int>(std::round(segment[0] / model->opt.timestep));
            for (int step = 0; step < steps; ++step) {
                joints.apply(data.get(), mecanum::solve(segment[1], segment[2], segment[3],
                             data->qpos[joints.qpos[2]], initialYaw));
                if (segment[3] == 0) {
                    const double yawError = std::remainder(data->qpos[joints.qpos[2]] - initialYaw,
                                                           2 * mecanum::kPi);
                    data->qvel[joints.qvel[2]] = -mecanum::kYawHoldP * yawError
                                                 -mecanum::kYawHoldD * data->qvel[joints.qvel[2]];
                }
                suspension.apply(data.get(), sensors.reading());
                mj_step(model.get(), data.get());
                sensors.update(data.get());
            }
            ++index;
            out << index << ',' << data->time;
            for (int i = 0; i < 3; ++i)
                out << ',' << data->qpos[joints.qpos[i]] * (i == 2 ? 180 / mecanum::kPi : 1);
            for (int i = 3; i < 7; ++i) out << ',' << data->qvel[joints.qvel[i]];
            const auto& reading = sensors.reading();
            out << ',' << reading.roll * 180 / mecanum::kPi
                << ',' << reading.pitch * 180 / mecanum::kPi
                << ',' << reading.yaw * 180 / mecanum::kPi << ',';
            if (reading.front.valid) out << reading.front.metres;
            out << ',';
            if (reading.down.valid) out << reading.down.metres;
            out << ',';
            if (suspension.status().heightValid) out << suspension.status().height;
            for (double target : suspension.status().targets) out << ',' << target;
            out << '\n';
            std::cout << "Segment " << index << " | time=" << data->time
                      << " | x=" << data->qpos[joints.qpos[0]] << " y=" << data->qpos[joints.qpos[1]]
                      << " yaw=" << data->qpos[joints.qpos[2]] * 180 / mecanum::kPi << " deg\n";
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
