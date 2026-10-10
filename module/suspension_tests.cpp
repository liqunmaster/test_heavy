#include "imu.hpp"
#include "suspension.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

int main() {
    try {
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML("model/mecanum_wheel.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("Cannot allocate simulation data");
        const int rollJoint = mj_name2id(model.get(), mjOBJ_JOINT, "roll");
        const int pitchJoint = mj_name2id(model.get(), mjOBJ_JOINT, "pitch");
        const int heaveJoint = mj_name2id(model.get(), mjOBJ_JOINT, "heave");
        data->qpos[model->jnt_qposadr[rollJoint]] = 0.04;
        data->qpos[model->jnt_qposadr[pitchJoint]] = -0.03;
        data->qpos[model->jnt_qposadr[heaveJoint]] = 0.01;
        mj_forward(model.get(), data.get());
        chassis::Imu imu(model.get());
        chassis::ActiveSuspension suspension(model.get());
        imu.update(data.get());
        const int steps = static_cast<int>(3.0 / model->opt.timestep);
        for (int i = 0; i < steps; ++i) {
            suspension.apply(data.get(), imu.reading());
            for (double target : suspension.status().targets)
                if (!std::isfinite(target) || std::abs(target) > 0.200001)
                    throw std::runtime_error("Motor target exceeded rod travel limit");
            mj_step(model.get(), data.get());
            imu.update(data.get());
        }
        const double roll = data->qpos[model->jnt_qposadr[rollJoint]];
        const double pitch = data->qpos[model->jnt_qposadr[pitchJoint]];
        const double height = suspension.status().height;
        std::cout << "Final roll=" << roll << " pitch=" << pitch << " height=" << height << '\n';
        if (!suspension.status().heightValid || !std::isfinite(roll) || !std::isfinite(pitch) ||
            std::abs(roll) > 0.15 || std::abs(pitch) > 0.15 || height < 0.10 || height > 0.50)
            throw std::runtime_error("Suspension output became unstable");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
