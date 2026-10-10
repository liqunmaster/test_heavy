#pragma once
#include <array>
#include <stdexcept>
#include <mujoco/mujoco.h>
#include "mecanum_control.h"

namespace mecanum {
struct ModelJoints {
    std::array<int, 7> qpos;
    std::array<int, 7> qvel;
    explicit ModelJoints(const mjModel* model) {
        const char* names[] = {"slide_x", "slide_y", "yaw", "w_fl", "w_fr", "w_rl", "w_rr"};
        for (int i = 0; i < 7; ++i) {
            const int id = mj_name2id(model, mjOBJ_JOINT, names[i]);
            if (id < 0) throw std::runtime_error(std::string("Missing joint: ") + names[i]);
            qpos[i] = model->jnt_qposadr[id];
            qvel[i] = model->jnt_dofadr[id];
        }
    }
    void apply(mjData* data, const Command& command) const {
        data->qvel[qvel[0]] = command.world.x;
        data->qvel[qvel[1]] = command.world.y;
        data->qvel[qvel[2]] = command.world.yaw;
        for (int i = 0; i < 4; ++i) data->qvel[qvel[3 + i]] = command.wheels[i];
    }
};
}  // namespace mecanum
