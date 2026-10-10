// Independent forward-kinematics reconstruction and rotating-translation regression tests.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include "mecanum_model.h"

namespace {
void near(double actual, double expected, double tolerance = 1e-8) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error("Mismatch: " + std::to_string(actual) + " vs " + std::to_string(expected));
}
void checkWheels(const mecanum::Command& command, double yaw) {
    using namespace mecanum;
    const auto& w = command.wheels;
    const double bx = kWheelRadius * (w[0] + w[1] + w[2] + w[3]) / 4;
    const double by = kWheelRadius * (-w[0] + w[1] + w[2] - w[3]) / 4;
    const double spin = kWheelRadius * (-w[0] + w[1] - w[2] + w[3]) / (4 * kLeverArm);
    near(std::cos(yaw) * bx - std::sin(yaw) * by, command.world.x);
    near(std::sin(yaw) * bx + std::cos(yaw) * by, command.world.y);
    near(spin, command.world.yaw);
}

// Project the actual rotated primitives, rather than checking only wheel centres.
double extent(const mjModel* model, const mjData* data, int geom, int axis) {
    const auto* s = model->geom_size + 3 * geom;
    const auto* r = data->geom_xmat + 9 * geom + 3 * axis;
    switch (model->geom_type[geom]) {
        case mjGEOM_BOX:
            return std::abs(r[0]) * s[0] + std::abs(r[1]) * s[1] + std::abs(r[2]) * s[2];
        case mjGEOM_SPHERE: return s[0];
        case mjGEOM_CAPSULE: return s[0] + std::abs(r[2]) * s[1];
        case mjGEOM_CYLINDER:
            return s[0] * std::hypot(r[0], r[1]) + s[1] * std::abs(r[2]);
        case mjGEOM_ELLIPSOID:
            return std::sqrt(r[0] * r[0] * s[0] * s[0] + r[1] * r[1] * s[1] * s[1]
                             + r[2] * r[2] * s[2] * s[2]);
        default: throw std::runtime_error("Missing geometry bounds for model primitive");
    }
}

void checkFootprint(const mjModel* model, const mecanum::ModelJoints& joints) {
    using namespace mecanum;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model), mj_deleteData);
    double halfExtent[2] = {};
    double wheelRadius = 0;
    double wheelHalfWidth = 0;
    for (int degree = 0; degree < 360; ++degree) {
        for (int w = 0; w < 4; ++w) data->qpos[joints.qpos[w + 3]] = degree * kPi / 180;
        mj_forward(model, data.get());
        for (int g = 0; g < model->ngeom; ++g) {
            const int body = model->geom_bodyid[g];
            if (body == 0) continue; // The ground is not part of the chassis.
            const char* bodyName = mj_id2name(model, mjOBJ_BODY, body);
            const bool wheel = bodyName && std::string(bodyName).rfind("wheel_", 0) == 0;
            for (int axis = 0; axis < 3; ++axis) {
                const double reach = extent(model, data.get(), g, axis);
                if (axis < 2) {
                    const double bound = std::abs(data->geom_xpos[3 * g + axis]) + reach;
                    halfExtent[axis] = std::max(halfExtent[axis], bound);
                    if (bound > kFootprintHalfSize + 1e-6)
                        throw std::runtime_error(std::string("Chassis geometry exceeds 590 mm envelope: ") +
                                                 (mj_id2name(model, mjOBJ_GEOM, g) ?: "unnamed") +
                                                 " axis=" + std::to_string(axis) + " bound=" + std::to_string(bound));
                }
                if (wheel) {
                    const double relative = std::abs(data->geom_xpos[3 * g + axis]
                                                     - data->xpos[3 * body + axis]) + reach;
                    if (axis == 1) wheelHalfWidth = std::max(wheelHalfWidth, relative);
                    else wheelRadius = std::max(wheelRadius, relative);
                }
            }
        }
    }
    near(wheelRadius, kWheelRadius, 2e-6); // Includes rollers throughout a full turn.
    if (2 * kWheelHalfLength - 2 * wheelRadius < .020)
        throw std::runtime_error("Front/rear wheels have less than 20 mm clearance");
    if (kWheelHalfTrack - wheelHalfWidth - kDeckHalfWidth < .005)
        throw std::runtime_error("Deck needs at least 5 mm lateral wheel clearance");
    std::cout << "Geometry envelope: " << 2000 * halfExtent[0] << " x "
              << 2000 * halfExtent[1] << " mm; front/rear wheel gap: "
              << 2000 * (kWheelHalfLength - wheelRadius) << " mm.\n";
}
}  // namespace

int main() {
    try {
        using namespace mecanum;
        for (double yaw : {0., kPi / 4, kPi / 2, kPi, -kPi / 2}) {
            for (const auto direction : {std::array<double, 2>{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
                for (double spin : {-1., 0., 1.}) {
                    const auto cmd = solve(direction[0], direction[1], spin, yaw);
                    near(cmd.world.x, direction[0]); near(cmd.world.y, direction[1]);
                    checkWheels(cmd, yaw);
                }
            }
            const auto ref = solve(1, 0, 0, yaw, kPi / 2);
            near(ref.world.x, 0); near(ref.world.y, 1); checkWheels(ref, yaw);
        }
        const auto diagonal = keyboardCommand(1, 1, false, false, 1.7);
        near(std::hypot(diagonal.world.x, diagonal.world.y), std::abs(kLinearSpeed));
        near(keyboardCommand(0, 0, true, false, 0).world.yaw, kAngularSpeed);
        near(keyboardCommand(0, 0, false, true, 0).world.yaw, -kAngularSpeed);
        near(keyboardCommand(0, 0, true, true, 0).world.yaw, 0);

        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML("model/mecanum_wheel.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        const ModelJoints joints(model.get());
        // The generated scene includes climb obstacles; disable them for pure
        // kinematics regression so contact does not alter the expected path.
        for (const char* name : {"single_side_bridge_5cm", "step_20cm"}) {
            const int geom = mj_name2id(model.get(), mjOBJ_GEOM, name);
            if (geom >= 0) {
                model->geom_contype[geom] = 0;
                model->geom_conaffinity[geom] = 0;
            }
        }
        const char* wheelNames[] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
        const int xs[] = {1, 1, -1, -1}, ys[] = {1, -1, 1, -1};
        for (int i = 0; i < 4; ++i) {
            const int id = mj_name2id(model.get(), mjOBJ_BODY, wheelNames[i]);
            const int parent = model->body_parentid[id];
            near(model->body_pos[3 * parent] + model->body_pos[3 * id], xs[i] * kWheelHalfLength);
            near(model->body_pos[3 * parent + 1] + model->body_pos[3 * id + 1], ys[i] * kWheelHalfTrack);
        }
        for (const char* name : {"lower_deck"}) {
            const int id = mj_name2id(model.get(), mjOBJ_GEOM, name);
            near(model->geom_size[3 * id] * 2, 2 * kDeckHalfSize);
            near(model->geom_size[3 * id + 1] * 2, 2 * kDeckHalfWidth);
        }
        // The vertical rods sit outside the deck edge, leaving clearance instead of
        // relying on coincident collision geometry to behave like a drilled hole.
        for (const char* name : {"susp_fl", "susp_fr", "susp_rl", "susp_rr"}) {
            const int body = mj_name2id(model.get(), mjOBJ_BODY, name);
            const double rodY = std::abs(model->body_pos[3 * body + 1]);
            if (rodY - 0.028 <= kDeckHalfWidth)
                throw std::runtime_error("Suspension rod enters deck envelope without a clearance hole");
        }
        checkFootprint(model.get(), joints);
        for (double spin : {-kPi, kPi}) {
            std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
            for (int step = 0; step < 400; ++step) {
                const double yaw = data->qpos[joints.qpos[2]];
                const auto cmd = solve(1, 0, spin, yaw);
                checkWheels(cmd, yaw);
                joints.apply(data.get(), cmd);
                mj_step(model.get(), data.get());
            }
            near(data->qpos[joints.qpos[0]], 400 * model->opt.timestep, 1e-3);
            near(data->qpos[joints.qpos[1]], 0, 1e-3);
            near(data->qpos[joints.qpos[2]], spin * 400 * model->opt.timestep, 1e-3);
            const auto stopped = keyboardCommand(0, 0, false, false, data->qpos[joints.qpos[2]]);
            for (double wheel : stopped.wheels) near(wheel, 0);
        }
        std::cout << "PASS: fixed directions, inverse/forward kinematics, Q/E, diagonal speed, "
                     "simultaneous rotation/translation, model dimensions.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
