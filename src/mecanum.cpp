// Pure C++ controller using MuJoCo's official simulate UI.
// Tune speed and geometry in mecanum_config.h. No Python runtime is required.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "lodepng.h"
#include "mecanum_model.h"
#include "imu.hpp"
#include "suspension.hpp"

namespace {
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
enum KeyBits { Forward = 1, Backward = 2, Left = 4, Right = 8, CCW = 16, CW = 32,
               Raise = 64, Lower = 128 };

class ChassisAdapter : public mujoco::GlfwAdapter {
 public:
    mujoco::Simulate* sim = nullptr;
    std::atomic<unsigned> keys{0};
    std::string screenshot;

    void PollEvents() override {
        mujoco::GlfwAdapter::PollEvents();
        // GLFW input stays on the render thread; physics reads an atomic snapshot.
        GLFWwindow* window = glfwGetCurrentContext();
        if (!window || !glfwGetWindowAttrib(window, GLFW_FOCUSED) || editing() ||
            IsAltKeyPressed() || IsCtrlKeyPressed()) {
            keys.store(0);
            return;
        }
        auto held = [window](int key) { return glfwGetKey(window, key) == GLFW_PRESS; };
        unsigned value = 0;
        if (held(GLFW_KEY_W)) value |= Forward;
        if (held(GLFW_KEY_S)) value |= Backward;
        if (held(GLFW_KEY_A)) value |= Left;
        if (held(GLFW_KEY_D)) value |= Right;
        if (held(GLFW_KEY_Q)) value |= CCW;
        if (held(GLFW_KEY_E)) value |= CW;
        if (held(GLFW_KEY_UP)) value |= Raise;
        if (held(GLFW_KEY_DOWN)) value |= Lower;
        keys.store(value);
    }

    void SwapBuffers() override {
        if (sim && sim->m_ && ++frames_ > 20 && !screenshot.empty()) {
            const auto [width, height] = GetFramebufferSize();
            std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 4);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            const size_t stride = static_cast<size_t>(width) * 4;
            for (int row = 0; row < height / 2; ++row) {
                std::swap_ranges(pixels.begin() + row * stride, pixels.begin() + (row + 1) * stride,
                                 pixels.begin() + (height - row - 1) * stride);
            }
            const auto parent = std::filesystem::path(screenshot).parent_path();
            if (!parent.empty()) std::filesystem::create_directories(parent);
            const unsigned error = lodepng::encode(screenshot, pixels, width, height);
            if (error) std::cerr << "Screenshot error: " << lodepng_error_text(error) << '\n';
            else std::cout << "Saved " << screenshot << '\n';
            screenshot.clear();
        }
        mujoco::GlfwAdapter::SwapBuffers();
    }

 protected:
    void OnKey(int key, int scancode, int action) override {
        if (!editing() && !IsAltKeyPressed() && !IsCtrlKeyPressed()) {
            switch (key) {
                case GLFW_KEY_R:
                    if (action == GLFW_PRESS && sim) sim->pending_.reset = true;
                    return;
                case GLFW_KEY_W: case GLFW_KEY_S: case GLFW_KEY_A: case GLFW_KEY_D:
                case GLFW_KEY_UP: case GLFW_KEY_DOWN: case GLFW_KEY_LEFT: case GLFW_KEY_RIGHT:
                case GLFW_KEY_Q: case GLFW_KEY_E:
                case GLFW_KEY_LEFT_SHIFT: case GLFW_KEY_RIGHT_SHIFT:
                    return;  // Reserve chassis keys rather than triggering UI shortcuts.
            }
        }
        mujoco::GlfwAdapter::OnKey(key, scancode, action);
    }

 private:
    int frames_ = 0;
    bool editing() const { return sim && (sim->ui0.editsect || sim->ui1.editsect); }
};

void physics(mujoco::Simulate& sim, ChassisAdapter& input, ModelPtr& modelOwner, DataPtr& dataOwner,
             const std::string& filename, double smokeSeconds) {
    using Clock = std::chrono::steady_clock;
    mjModel* model = modelOwner.get();
    mjData* data = dataOwner.get();
    mecanum::ModelJoints joints(model);
    auto sensors = std::make_unique<chassis::Imu>(model);
    auto suspension = std::make_unique<chassis::ActiveSuspension>(model);
    sensors->update(data);
    double nextSensorReport = 0;
    const double referenceYaw = data->qpos[joints.qpos[2]];
    sim.Load(model, data, filename.c_str());
    {
        const mujoco::MutexLock lock(sim.mtx);
        sim.cam.type = mjCAMERA_TRACKING;
        sim.cam.trackbodyid = mj_name2id(model, mjOBJ_BODY, "chassis");
        sim.cam.distance = mecanum::kCameraDistance;
        sim.cam.azimuth = 180;
        sim.cam.elevation = -35;
    }
    const auto started = Clock::now();
    auto deadline = started;
    while (!sim.exitrequest.load()) {
        if (smokeSeconds > 0 && std::chrono::duration<double>(Clock::now() - started).count() >= smokeSeconds) {
            sim.exitrequest.store(1);
            break;
        }
        std::string reload;
        {
            const mujoco::MutexLock lock(sim.mtx);
            if (sim.droploadrequest.exchange(0)) reload = sim.dropfilename;
            else if (sim.uiloadrequest.exchange(0)) reload = sim.filename;
        }
        if (!reload.empty()) {
            char error[1024] = {};
            try {
                ModelPtr nextModel(mj_loadXML(reload.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
                if (!nextModel) throw std::runtime_error(error);
                mecanum::ModelJoints nextJoints(nextModel.get());
                auto nextSensors = std::make_unique<chassis::Imu>(nextModel.get());
                auto nextSuspension = std::make_unique<chassis::ActiveSuspension>(nextModel.get());
                DataPtr nextData(mj_makeData(nextModel.get()), mj_deleteData);
                if (!nextData) throw std::runtime_error("Cannot allocate reloaded data");
                mj_forward(nextModel.get(), nextData.get());
                nextSensors->update(nextData.get());
                sim.Load(nextModel.get(), nextData.get(), reload.c_str());
                const mujoco::MutexLock lock(sim.mtx);
                modelOwner.swap(nextModel);
                dataOwner.swap(nextData);
                model = modelOwner.get(); data = dataOwner.get(); joints = nextJoints;
                sensors = std::move(nextSensors);
                suspension = std::move(nextSuspension);
                nextSensorReport = 0;
                sim.cam.trackbodyid = mj_name2id(model, mjOBJ_BODY, "chassis");
                sim.load_error[0] = '\0';
            } catch (const std::exception& error) {
                const mujoco::MutexLock lock(sim.mtx);
                std::snprintf(sim.load_error, sizeof(sim.load_error), "%s", error.what());
            }
            deadline = Clock::now();
        }
        double interval = .005;
        {
            const mujoco::MutexLock lock(sim.mtx);
            const unsigned keys = sim.run ? input.keys.load() : 0;
            const double heightStep = model->opt.timestep * mecanum::kHeightCommandSpeed;
            if (keys & Raise) suspension->setTargetHeight(suspension->targetHeight() + heightStep);
            if (keys & Lower) suspension->setTargetHeight(suspension->targetHeight() - heightStep);
            const double forward = static_cast<int>(!!(keys & Forward)) - static_cast<int>(!!(keys & Backward));
            const double left = static_cast<int>(!!(keys & Left)) - static_cast<int>(!!(keys & Right));
            joints.apply(data, mecanum::keyboardCommand(forward, left, keys & CCW, keys & CW,
                                                        data->qpos[joints.qpos[2]], referenceYaw));
            if (!(keys & (CCW | CW))) {
                const double yawError = std::remainder(data->qpos[joints.qpos[2]] - referenceYaw,
                                                       2 * mecanum::kPi);
                data->qvel[joints.qvel[2]] = -mecanum::kYawHoldP * yawError
                                             -mecanum::kYawHoldD * data->qvel[joints.qvel[2]];
            }
            if (data->time < sensors->reading().time) {
                sensors->reset();
                suspension->reset();
                mj_forward(model, data);
                sensors->update(data);
                nextSensorReport = 0;
            }
            if (sim.run) {
                suspension->apply(data, sensors->reading());
                mj_step(model, data);
                sim.AddToHistory();
            } else {
                mj_forward(model, data);
            }
            const auto& reading = sensors->update(data);
            if (sim.run && reading.time >= nextSensorReport) {
                std::cout << "IMU r/p/y=" << reading.roll * 180 / mjPI << '/'
                          << reading.pitch * 180 / mjPI << '/'
                          << reading.yaw * 180 / mjPI << " deg | front=";
                if (reading.front.valid) std::cout << reading.front.metres << " m";
                else std::cout << "no return";
                std::cout << " | down=";
                if (reading.down.valid) std::cout << reading.down.metres << " m";
                else std::cout << "no return";
                std::cout << " | height=";
                if (suspension->status().heightValid) std::cout << suspension->status().height << " m";
                else std::cout << "no return";
                std::cout << " | target=" << suspension->targetHeight() << " m\n";
                nextSensorReport = reading.time + 0.5;
            }
            interval = model->opt.timestep * 100.0 / sim.percentRealTime[sim.real_time_index];
        }
        deadline += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(interval));
        if (Clock::now() > deadline + std::chrono::milliseconds(100)) deadline = Clock::now();
        while (!sim.exitrequest.load() && Clock::now() < deadline) {
            std::this_thread::sleep_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(10)));
        }
    }
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string filename = "model/mecanum_wheel.xml", screenshot;
        double smokeSeconds = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--smoke-test" && i + 1 < argc) smokeSeconds = std::stod(argv[++i]);
            else if (arg == "--screenshot" && i + 1 < argc) screenshot = argv[++i];
            else filename = arg;
        }
        if (mj_version() != mjVERSION_HEADER) throw std::runtime_error("MuJoCo headers/DLL version mismatch");
        char error[1024] = {};
        ModelPtr model(mj_loadXML(filename.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        DataPtr data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("Cannot allocate simulation data");
        const mecanum::ModelJoints joints(model.get());  // Validate before starting the UI/thread.
        mj_forward(model.get(), data.get());
        mjvCamera cam; mjv_defaultCamera(&cam);
        mjvOption opt; mjv_defaultOption(&opt);
        mjvPerturb pert; mjv_defaultPerturb(&pert);
        auto adapter = std::make_unique<ChassisAdapter>();
        auto* input = adapter.get();
        input->screenshot = screenshot;
        auto sim = std::make_unique<mujoco::Simulate>(std::move(adapter), &cam, &opt, &pert, false);
        input->sim = sim.get();
        std::cout << "MuJoCo " << mj_versionString() << " | C++ official viewer\n"
                  << "W/S: forward/back; A/D: strafe; Q/E: rotate; Up/Down: raise/lower; R: reset.\n"
                  << "Height target: 0.12..0.50 m.\n" << std::flush;
        std::thread worker(physics, std::ref(*sim), std::ref(*input), std::ref(model), std::ref(data),
                           std::cref(filename), smokeSeconds);
        sim->RenderLoop();
        worker.join();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
