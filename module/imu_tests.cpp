#include "imu.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
void near(double actual, double expected, double tolerance = 1e-5) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error("Expected " + std::to_string(expected) +
                                 ", got " + std::to_string(actual));
}
}

int main() {
    try {
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML("model/mecanum_wheel.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("Cannot allocate simulation data");
        chassis::Imu imu(model.get());
        mj_forward(model.get(), data.get());
        const auto& initial = imu.update(data.get());
        if (!initial.down.valid || (initial.front.valid && initial.front.metres <= 0))
            throw std::runtime_error("Incorrect rangefinder hit state");
        near(initial.down.metres, 0.077, 1e-4);
        near(initial.roll, 0);
        near(initial.pitch, 0);

        const int gyro = mj_name2id(model.get(), mjOBJ_SENSOR, "imu_gyro");
        const int address = model->sensor_adr[gyro];
        for (int i = 1; i <= 100; ++i) {
            data->time = i * 0.01;
            data->sensordata[address + 2] = 1;
            imu.update(data.get());
        }
        near(imu.reading().yaw, 1, 1e-4);
        data->sensordata[address + 2] = 0;
        const int accel = mj_name2id(model.get(), mjOBJ_SENSOR, "imu_accel");
        const int accelAddress = model->sensor_adr[accel];
        data->sensordata[accelAddress + 1] = 9.81 * std::sin(0.2);
        data->sensordata[accelAddress + 2] = 9.81 * std::cos(0.2);
        for (int i = 101; i <= 200; ++i) {
            data->time = i * 0.01;
            imu.update(data.get());
        }
        near(imu.reading().roll, 0.2 * (1 - std::exp(-2)), 1e-3);
        data->time = 0;
        mj_forward(model.get(), data.get());
        near(imu.update(data.get()).yaw, 0);
        if (!imu.reading().down.valid) throw std::runtime_error("Downward range lost after reset");
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> scene(
            mj_loadXML("model/sensor_test.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!scene) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> sceneData(
            mj_makeData(scene.get()), mj_deleteData);
        chassis::Imu sceneImu(scene.get());
        mj_forward(scene.get(), sceneData.get());
        const auto& hit = sceneImu.update(sceneData.get());
        if (!hit.front.valid || !hit.down.valid)
            throw std::runtime_error("Expected front wall and ground returns");
        near(hit.front.metres, 0.57, 1e-4);
        near(hit.down.metres, 0.153, 1e-4);
        std::cout << "PASS: gravity tilt, gyro yaw, reset, front wall/no return, ground distance\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
