#include <mujoco/mujoco.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

static const double kSimDuration = 3.0;

mjModel* LoadModel(const char* xmlPath)
{
    char error[1024] = {0};
    mjModel* model = mj_loadXML(xmlPath, NULL, error, sizeof(error));
    if (!model)
    {
        std::fprintf(stderr, "[Error] 加载模型失败: %s\n", error);
    }
    return model;
}

mjData* MakeData(const mjModel* model)
{
    if (!model)
    {
        return NULL;
    }
    mjData* data = mj_makeData(model);
    if (!data)
    {
        std::fprintf(stderr, "[Error] 创建仿真数据失败\n");
    }
    return data;
}

double RunSimulation(const mjModel* model, mjData* data, int nstep)
{
    if (!model || !data)
    {
        return 0.0;
    }
    double peakKinetic = 0.0;
    for (int i = 0; i < nstep; ++i)
    {
        mj_step(model, data);

        mj_energyVel(model, data);
        if (data->energy[1] > peakKinetic)
        {
            peakKinetic = data->energy[1];
        }
    }
    mj_forward(model, data);
    mj_energyVel(model, data);
    return peakKinetic;
}

void PrintSummary(const mjModel* model, const mjData* data, double peakKinetic)
{
    const double kRad2Deg = 180.0 / 3.14159265358979;
    std::printf("=== MuJoCo 仿真结果 ===\n");
    std::printf("模型名称   : %s\n", model->names);
    std::printf("仿真时长   : %.3f s\n", data->time);
    std::printf("关节数量   : %lld\n", model->nq);
    std::printf("关节位置(°): ");
    for (int i = 0; i < model->nq; ++i)
    {
        std::printf("%.4f ", data->qpos[i] * kRad2Deg);
    }
    std::printf("\n关节速度(°/s): ");
    for (int i = 0; i < model->nv; ++i)
    {
        std::printf("%.4f ", data->qvel[i] * kRad2Deg);
    }
    std::printf("\n当前动能   : %.6f J\n", data->energy[1]);
    std::printf("峰值动能   : %.6f J\n", peakKinetic);
    std::printf("末端位置   : %.4f %.4f %.4f\n", data->site_xpos[0 * 3 + 0], data->site_xpos[0 * 3 + 1], data->site_xpos[0 * 3 + 2]);
}

int main(int argc, char* argv[])
{
    const char* xmlPath = (argc > 1) ? argv[1] : "model/pendulum.xml";
    mjModel* model = LoadModel(xmlPath);
    if (!model)
    {
        return 1;
    }
    mjData* data = MakeData(model);
    if (!data)
    {
        mj_deleteModel(model);
        return 1;
    }

    if (model->nkey > 0)
    {
        mj_resetDataKeyframe(model, data, 0);
    }
    const int nstep = static_cast<int>(std::ceil(kSimDuration / model->opt.timestep));
    const double peakKinetic = RunSimulation(model, data, nstep);
    PrintSummary(model, data, peakKinetic);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
