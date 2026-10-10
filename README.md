# 麦克纳姆底盘仿真（纯 C++）

使用 MinGW + CMake + MuJoCo 3.15.0 + GLFW。构建、模型生成、官方查看器和运动控制均不依赖 Python。

在 VS Code 按 `Ctrl+Shift+B`：配置 CMake、构建工程，然后启动带官方红色控制面板的 C++ 查看器。

```powershell
cmake --preset mingw-debug
cmake --build --preset mingw-debug
ctest --test-dir build/cmake --output-on-failure
```

程序在 `build/cmake/bin`。该目录内有 DLL 和 model 子目录，可以从该目录直接启动 `mecanum.exe`。

## 按键

先点击查看器窗口。上方向键抬高底盘，下方向键降低底盘，R 复位；底盘目标高度由下向激光测距闭环控制。
释放按键或切换到其他窗口后停止移动。编辑官方面板的输入框时不会控制小车。

查看器中 W/S 前后移动、A/D 横移、Q/E 旋转；上/下方向键只调节底盘高度，R 复位。

## 参数

统一修改 `src/mecanum_config.h` 并重新构建：

- `kLinearSpeed`：平移速度（m/s）。
- `kAngularSpeed`：旋转速度（rad/s）；`kPi` 等于 180°/s。
- `kFootprintHalfSize`：整车包络半边长；0.325 对应 650 × 650 mm，包含轮子和车身附件。
- `kDeckHalfSize`、`kDeckHalfWidth`：车身板半长、半宽；当前车身板为 590 × 300 mm，为两侧轮子留出空间。
- `kWheelRadius`：轮半径；0.152 对应 152 mm。
- `kWheelHalfLength`、`kWheelHalfTrack`：轮心 X、Y 坐标的绝对值。
  当前为 0.17、0.24 m，四轮布置为 `(±17 cm, ±24 cm)`，以底盘中心为原点。

以 +X 为前方、+Y 为左方、+Z 为上方；轮序为 FL、FR、RL、RR。轮距 0.48 m，轴距 0.34 m，运动学旋转力臂为 0.41 m。
轮半径保留 152 mm：前后两轮名义直径合计已达 608 mm，因此整车范围调整为 650 × 650 mm。
尺寸测试会检查所有车身几何（含斜滚子和附件）的实际投影包络，以及轮子旋转时的外径和前后轮间隙。

`src/model_generator.cpp` 根据上述尺寸生成 `model/mecanum_wheel.xml`；背景、材质和装饰在这个 C++ 文件里调整。
`src/mecanum_control.h` 是旋转矩阵与麦轮运动学；`src/mecanum.cpp` 是官方界面与按键控制。
`src/mecanum_kernel.cpp` 为无窗口演示，生成 `outputs/mecanum_trajectory.csv`。

## 底盘传感器

`module/imu.cpp` 读取底盘上的陀螺仪、加速度计和两路激光测距。姿态用陀螺仪积分，并用重力方向校正横滚、俯仰；偏航以启动时的方向为零点，长时间运行会有积分漂移。读数为弧度、rad/s、m/s² 和米。未命中或无效测距用 `valid=false` 表示，CSV 中留空。

新模型需定义四个传感器：`imu_gyro` (`gyro`)、`imu_accel` (`accelerometer`)、`range_front` 和 `range_down` (均为 `rangefinder`)。它们的 site 都固定在底盘 body 上；IMU site 的本地 +X 为车头、+Y 为左、+Z 为上。MuJoCo 测距沿 site 本地 +Z 发射，因此前向 site 的 +Z 指向车头，下向 site 的 +Z 指向地面。`chassis::SensorNames` 可在代码中更改名称。重新导入模型时会重新绑定传感器。

当前主动悬架是四套独立竖直连杆：每个麦轮上方都有一根长支柱/减振杆和一个位置电机，杆件沿 Z 方向上下伸缩。上方向键增大下向激光的目标距离，也就是抬高底盘；下方向键减小目标距离，也就是降低底盘。目标距离被限制在 `0.12~0.50 m`，松开按键后保持目标值。四个电机在高度控制基础上叠加 IMU 的 roll/pitch 差动补偿。

上/下键升降速度由 `src/mecanum_config.h` 的 `kHeightCommandSpeed` 控制，单位是 m/s。

当前麦轮半径为 `0.076 m`。竖杆截面和轮顶之间保留了明确间隙，杆件不会插入轮子；杆件允许穿过固定底盘的包络，并且不参与底盘碰撞。

当前整车外包络按 `590 mm x 590 mm` 控制，麦轮横向轮心调整为 `+/-210 mm`，缩小后的轮胎外缘仍留在包络内。

每个麦轮现在由两根平行细长矩形杆夹持，轮子位于两杆之间；杆底部通过横向轮轴座连接，杆与轮胎外缘保留间隙。

场景中还加入了两个测试障碍：右侧单轮路线上的 5 cm 高、80 cm 长单边桥，以及横跨车宽的 20 cm 高、40 cm 长台阶；两者中心间距为 1.5 m。障碍使用固定世界碰撞几何、接触余量和更高求解迭代，W/S 可用于驶向障碍测试。

竖杆目前放在底盘四角外侧，和甲板边缘留有间隙，不与甲板穿插；杆件与导向套也被排除在碰撞对之外，避免接触求解产生弹跳。后续如果真实结构需要让杆子穿过甲板，应把甲板拆成带四个圆孔的分块几何，并保持孔壁与杆件之间的间隙。

模型仍为运动学演示：直接设置底盘和轮子的速度，斜滚子为视觉几何，未实现滚子独立转动和接触摩擦动力学。

官方查看器源码及版本说明在 `thirdparty/mujoco_simulate/UPSTREAM.md`。
