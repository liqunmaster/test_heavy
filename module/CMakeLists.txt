add_library(chassis_imu imu.cpp suspension.cpp)
target_include_directories(chassis_imu PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(chassis_imu PUBLIC MuJoCo::MuJoCo)
