cmake_minimum_required(VERSION 3.20)
project(MecanumSimulation VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

include(CTest)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
add_subdirectory(thirdparty)
add_subdirectory(module)
add_subdirectory(src)

add_custom_target(viewer
    COMMAND "$<TARGET_FILE:mecanum>"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:mecanum>"
    DEPENDS mecanum
    USES_TERMINAL
    COMMENT "Launch the C++ official MuJoCo viewer")
