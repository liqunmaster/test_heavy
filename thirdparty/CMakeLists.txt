# Override these cache paths when using a different local MuJoCo/GLFW package.
set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/mujoco" CACHE PATH "MuJoCo package root")
set(GLFW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/glfw" CACHE PATH "GLFW package root")

find_path(MUJOCO_INCLUDE_DIR NAMES mujoco/mujoco.h
    PATHS "${MUJOCO_ROOT}/include/mujoco" "${MUJOCO_ROOT}/include"
    NO_DEFAULT_PATH REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco
    PATHS "${MUJOCO_ROOT}/bin" "${MUJOCO_ROOT}/lib"
    NO_DEFAULT_PATH REQUIRED)
find_file(MUJOCO_DLL NAMES mujoco.dll PATHS "${MUJOCO_ROOT}/bin"
    NO_DEFAULT_PATH REQUIRED)
find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h PATHS "${GLFW_ROOT}/include"
    NO_DEFAULT_PATH REQUIRED)
find_library(GLFW_LIBRARY NAMES glfw3dll libglfw3dll PATHS "${GLFW_ROOT}/lib"
    NO_DEFAULT_PATH REQUIRED)
find_file(GLFW_DLL NAMES glfw3.dll PATHS "${GLFW_ROOT}/lib" "${GLFW_ROOT}/bin"
    NO_DEFAULT_PATH REQUIRED)

add_library(MuJoCo::MuJoCo SHARED IMPORTED GLOBAL)
set_target_properties(MuJoCo::MuJoCo PROPERTIES
    IMPORTED_IMPLIB "${MUJOCO_LIBRARY}"
    IMPORTED_LOCATION "${MUJOCO_DLL}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
add_library(GLFW::GLFW SHARED IMPORTED GLOBAL)
set_target_properties(GLFW::GLFW PROPERTIES
    IMPORTED_IMPLIB "${GLFW_LIBRARY}"
    IMPORTED_LOCATION "${GLFW_DLL}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLFW_INCLUDE_DIR}")

# Official MuJoCo 3.15.0 simulate UI (vendored sources; no Python dependency).
add_library(lodepng STATIC lodepng/lodepng.cpp)
target_include_directories(lodepng PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lodepng")
add_library(mujoco_simulate STATIC
    mujoco_simulate/simulate.cc
    mujoco_simulate/glfw_adapter.cc
    mujoco_simulate/glfw_dispatch.cc
    mujoco_simulate/platform_ui_adapter.cc)
target_include_directories(mujoco_simulate PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/mujoco_simulate")
target_link_libraries(mujoco_simulate PUBLIC MuJoCo::MuJoCo GLFW::GLFW OpenGL::GL Threads::Threads lodepng)

