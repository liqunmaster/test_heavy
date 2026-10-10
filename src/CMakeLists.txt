add_executable(mecanum mecanum.cpp)
target_link_libraries(mecanum PRIVATE mujoco_simulate chassis_imu)

add_executable(mecanum_kernel mecanum_kernel.cpp)
target_link_libraries(mecanum_kernel PRIVATE chassis_imu)

add_executable(pendulum main.cpp)
target_link_libraries(pendulum PRIVATE MuJoCo::MuJoCo)

add_executable(model_generator model_generator.cpp)
if(MINGW)
    target_link_options(model_generator PRIVATE -static -static-libgcc -static-libstdc++)
endif()
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/model/mecanum_wheel.xml"
    COMMAND "$<TARGET_FILE:model_generator>" "${PROJECT_SOURCE_DIR}/model/mecanum_wheel.xml"
    DEPENDS model_generator "${CMAKE_CURRENT_SOURCE_DIR}/mecanum_config.h"
    VERBATIM)
add_custom_target(generate_model DEPENDS "${PROJECT_SOURCE_DIR}/model/mecanum_wheel.xml")
add_custom_target(copy_models ALL
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${PROJECT_SOURCE_DIR}/model" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/model"
    VERBATIM)
add_dependencies(copy_models generate_model)
foreach(target IN ITEMS mecanum mecanum_kernel pendulum)
    add_dependencies(${target} copy_models)
    if(MINGW)
        target_link_options(${target} PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:MuJoCo::MuJoCo>" "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
endforeach()
add_custom_command(TARGET mecanum POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "$<TARGET_FILE:GLFW::GLFW>" "$<TARGET_FILE_DIR:mecanum>"
    VERBATIM)

if(BUILD_TESTING)
    add_executable(suspension_tests "${PROJECT_SOURCE_DIR}/module/suspension_tests.cpp")
    target_link_libraries(suspension_tests PRIVATE chassis_imu)
    add_dependencies(suspension_tests copy_models)
    if(MINGW)
        target_link_options(suspension_tests PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
    add_custom_command(TARGET suspension_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:MuJoCo::MuJoCo>" "$<TARGET_FILE_DIR:suspension_tests>"
        VERBATIM)
    add_test(NAME suspension_leveling COMMAND suspension_tests)
    set_tests_properties(suspension_leveling PROPERTIES WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    add_executable(chassis_sensor_tests "${PROJECT_SOURCE_DIR}/module/imu_tests.cpp")
    target_link_libraries(chassis_sensor_tests PRIVATE chassis_imu)
    add_dependencies(chassis_sensor_tests copy_models)
    if(MINGW)
        target_link_options(chassis_sensor_tests PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
    add_custom_command(TARGET chassis_sensor_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:MuJoCo::MuJoCo>" "$<TARGET_FILE_DIR:chassis_sensor_tests>"
        VERBATIM)
    add_test(NAME chassis_sensors COMMAND chassis_sensor_tests)
    set_tests_properties(chassis_sensors PROPERTIES WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    add_executable(mecanum_tests mecanum_tests.cpp)
    target_link_libraries(mecanum_tests PRIVATE MuJoCo::MuJoCo)
    add_dependencies(mecanum_tests copy_models)
    if(MINGW)
        target_link_options(mecanum_tests PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
    add_custom_command(TARGET mecanum_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:MuJoCo::MuJoCo>" "$<TARGET_FILE_DIR:mecanum_tests>"
        VERBATIM)
    add_test(NAME mecanum_kinematics COMMAND mecanum_tests)
    set_tests_properties(mecanum_kinematics PROPERTIES WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()
