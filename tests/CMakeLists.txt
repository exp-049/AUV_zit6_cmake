cmake_minimum_required(VERSION 3.22)
project(zit6_host_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# 全局编译定义（与固件一致）
add_compile_definitions(EIGEN_NO_DEBUG NDEBUG)

# ============================================================================
# Google Test（使用系统安装的源码，避免网络下载）
set(GTEST_ROOT "/usr/src/googletest")
if(EXISTS "${GTEST_ROOT}/CMakeLists.txt")
  add_subdirectory("${GTEST_ROOT}" "${CMAKE_BINARY_DIR}/googletest" EXCLUDE_FROM_ALL)
else()
  # 回退：网络下载
  include(FetchContent)
  FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# ============================================================================
# Source paths (relative to project root)
# ============================================================================
set(ALGO_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Algorithm)
set(EIGEN_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Thirdparty)
set(COMPONENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component)

# ============================================================================
# Phase 1 — 纯算法测试（无硬件依赖）
# ============================================================================

# 算法源码（与固件共用，零修改）
set(ALGO_SOURCES
    ${ALGO_DIR}/PID_Controller.cpp
    ${ALGO_DIR}/KinematicProfile.cpp
)

add_executable(test_algo
    test_pid.cpp
    test_kinematic.cpp
    test_math_utils.cpp
    ${ALGO_SOURCES}
)

target_include_directories(test_algo PRIVATE
    ${ALGO_DIR}
    ${EIGEN_DIR}
)

target_link_libraries(test_algo
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_algo
    DISCOVERY_MODE PRE_TEST
)

# ============================================================================
# Phase 2 — 带 Mock 的模块测试
# ============================================================================

# 桩头文件目录（优先于 UserApp 路径，替换 FreeRTOS/main 等）
set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Peripherals are organized by device module. Keep all public driver include
# roots available to host tests that compile shared context sources.
set(PERIPHERAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/DepthSensor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/Pushrod/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/Presets/SelfCalcBoard/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/INS/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/MotionController/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/USBL/inc
)

# SafetyMonitor + 依赖的上下文对象
set(SM_SOURCES
    ${COMPONENT_DIR}/Safety/SafetyMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/MotionContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/SystemContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config/SystemConfig.cpp
)

add_executable(test_safety_monitor
    test_safety_monitor.cpp
    ${STUB_DIR}/app_ctx_stub.cpp
    ${STUB_DIR}/hal_tick_stub.cpp
    ${SM_SOURCES}
)

target_include_directories(test_safety_monitor PRIVATE
    ${STUB_DIR}              # 优先：FreeRTOS/task/main/Mock 桩
    ${ALGO_DIR}              # PID_Controller etc.
    ${EIGEN_DIR}             # Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common   # MotionContext, SystemContext
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config    # SystemConfig
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_safety_monitor
    GTest::gtest_main
)

gtest_discover_tests(test_safety_monitor
    DISCOVERY_MODE PRE_TEST
)

# ============================================================================
# Phase 3 — SetpointRouter + SoftWatchdog
# ============================================================================

set(COMPONENT_TEST_SOURCES
    ${COMPONENT_DIR}/Chassis/SetpointRouter.cpp
    ${COMPONENT_DIR}/Safety/SoftWatchdog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/MotionContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config/SystemConfig.cpp
)

add_executable(test_component
    test_setpoint_router.cpp
    test_soft_watchdog.cpp
    ${STUB_DIR}/app_ctx_stub.cpp
    ${STUB_DIR}/hal_tick_stub.cpp
    ${COMPONENT_TEST_SOURCES}
)

target_include_directories(test_component PRIVATE
    ${STUB_DIR}
    ${ALGO_DIR}
    ${EIGEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_component
    GTest::gtest_main
)

gtest_discover_tests(test_component
    DISCOVERY_MODE PRE_TEST
)

# ============================================================================
# Phase 4 — CascadeController + ConfigService
# ============================================================================

# CascadeController（核心控制链，需要算法源文件链接）
set(CASCADE_SOURCES
    ${COMPONENT_DIR}/Chassis/CascadeController.cpp
    ${ALGO_DIR}/PID_Controller.cpp
    ${ALGO_DIR}/KinematicProfile.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/MotionContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config/SystemConfig.cpp
)

add_executable(test_cascade
    test_cascade_controller.cpp
    ${STUB_DIR}/app_ctx_stub.cpp
    ${STUB_DIR}/hal_tick_stub.cpp
    ${CASCADE_SOURCES}
)

target_include_directories(test_cascade PRIVATE
    ${STUB_DIR}
    ${ALGO_DIR}
    ${EIGEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_cascade
    GTest::gtest_main
)

gtest_discover_tests(test_cascade
    DISCOVERY_MODE PRE_TEST
)

# ConfigService（仅测试静态辅助函数，不编译 ConfigService.cpp 因深度耦合硬件）
set(CONFIG_SERVICE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/MotionContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config/SystemConfig.cpp
)

add_executable(test_config_service
    test_config_service.cpp
    ${STUB_DIR}/app_ctx_stub.cpp
    ${STUB_DIR}/hal_tick_stub.cpp
    ${CONFIG_SERVICE_SOURCES}
)

target_include_directories(test_config_service PRIVATE
    ${STUB_DIR}
    ${ALGO_DIR}
    ${EIGEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Chassis
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_config_service
    GTest::gtest_main
)

gtest_discover_tests(test_config_service
    DISCOVERY_MODE PRE_TEST
)

# ============================================================================
# Phase 5 — ConfigService 完整测试（getParamsJson + updateParams）
# 编译 ConfigService.cpp + cJSON + 上下文桩
# ============================================================================

set(CONFIG_FULL_SOURCES
    ${COMPONENT_DIR}/ConfigService/ConfigService.cpp
    ${COMPONENT_DIR}/Safety/SoftWatchdog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common/SystemContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config/SystemConfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Thirdparty/cJSON/cJSON.c
)

add_executable(test_config_full
    test_config_service_full.cpp
    ${STUB_DIR}/app_ctx_stub.cpp
    ${STUB_DIR}/hal_tick_stub.cpp
    ${CONFIG_FULL_SOURCES}
)

target_include_directories(test_config_full PRIVATE
    ${STUB_DIR}
    ${ALGO_DIR}
    ${EIGEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Component/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals
    ${PERIPHERAL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Thirdparty/cJSON
)

target_link_libraries(test_config_full
    GTest::gtest_main
)

gtest_discover_tests(test_config_full
    DISCOVERY_MODE PRE_TEST
)

# Phase 6 — depth facade and independent pushrod facade
add_executable(test_depth_pushrod
    test_depth_pushrod_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/DepthSensor/src/Depth_Sensor_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/Pushrod/src/Pushrod_Driver.cpp
)

target_include_directories(test_depth_pushrod PRIVATE
    ${STUB_DIR}
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_depth_pushrod
    GTest::gtest_main
)

gtest_discover_tests(test_depth_pushrod
    DISCOVERY_MODE PRE_TEST
)

# PB8/PB7 non-PWM pushrod backend
add_executable(test_pushrod_gpio_backend
    test_pushrod_gpio_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/Pushrod/src/Pushrod_GPIO_Backend.cpp
)

target_include_directories(test_pushrod_gpio_backend PRIVATE
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_pushrod_gpio_backend
    GTest::gtest_main
)

gtest_discover_tests(test_pushrod_gpio_backend
    DISCOVERY_MODE PRE_TEST
)

# M14 UART backend and its device-specific ASCII protocol
add_executable(test_m14_backend
    test_m14_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/DepthSensor/src/M14_UART_Backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserApp/Peripherals/DepthSensor/src/Depth_Sensor_Driver.cpp
)

target_include_directories(test_m14_backend PRIVATE
    ${STUB_DIR}
    ${PERIPHERAL_INCLUDE_DIRS}
)

target_link_libraries(test_m14_backend
    GTest::gtest_main
)

gtest_discover_tests(test_m14_backend
    DISCOVERY_MODE PRE_TEST
)
