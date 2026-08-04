cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Ensure C++ settings so .cpp files are compiled as C++ (avoid accidental C translation units)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Firmware application mode is deliberately a configure-time choice. Debug
# modes do not coexist with the NORMAL control, micro-ROS, or monitor tasks.
set(AUV_APP_MODE "NORMAL" CACHE STRING "Firmware application mode")
set_property(CACHE AUV_APP_MODE PROPERTY STRINGS NORMAL MS5837_CAL_DEBUG USBL_DEBUG INS_DEBUG MOTION_DEBUG)
set(_auv_app_modes NORMAL MS5837_CAL_DEBUG USBL_DEBUG INS_DEBUG MOTION_DEBUG)
if(NOT AUV_APP_MODE IN_LIST _auv_app_modes)
    message(FATAL_ERROR "Unsupported AUV_APP_MODE='${AUV_APP_MODE}'")
endif()

# Generate config header from JSON (run on build when config.json changes)
set(GEN_CONFIG_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/gen_config.py)
set(CONFIG_JSON ${CMAKE_CURRENT_SOURCE_DIR}/Config/config.json)
set(GEN_CONFIG_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Config)
set(GEN_CONFIG_HPP ${GEN_CONFIG_OUT_DIR}/SystemConfig.hpp)
set(GEN_CONFIG_CPP ${GEN_CONFIG_OUT_DIR}/SystemConfig.cpp)

set(GEN_CONFIG_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/templates/SystemConfig.hpp.j2)

add_custom_command(
    OUTPUT ${GEN_CONFIG_HPP} ${GEN_CONFIG_CPP}
    COMMAND python3 ${GEN_CONFIG_SCRIPT} ${CONFIG_JSON} ${GEN_CONFIG_OUT_DIR}
    DEPENDS ${GEN_CONFIG_SCRIPT} ${CONFIG_JSON} ${GEN_CONFIG_TEMPLATE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating SystemConfig from config.json"
    VERBATIM
)
add_custom_target(gen_system_config DEPENDS ${GEN_CONFIG_HPP} ${GEN_CONFIG_CPP})

# Set the project name
set(CMAKE_PROJECT_NAME AUV_zit6)

# ============================================================================
# 静态分析集成
# ============================================================================
option(AUV_ENABLE_CLANG_TIDY "在编译时运行 clang-tidy（会显著增加编译时间）" OFF)

if(AUV_ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_BIN clang-tidy)
    if(CLANG_TIDY_BIN)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_BIN}")
        message(STATUS "clang-tidy 已启用: ${CLANG_TIDY_BIN}")
    else()
        message(WARNING "AUV_ENABLE_CLANG_TIDY=ON 但未找到 clang-tidy")
    endif()
endif()

# 自定义目标：运行完整静态分析（cppcheck + clang-tidy）
add_custom_target(static-analysis
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/run_static_analysis.sh
        -b ${CMAKE_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "运行静态分析 (cppcheck + clang-tidy)"
    USES_TERMINAL
)

# 自定义目标：仅 cppcheck（快速）
add_custom_target(static-analysis-fast
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/run_static_analysis.sh
        -b ${CMAKE_BINARY_DIR} -f
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "运行快速静态分析 (仅 cppcheck)"
    USES_TERMINAL
)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
# Use literal project name so colcon can identify the package correctly
project(AUV_zit6 LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
add_dependencies(${CMAKE_PROJECT_NAME} gen_system_config)

if(AUV_APP_MODE STREQUAL "NORMAL")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=0)
elseif(AUV_APP_MODE STREQUAL "MS5837_CAL_DEBUG")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=1)
elseif(AUV_APP_MODE STREQUAL "USBL_DEBUG")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=2)
elseif(AUV_APP_MODE STREQUAL "INS_DEBUG")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=3)
elseif(AUV_APP_MODE STREQUAL "MOTION_DEBUG")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=4)
endif()

# Add STM32CubeMX generated sources (HAL/Core/Drivers/Middlewares)
add_subdirectory(Peripherals/HAL)

# UserApp modules (each handles its own sources and include paths)
add_subdirectory(Algorithm)
add_subdirectory(Application)
add_subdirectory(Common)
add_subdirectory(Config)
add_subdirectory(Component)
add_subdirectory(Debug)
add_subdirectory(Protocol)
add_subdirectory(Peripherals)
add_subdirectory(Porting)
add_subdirectory(Thirdparty)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Thirdparty/micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
    Thirdparty/micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
    Thirdparty/micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
  Peripherals/HAL/Core/Src/main.c
  Peripherals/HAL/Core/Src/gpio.c
  Peripherals/HAL/Core/Src/i2c.c
  Peripherals/HAL/Core/Src/dma.c
  Peripherals/HAL/Core/Src/freertos.c
  Peripherals/HAL/Core/Src/usart.c
  Peripherals/HAL/Core/Src/stm32h7xx_it.c
  Peripherals/HAL/Core/Src/stm32h7xx_hal_msp.c
  Peripherals/HAL/Core/Src/stm32h7xx_hal_timebase_tim.c
    Thirdparty/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c
    Thirdparty/RTT/RTT/SEGGER_RTT.c
    Thirdparty/RTT/RTT/SEGGER_RTT_printf.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
    Thirdparty/RTT/RTT
    Thirdparty/RTT/Config
)

# Add project symbols (macros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")
add_compile_options(-Wno-error -Wno-error=fpermissive -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast -Wno-unused-result -fpermissive)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    DATA_IN_D2_SRAM

    # Eigen 矩阵库配置：禁用运行时断言和调试检查
    EIGEN_NO_DEBUG
    NDEBUG

    # HITL 仿真模式：Debug 下启用，Release 下注释即可剔除仿真代码
    $<$<CONFIG:Debug>:AUV_SIMULATION_ENABLE>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# micro-ROS is generated by the Docker builder and is intentionally ignored by
# git.  Model the archive as a real build output so Ninja can generate it when
# a fresh checkout is built, instead of treating a missing absolute path as an
# external file with no rule.
set(MICROROS_LIBRARY_FOLDER
    "Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide")
set(MICROROS_LIBRARY
    "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIBRARY_FOLDER}/libmicroros/libmicroros.a")
set(MICROROS_GENERATOR_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIBRARY_FOLDER}/library_generation")
set(MICROROS_EXTRA_PACKAGES_DIR "${MICROROS_GENERATOR_DIR}/extra_packages")
set(MICROROS_EXTRA_REPOS_TEMPLATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/extra_packages.repos")

find_program(DOCKER_EXECUTABLE docker)
if(NOT EXISTS "${MICROROS_LIBRARY}" AND NOT DOCKER_EXECUTABLE)
    message(FATAL_ERROR
        "micro-ROS library is missing and Docker was not found. "
        "Install Docker or generate ${MICROROS_LIBRARY} manually.")
endif()

file(GLOB_RECURSE MICRO_ROS_INTERFACE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/../zit6_interfaces/*")

add_custom_command(
    OUTPUT "${MICROROS_LIBRARY}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${MICROROS_EXTRA_PACKAGES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${MICROROS_EXTRA_PACKAGES_DIR}/zit6_interfaces"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/../zit6_interfaces"
            "${MICROROS_EXTRA_PACKAGES_DIR}/zit6_interfaces"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MICROROS_EXTRA_REPOS_TEMPLATE}"
            "${MICROROS_EXTRA_PACKAGES_DIR}/extra_packages.repos"
    COMMAND "${DOCKER_EXECUTABLE}" pull microros/micro_ros_static_library_builder:humble
    COMMAND "${DOCKER_EXECUTABLE}" run --rm --network host
            -v "${CMAKE_CURRENT_SOURCE_DIR}:/project"
            --env MICROROS_LIBRARY_FOLDER=${MICROROS_LIBRARY_FOLDER}
            --env http_proxy=http://127.0.0.1:7897
            --env https_proxy=http://127.0.0.1:7897
            --env all_proxy=socks5://127.0.0.1:7897
            microros/micro_ros_static_library_builder:humble
    DEPENDS
        "${MICROROS_GENERATOR_DIR}/library_generation.sh"
        "${MICROROS_GENERATOR_DIR}/colcon.meta"
        "${MICROROS_GENERATOR_DIR}/colcon-embeddedrtps.meta"
        "${MICROROS_GENERATOR_DIR}/toolchain.cmake"
        "${MICROROS_EXTRA_REPOS_TEMPLATE}"
        ${MICRO_ROS_INTERFACE_FILES}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating micro-ROS static library via Docker"
    USES_TERMINAL
    VERBATIM
)

add_custom_target(microros_build DEPENDS "${MICROROS_LIBRARY}")
add_dependencies(${CMAKE_PROJECT_NAME} microros_build)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    "${MICROROS_LIBRARY}"
    # Add user defined libraries
)
