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

# Add STM32CubeMX generated sources (HAL/Core/Drivers/Middlewares)
add_subdirectory(Peripherals/HAL)

# UserApp modules (each handles its own sources and include paths)
add_subdirectory(Algorithm)
add_subdirectory(Application)
add_subdirectory(Common)
add_subdirectory(Config)
add_subdirectory(Component)
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
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
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

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
    # Add user defined libraries
)

add_custom_target(microros_build
    COMMAND docker pull microros/micro_ros_static_library_builder:humble
    COMMAND docker run --rm --network host -v ${CMAKE_CURRENT_SOURCE_DIR}:/project --env MICROROS_LIBRARY_FOLDER=Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide --env http_proxy=http://127.0.0.1:7897 --env https_proxy=http://127.0.0.1:7897 --env all_proxy=socks5://127.0.0.1:7897 microros/micro_ros_static_library_builder:humble
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building micro-ROS static library via Docker with Proxy"
)
