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
set(GEN_CONFIG_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_config.py)
set(CONFIG_JSON ${CMAKE_CURRENT_SOURCE_DIR}/config.json)
set(GEN_CONFIG_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/UserApp/Common)
set(GEN_CONFIG_HPP ${GEN_CONFIG_OUT_DIR}/SystemConfig.hpp)
set(GEN_CONFIG_CPP ${GEN_CONFIG_OUT_DIR}/SystemConfig.cpp)

add_custom_command(
    OUTPUT ${GEN_CONFIG_HPP} ${GEN_CONFIG_CPP}
    COMMAND python3 ${GEN_CONFIG_SCRIPT} ${CONFIG_JSON} ${GEN_CONFIG_OUT_DIR}
    DEPENDS ${GEN_CONFIG_SCRIPT} ${CONFIG_JSON}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating SystemConfig from config.json"
    VERBATIM
)
add_custom_target(gen_system_config DEPENDS ${GEN_CONFIG_HPP} ${GEN_CONFIG_CPP})

# Set the project name
set(CMAKE_PROJECT_NAME AUV_zit6)

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

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
  Core/Src/main.c
  Core/Src/gpio.c
  Core/Src/i2c.c
  Core/Src/dma.c
  Core/Src/freertos.c
  Core/Src/usart.c
  Core/Src/stm32h7xx_it.c
  Core/Src/stm32h7xx_hal_msp.c
  Core/Src/stm32h7xx_hal_timebase_tim.c
    UserApp/Application/AppMain.cpp
    UserApp/Application/ControlTask.cpp
    UserApp/Application/MicroRosTask.cpp
    UserApp/Application/MicroRosTransport.cpp
    UserApp/Application/MicroRosPublisher.cpp
    UserApp/Application/MicroRosSubscriber.cpp
    UserApp/Application/MicroRosService.cpp
    UserApp/Application/IICTask.cpp
    UserApp/Common/MotionContext.cpp
    UserApp/Common/SystemContext.cpp
    UserApp/Common/SystemConfig.cpp
    UserApp/Common/ConfigService.cpp
    UserApp/Peripherals/SoftWatchdog.cpp
    UserApp/Peripherals/INS_Driver.cpp
    UserApp/Peripherals/MotionController_Driver.cpp
    UserApp/Peripherals/MS5837_Class.cpp
    UserApp/Peripherals/RosLogger.cpp
    UserApp/Porting/SerialPort.cpp
    UserApp/Porting/cJSON.c
    UserApp/Control/ChassisManager.cpp
    UserApp/Control/SafetyMonitor.cpp
    UserApp/Control/PID_Controller.cpp
    UserApp/Control/KinematicProfile.cpp
    micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
    UserApp/Common
    UserApp/Peripherals
    UserApp/Control
    UserApp/Application
    UserApp/Porting
    UserApp/Thirdparty  # Eigen 矩阵库
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
    ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
    # Add user defined libraries
)

add_custom_target(microros_build
    COMMAND docker pull microros/micro_ros_static_library_builder:humble
    COMMAND docker run --rm --network host -v ${CMAKE_CURRENT_SOURCE_DIR}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide --env http_proxy=http://127.0.0.1:7897 --env https_proxy=http://127.0.0.1:7897 --env all_proxy=socks5://127.0.0.1:7897 microros/micro_ros_static_library_builder:humble
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building micro-ROS static library via Docker with Proxy"
)
