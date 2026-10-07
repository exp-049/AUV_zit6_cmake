#!/bin/bash

# 确保脚本在出错时停止
set -e

# 获取当前目录
PROJECT_ROOT=$(pwd)

echo "========================================================="
echo "   AUV Zit6 - Integrated Build Script"
echo "========================================================="

# 0/1. Share the same fingerprint cache and protected generation path as CMake.
echo "[0/3] Checking micro-ROS inputs and cached artifacts..."
python3 scripts/ensure_microros_library.py --project-root "$PROJECT_ROOT"

# 2. 编译 STM32 固件
echo "[2/3] Building STM32 firmware..."
BUILD_DIR="build"
# 如果历史构建目录使用了其他生成器，切换到独立 Ninja 目录，避免
# CMake 因生成器不一致而中止，也不删除用户已有的构建结果。
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    CACHED_GENERATOR=$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' "$BUILD_DIR/CMakeCache.txt" | head -n 1)
    if [ -n "$CACHED_GENERATOR" ] && [ "$CACHED_GENERATOR" != "Ninja" ]; then
        BUILD_DIR="build_ninja"
        echo ">>> Existing build uses '$CACHED_GENERATOR'; using $BUILD_DIR for Ninja."
    fi
fi
mkdir -p "$BUILD_DIR"
# 使用项目自带的 ARM 工具链文件重新生成配置
cmake -S . -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=UserApp/Peripherals/HAL/gcc-arm-none-eabi.cmake
cmake --build "$BUILD_DIR"

# 3. 编译上位机接口
echo "[3/3] Building ROS 2 host interfaces..."
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi
# 指定 base path 为 `zit6_interfaces`，避免顶层 CMake 包（AUV_zit6）阻止对子目录包的发现
colcon build --base-paths zit6_interfaces --packages-select zit6_interfaces --symlink-install

echo "========================================================="
echo "   Build Success!"
echo "========================================================="
