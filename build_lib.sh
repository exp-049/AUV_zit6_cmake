#!/bin/bash

# 确保脚本在出错时停止
set -e

# 获取当前目录
PROJECT_ROOT=$(pwd)

echo "========================================================="
echo "   AUV Zit6 - Integrated Build Script"
echo "========================================================="

# 0. 准备工作与缓存检查
echo "[0/3] Preparing environment & checking cache..."
EXTRA_PKG_DIR="micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages"
HASH_FILE=".msg_hash"
LIB_FILE="micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a"
MICROROS_CONFIG="microros_config.meta"
MERGE_SCRIPT="scripts/merge_microros_config.py"

# 如果有外置配置，将其注入到 colcon.meta 中
COLCON_META="micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/colcon.meta"
if [ -f "$MICROROS_CONFIG" ] && [ -f "$COLCON_META" ]; then
    echo ">>> Injecting $MICROROS_CONFIG into colcon.meta..."
    python3 "$MERGE_SCRIPT" "$COLCON_META" "$MICROROS_CONFIG" "$COLCON_META"
fi

# 计算当前接口定义与生成配置的哈希值
# 包含 zit6_interfaces、生成配置目录、及外置 RMW 配置
HASH_PATHS="zit6_interfaces"
GEN_DIR="micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation"
if [ -d "$GEN_DIR" ]; then
    HASH_PATHS="$HASH_PATHS $GEN_DIR"
fi
if [ -f "$MICROROS_CONFIG" ]; then
    HASH_PATHS="$HASH_PATHS $MICROROS_CONFIG"
fi
CURRENT_HASH=$(find $HASH_PATHS -type f -exec md5sum {} + | sort | md5sum | awk '{print $1}')
OLD_HASH=""
if [ -f "$HASH_FILE" ]; then OLD_HASH=$(cat "$HASH_FILE"); fi

# 检查是否需要重新生成库
NEED_REBUILD=true
if [ "$CURRENT_HASH" == "$OLD_HASH" ] && [ -f "$LIB_FILE" ]; then
    echo ">>> Interfaces haven't changed. Skipping micro-ROS library generation."
    NEED_REBUILD=false
else
    echo ">>> Interfaces changed or library missing. Rebuilding..."
fi

if [ "$NEED_REBUILD" = true ]; then
    # 仅在真正需要时清理和同步
    sudo rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
    mkdir -p "$EXTRA_PKG_DIR"
    if [ -L "$EXTRA_PKG_DIR/zit6_interfaces" ]; then rm "$EXTRA_PKG_DIR/zit6_interfaces"; fi
    cp -rL zit6_interfaces "$EXTRA_PKG_DIR/"

    # 1. 生成 micro-ROS 静态库 (使用 Docker)
    echo "[1/3] Generating micro-ROS library via Docker..."
    docker run --rm --network host --name microros_builder \
      -v "${PROJECT_ROOT}:/project" \
      --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
      --env http_proxy=http://127.0.0.1:7897 \
      --env https_proxy=http://127.0.0.1:7897 \
      --env all_proxy=socks5://127.0.0.1:7897 \
      microros/micro_ros_static_library_builder:humble
    
    # 保存哈希
    echo "$CURRENT_HASH" > "$HASH_FILE"
    echo ">>> micro-ROS library generation finished."
fi

# 2. 编译 STM32 固件
echo "[2/3] Building STM32 firmware..."
mkdir -p build
# 使用项目自带的 ARM 工具链文件重新生成配置
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build

# 3. 编译上位机接口
echo "[3/3] Building ROS 2 host interfaces..."
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi
# 指定 base path 为 `zit6_interfaces`，避免顶层 CMake 包（AUV_zit6）阻止对子目录包的发现
colcon build --base-paths zit6_interfaces --packages-select zit6_interfaces --symlink-install

echo "========================================================="
echo "   Build Success!"
echo "========================================================="
