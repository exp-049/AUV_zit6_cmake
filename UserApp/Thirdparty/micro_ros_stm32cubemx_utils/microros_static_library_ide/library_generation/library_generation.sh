#!/bin/bash
set -e

run_with_proxy_fallback() {
    local desc="$1"
    shift

    echo "Running: ${desc}"
    if "$@"; then
        return 0
    fi

    local rc=$?
    echo "WARN: ${desc} failed with code ${rc}. Retrying without git proxy and with clean firmware workspace..."

    git config --global --unset http.proxy || true
    git config --global --unset https.proxy || true
    unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY

    rm -rf /uros_ws/firmware || true
    "$@"
}

export BASE_PATH=/project/$MICROROS_LIBRARY_FOLDER

# 深度优化 Git 代理设置以对抗不稳定的网络
git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
git config --global http.sslVerify false
git config --global http.version 1.1
git config --global http.postBuffer 1048576000
git config --global http.lowSpeedLimit 0
git config --global http.lowSpeedTime 999999
git config --global core.compression 0

######## Check existing library ########
if [ -f "$BASE_PATH/libmicroros/libmicroros.a" ]; then
    echo "micro-ROS library found. Skipping..."
    echo "Delete $MICROROS_LIBRARY_FOLDER/libmicroros/ for rebuild."
    exit 0
fi
######## Trying to retrieve CFLAGS ########
echo "Bypassing CFLAGS extraction for CMake..."
export RET_CFLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"

######## Init ########
apt-get update

# 增加重试机制以应对不稳定的代理
for i in {1..5}; do
    apt-get install -y --fix-missing gcc-arm-none-eabi && break || sleep 5
done

cd /uros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f install/local_setup.bash ]; then
    source install/local_setup.bash
fi

run_with_proxy_fallback \
    "ros2 run micro_ros_setup create_firmware_ws.sh generate_lib" \
    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

######## Adding extra packages ########
pushd firmware/mcu_ws > /dev/null

    # Workaround: Copy just tf2_msgs
    git clone -b ros2 https://github.com/ros2/geometry2
    cp -R geometry2/tf2_msgs ros2/tf2_msgs
    rm -rf geometry2

    # Import user defined packages
    mkdir extra_packages
    pushd extra_packages > /dev/null
        USER_CUSTOM_PACKAGES_DIR=$BASE_PATH/../../microros_component/extra_packages 
    	if [ -d "$USER_CUSTOM_PACKAGES_DIR" ]; then
    		cp -R $USER_CUSTOM_PACKAGES_DIR/* .
		fi
        if [ -f $USER_CUSTOM_PACKAGES_DIR/extra_packages.repos ]; then
        	vcs import --input $USER_CUSTOM_PACKAGES_DIR/extra_packages.repos
        fi
        cp -R $BASE_PATH/library_generation/extra_packages/* .
        vcs import --input extra_packages.repos
    popd > /dev/null

    if [ ! -z ${MICROROS_USE_EMBEDDEDRTPS+x} ]; then
        rm -rf eProsima/Micro-XRCE-DDS-Client
        rm -rf uros/rmw_microxrcedds

        git clone -b main https://github.com/micro-ROS/rmw_embeddedrtps uros/rmw_embeddedrtps
        git clone -b main https://github.com/micro-ROS/embeddedRTPS uros/embeddedRTPS
    fi

popd > /dev/null

######## Build  ########
export TOOLCHAIN_PREFIX=/usr/bin/arm-none-eabi-

if [ ! -z ${MICROROS_USE_EMBEDDEDRTPS+x} ]; then
    run_with_proxy_fallback \
        "ros2 run micro_ros_setup build_firmware.sh (embeddedrtps)" \
        ros2 run micro_ros_setup build_firmware.sh $BASE_PATH/library_generation/toolchain.cmake $BASE_PATH/library_generation/colcon-embeddedrtps.meta
else
    run_with_proxy_fallback \
        "ros2 run micro_ros_setup build_firmware.sh" \
        ros2 run micro_ros_setup build_firmware.sh $BASE_PATH/library_generation/toolchain.cmake $BASE_PATH/library_generation/colcon.meta
fi

find firmware/build/include/ -name "*.c"  -delete
rm -rf $BASE_PATH/libmicroros
mkdir -p $BASE_PATH/libmicroros/include
cp -R firmware/build/include/* $BASE_PATH/libmicroros/include/
cp -R firmware/build/libmicroros.a $BASE_PATH/libmicroros/libmicroros.a

######## Fix include paths  ########
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$BASE_PATH/libmicroros/include/${var}/${var}" ]; then
        rsync -r $BASE_PATH/libmicroros/include/${var}/${var}/* $BASE_PATH/libmicroros/include/${var}
        rm -rf $BASE_PATH/libmicroros/include/${var}/${var}
    fi
done

######## Generate extra files ########
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > $BASE_PATH/libmicroros/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> $BASE_PATH/libmicroros/available_ros2_types

cd firmware
echo "" > $BASE_PATH/libmicroros/built_packages
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> $BASE_PATH/libmicroros/built_packages; popd > /dev/null; done;

######## Fix permissions ########
sudo chmod -R 777 $BASE_PATH/libmicroros/
sudo chmod -R 777 $BASE_PATH/libmicroros/include/
sudo chmod -R 777 $BASE_PATH/libmicroros/libmicroros.a
