#!/bin/bash
set -e

######## Detect paths ########

# Project root: auto-detect from script location or use env var
if [ -n "$PROJECT_ROOT" ]; then
    PROJECT_ROOT="$PROJECT_ROOT"
else
    PROJECT_ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
fi

export BASE_PATH=$PROJECT_ROOT/micro_ros_stm32cubemx_utils/microros_static_library

# Firmware workspace
MICROROS_FW_WS=${MICROROS_FW_WS:-$HOME/microros_firmware_ws}

######## Init ########

# Toolchain: use env var, or auto-detect common paths
if [ -n "$TOOLCHAIN_PREFIX" ]; then
    echo "Using toolchain: $TOOLCHAIN_PREFIX"
elif [ -d "/host_toolchain/bin" ]; then
    echo "Using host toolchain from /host_toolchain"
    export TOOLCHAIN_PREFIX=/host_toolchain/bin/arm-none-eabi-
elif TOOLCHAIN_BIN=$(dirname $(which arm-none-eabi-gcc 2>/dev/null) 2>/dev/null) && [ -n "$TOOLCHAIN_BIN" ]; then
    echo "Using system toolchain: $TOOLCHAIN_BIN"
    export TOOLCHAIN_PREFIX=$TOOLCHAIN_BIN/arm-none-eabi-
else
    echo "No ARM toolchain found! Set TOOLCHAIN_PREFIX or install arm-none-eabi-gcc"
    exit 1
fi

source /opt/ros/$ROS_DISTRO/setup.bash

# Source micro_ros_setup workspace
MICROROS_SETUP_WS=${MICROROS_SETUP_WS:-$HOME/microros_ws}
if [ -f "$MICROROS_SETUP_WS/install/setup.bash" ]; then
    source $MICROROS_SETUP_WS/install/setup.bash
else
    echo "micro_ros_setup not found at $MICROROS_SETUP_WS. Install it first."
    exit 1
fi

# Create firmware workspace if not exists
if [ ! -f "$MICROROS_FW_WS/install/local_setup.bash" ]; then
    echo "Creating firmware workspace at $MICROROS_FW_WS ..."
    mkdir -p $MICROROS_FW_WS
    cd $MICROROS_FW_WS
    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
else
    echo "Using existing firmware workspace at $MICROROS_FW_WS"
    cd $MICROROS_FW_WS
fi

source install/local_setup.bash

######## Adding extra packages ########
pushd firmware/mcu_ws > /dev/null

    # Workaround: Copy just tf2_msgs
    git clone -b humble https://github.com/ros2/geometry2
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

popd > /dev/null

######## Trying to retrieve CFLAGS ########
pushd $PROJECT_ROOT > /dev/null
export RET_CFLAGS=$(make print_cflags)
RET_CODE=$?

if [ $RET_CODE = "0" ]; then
    echo "Found CFLAGS:"
    echo "-------------"
    echo $RET_CFLAGS
    echo "-------------"
    echo "Auto-confirming CFLAGS for non-interactive build."
else
    echo "Please read README.md to update your Makefile"
    exit 1;
fi
popd > /dev/null

######## Build  ########
ros2 run micro_ros_setup build_firmware.sh $BASE_PATH/library_generation/toolchain.cmake $BASE_PATH/library_generation/colcon.meta

find firmware/build/include/ -name "*.c"  -delete
rm -rf $BASE_PATH/libmicroros
mkdir -p $BASE_PATH/libmicroros/microros_include
cp -R firmware/build/include/* $BASE_PATH/libmicroros/microros_include/
cp -R firmware/build/libmicroros.a $BASE_PATH/libmicroros/libmicroros.a

######## Fix include paths  ########
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$BASE_PATH/libmicroros/microros_include/${var}/${var}" ]; then
        rsync -r $BASE_PATH/libmicroros/microros_include/${var}/${var}/* $BASE_PATH/libmicroros/microros_include/${var}
        rm -rf $BASE_PATH/libmicroros/microros_include/${var}/${var}
    fi
done

######## Generate extra files ########
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > $BASE_PATH/libmicroros/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> $BASE_PATH/libmicroros/available_ros2_types

cd firmware
echo "" > $BASE_PATH/libmicroros/built_packages
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> $BASE_PATH/libmicroros/built_packages; popd > /dev/null; done;

######## Fix permissions ########
chmod -R 755 $BASE_PATH/libmicroros/
chmod -R 755 $BASE_PATH/libmicroros/microros_include/
chmod 644 $BASE_PATH/libmicroros/libmicroros.a
