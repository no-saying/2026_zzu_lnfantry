# micro-ROS 上下位机使用说明

## 架构总览

```
┌─────────────────────────────┐      XRCE-DDS (serial)      ┌──────────────────────┐
│  STM32H723VGTx (下位机)      │ ──────────────────────────→  │  Ubuntu PC (上位机)    │
│                             │   USB VCP or USART10 DMA     │                      │
│  ┌───────────────────────┐  │                              │  micro_ros_agent     │
│  │  StartMicrorosTask    │  │                              │  (serial → UDP)      │
│  │  - node: stm32h723_node│  │                              │           ↓          │
│  │  - publisher:          │  │                              │  ROS 2 Humble        │
│  │    stm32h723/vision_data│ │                              │  $ ros2 topic echo   │
│  │  - transport:          │  │                              │                      │
│  │    VCP / UART callback │  │                              │                      │
│  └───────────────────────┘  │                              │                      │
│                             │                              │                      │
│  调用方:                     │                              │                      │
│  microros_publish_string()   │                              │                      │
│  (任意 task 均可调用)         │                              │                      │
└─────────────────────────────┘                              └──────────────────────┘
```

## 通信介质选择

在 `Makefile` 的 `C_DEFS` 中切换（二选一）：

| 宏定义 | 物理接口 | 共享方式 |
|--------|---------|---------|
| `-DCOMM_USE_VCP` **(当前)** | USB CDC 虚拟串口 (CN5) | vision + micro-ROS 共用 USB，通过多回调机制分流 |
| 不定义 `COMM_USE_VCP` | USART10 DMA | vision + micro-ROS 共用 USART10 |

切换后需 `make clean && make` 重新编译。

## 构建

### 项目构建

```bash
make clean && make -j$(nproc)
```

产物:
- `build/Basic_Framework_MC02.elf` — ELF 调试文件
- `build/Basic_Framework_MC02.hex` — Intel HEX
- `build/Basic_Framework_MC02.bin` — 原始二进制

### libmicroros.a 重新构建 (仅当需要更新 micro-ROS 包时)

环境要求: Docker、VPN（拉取 GitHub 仓库）

```bash
cd micro_ros_stm32cubemx_utils/microros_static_library/library_generation

# 启动 Docker，挂载宿主工具链
docker run -it --rm \
    -v $(pwd):/project \
    -v /home/yeht/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi:/host_toolchain \
    -v /home/yeht/stm/2026_zzu_lnfantry/Makefile:/project/Makefile \
    --net=host \
    microros/micro_ros_static_library_builder:humble

# 容器内执行:
bash library_generation.sh

# 将产物复制回项目
cp libmicroros/libmicroros.a /home/yeht/stm/2026_zzu_lnfantry/lib/
cp -r libmicroros/microros_include/* /home/yeht/stm/2026_zzu_lnfantry/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/
```

> **注意**: Docker 构建使用宿主 `arm-gnu-toolchain-14.3` 以保持与链接器的二进制兼容。若工具链路径不同，修改对应的 `-v` 挂载路径。

## 烧录

### DAP-Link (OpenOCD)

```bash
make download_dap
```

### J-Link

```bash
make download_jlink
```

## 上位机配置

### 1. 安装 micro_ros_agent

```bash
# Humble 版本
source /opt/ros/humble/setup.bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

### 2. 启动 Agent

**VCP 模式** (USB 虚拟串口):
```bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

**UART 模式** (USART10, 需 USB-TTL):
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

> 下位机上电初始化 micro-ROS 后会主动连接 agent，无需手动配对。若连接中断，agent 会自动重连。

### 3. 验证通信

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /stm32h723/vision_data
```

正常输出示例:
```
data: 'Hello from STM32H723!'
---
```

### 4. 查看节点信息

```bash
ros2 node list
# 应输出: /stm32h723_node

ros2 topic list
# 应输出: /stm32h723/vision_data
#        /parameter_events
```

## 下位机 API

### 发布数据

在任意任务中调用，发送字符串到上位机:

```c
#include "microros_transport.h"

// 示例: 在 vision 任务中发布检测结果
void VisionTask(void)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f}",
             gimbal.yaw, gimbal.pitch, gimbal.roll);
    microros_publish_string(buf);
}
```

### 调整 micro-ROS 任务栈

在 `Core/Src/freertos.c` 中:

```c
// 栈大小 (words = bytes/4, 当前 2048 = 8KB)
osThreadDef(microrosTask, StartMicrorosTask, osPriorityAboveNormal, 0, 2048);
```

> 若运行时出现栈溢出 (stack overflow hook 触发)，增大此值。

## Topic 信息

| 属性 | 值 |
|------|---|
| Topic 名称 | `/stm32h723/vision_data` |
| 消息类型 | `std_msgs/msg/String` |
| 节点名称 | `stm32h723_node` |
| QoS | 默认 (reliable, volatile, keep_last=10) |

## 添加自定义消息类型

当前仅使用 `std_msgs/msg/String`。如需添加其他消息类型（如自定义 ROS 2 msg）:

### 1. 添加 .msg 文件

```bash
# 创建自定义消息包
mkdir -p micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/my_custom_msgs/msg
cat > micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/my_custom_msgs/msg/robot_state.msg << 'EOF'
float64 yaw
float64 pitch
float64 roll
int32 fire_status
EOF
```

### 2. 更新 extra_packages.repos

在 `library_generation/extra_packages/extra_packages.repos` 中添加新包的依赖声明。

### 3. 重新构建 libmicroros.a

按照上文"libmicroros.a 重新构建"步骤执行。

### 4. 在下位机使用

```c
#include <my_custom_msgs/msg/robot_state.h>

static my_custom_msgs__msg__robot_state state_msg;

void publishRobotState(void)
{
    state_msg.yaw = 1.57;
    state_msg.pitch = 0.0;
    state_msg.roll = 0.0;
    state_msg.fire_status = 0;
    rcl_publish(&publisher, &state_msg, NULL);
}
```

## 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| Agent 无法连接 | 串口设备错误 | `ls /dev/tty*` 确认设备名 |
| | 串口被占用 | `sudo lsof /dev/ttyACM0` 查找占用进程 |
| | 下位机 VCP 未初始化 | 检查 USB 线缆连接，确认 `COMM_USE_VCP` 宏已定义 |
| `ros2 topic echo` 无输出 | publisher 未创建 | 检查 `freertos.c` 中 StartMicrorosTask 是否正常启动 |
| | 消息未发送 | 调用 `microros_publish_string()` 的代码路径未执行 |
| 编译报找不到 micro-ROS 头文件 | 库未构建 | 确认 `lib/libmicroros.a` 存在 |
| | 头文件路径错误 | 确认 `microros_include/` 指针正确 |
| 固件溢出 | FreeRTOS 堆过大 | DTCMRAM 段是否生效 (查看 map: `.dtcmram` 应有内容) |
