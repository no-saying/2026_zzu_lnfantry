# micro-ROS 上下位机使用说明

## 架构总览

### micro-ROS 模式 (`MICRO_ROS_ENABLED`)

```
┌─────────────────────────────┐      XRCE-DDS (serial)      ┌──────────────────────┐
│  STM32H723VGTx (下位机)      │ ──────────────────────────→  │  Ubuntu PC (上位机)    │
│                             │   USB VCP or USART10 DMA     │                      │
│  ┌───────────────────────┐  │                              │  micro_ros_agent     │
│  │  StartMicrorosTask    │  │   pub: stm32h723/vision_send │  (serial → UDP)      │
│  │  node: stm32h723_node │  │ ──────────────────────────→ │           ↓          │
│  │                       │  │   sub: stm32h723/vision_recv │  ROS 2 Humble        │
│  │                       │  │ ←────────────────────────── │  vision_computer node│
│  └───────────────────────┘  │                              │                      │
│  调用方: RobotCMDTask        │                              │                      │
│  microros_publish_vision()   │                              │                      │
└─────────────────────────────┘                              └──────────────────────┘
```

### 原始串口模式 (未定义 `MICRO_ROS_ENABLED`)

```
┌─────────────────────────────┐   Seasky 协议 (raw serial)   ┌──────────────────────┐
│  STM32H723VGTx (下位机)      │ ──────────────────────────→  │  视觉上位机            │
│                             │   USB VCP or USART1           │                      │
│  VisionSend() / VisionInit()│   cmd_id=0x02, float ypr      │  接收 yaw/pitch/roll  │
│                             │ ←──────────────────────────  │  发送 目标信息+开火    │
└─────────────────────────────┘                              └──────────────────────┘
```

## 通信模式切换

在 `Makefile` 的 `C_DEFS` 中控制：

| 宏定义 | 作用 | 说明 |
|--------|------|------|
| `-DCOMM_USE_VCP` | USB 虚拟串口 | 与 `COMM_USE_UART` 二选一 |
| `-DMICRO_ROS_ENABLED` | 启用 micro-ROS | vision 数据走 ROS 2 topic |
| 取消 `MICRO_ROS_ENABLED` | 回退原始串口 | vision 数据走 seasky 协议 |

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

#### 方式一: 原生构建 (推荐，无需 Docker)

环境要求: ROS 2 Humble、micro_ros_setup、arm-gnu-toolchain-14.3

```bash
cd micro_ros_stm32cubemx_utils/microros_static_library/library_generation

# 直接运行 (脚本自动检测工具链和 workspace)
bash library_generation.sh

# 产物会自动输出到 libmicroros/ 目录，无需手动复制
```

脚本会自动:
- 检测 `arm-none-eabi-gcc` 工具链位置
- 在 `~/microros_firmware_ws` 创建/复用固件编译工作区
- 读取 Makefile 的 CFLAGS 确保编译参数一致

可通过环境变量覆盖默认路径:

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `PROJECT_ROOT` | 脚本自动定位 | 项目根目录 |
| `TOOLCHAIN_PREFIX` | `which arm-none-eabi-gcc` 结果 | 工具链前缀 |
| `MICROROS_FW_WS` | `~/microros_firmware_ws` | 固件工作区路径 |
| `MICROROS_SETUP_WS` | `~/microros_ws` | micro_ros_setup 工作区 |

#### 方式二: Docker 构建 (备选)

环境要求: Docker、VPN（拉取 GitHub 仓库）

```bash
cd micro_ros_stm32cubemx_utils/microros_static_library/library_generation

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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 115200
```

**UART 模式** (USART10, 需 USB-TTL):
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

> 下位机上电初始化 micro-ROS 后会主动连接 agent，无需手动配对。若连接中断，agent 会自动重连。

### 3. 验证通信

**查看下位机发送的云台姿态:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /stm32h723/vision_send
```

正常输出示例 (~400 Hz, JSON < 64 bytes, 整数编码):
```
data: '{"e":0,"w":0,"b":15,"y":123,"p":-50,"r":10,"t":123456}'
---
```

字段说明:
| Key | 含义 | 编码 |
|-----|------|------|
| e | enemy_color | 原始值 |
| w | work_mode | 原始值 |
| b | bullet_speed | 原始值 |
| y | yaw | centi-degree (×100), 123 = 1.23° |
| p | pitch | centi-degree (×100), -50 = -0.50° |
| r | roll | centi-degree (×100), 10 = 0.10° |
| t | timestamp | ms (HAL_GetTick) |

**发送自瞄数据到下位机（模拟视觉上位机）:**
```bash
# 快捷脚本
bash tools/test_vision_recv.sh [fire_mode] [target_state] [target_type] [yaw_cdeg] [pitch_cdeg] [fire_tem]

# 或直接 ros2 topic pub
ros2 topic pub --once /stm32h723/vision_recv std_msgs/msg/String \
  '{"data":"{\"f\":2,\"s\":2,\"t\":3,\"y\":157,\"p\":-30,\"m\":0}"}'
```
> yaw/pitch 为 centi-degree 整数 (度×100)，下位机自动 ÷100 转换。

### 4. 查看节点信息

```bash
ros2 node list
# 应输出: /stm32h723_node

ros2 topic list
# 应输出: /stm32h723/vision_send
#        /stm32h723/vision_recv
#        /parameter_events

ros2 topic hz /stm32h723/vision_send
# 应输出: ~400 Hz (BEST_EFFORT publisher)
```

## 下位机 API

### 发布视觉数据 (micro-ROS 模式)

在 `RobotCMDTask()` 中自动调用，发送云台姿态 + 标志位到上位机:

```c
// RobotCMDTask() 中自动执行:
VisionSetAltitude(yaw, pitch, roll);
VisionSetFlag(enemy_color, work_mode, bullet_speed);
microros_publish_vision();   // → /stm32h723/vision_send
```

### 接收视觉自瞄数据 (micro-ROS 模式)

下位机自动从 `/stm32h723/vision_recv` topic 接收，解析后写入 `Vision_Recv_s` 结构体。应用层如原来一样通过 `vision_recv_data` 指针读取:

```c
Vision_Recv_s *vision_recv_data = VisionInit();

// 在 RobotCMDTask() 中使用:
if (vision_recv_data->fire_tem == 1) {
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
}
gimbal_cmd_send.yaw = -vision_recv_data->yaw;
gimbal_cmd_send.pitch = vision_recv_data->pitch;
```

### 实现注意事项

- **接收缓冲区预分配**: `std_msgs__msg__String` 的 `data.data` 必须用 `malloc()` 预分配内存，否则 micro-ROS 反序列化无目标缓冲区，回调不会被触发。
- **spin_some 交错调度**: executor 每 2 次迭代执行一次 `spin_some(1ms)`，兼顾接收可靠性和发布频率。过于频繁会降低发布速率，过于稀疏会丢失接收数据。
- **pack(1) 安全解析**: `Vision_Recv_s` 使用 `#pragma pack(1)`，`sscanf` 必须写入 `int` 局部变量后再赋值到 packed enum 字段，避免 4 字节写入覆盖相邻字段。

### 切换回原始串口模式

在 Makefile 中删除 `-DMICRO_ROS_ENABLED`，vision 数据自动回退到 seasky 协议，`VisionSend()` 和 seasky 解析恢复工作。

## Topic 信息

| 属性 | vision_send (下位机→上位机) | vision_recv (上位机→下位机) |
|------|--------------------------|--------------------------|
| Topic 名称 | `/stm32h723/vision_send` | `/stm32h723/vision_recv` |
| 消息类型 | `std_msgs/msg/String` | `std_msgs/msg/String` |
| 数据格式 | JSON: e(敌色), w(模式), b(弹速), y(偏航×100), p(俯仰×100), r(滚转×100), t(时间戳) | JSON: f(开火), s(目标状态), t(目标类型), y(偏航×100), p(俯仰×100), m(开火指令) |
| QoS | BEST_EFFORT | BEST_EFFORT |
| 发布频率 | ~400 Hz | 按需 |

> **QoS 说明**: 两个 topic 均使用 BEST_EFFORT。vision_send 以获得 ~400 Hz 发布频率；vision_recv 以避免 RELIABLE ACK 导致 XRCE 流冲突。上位机 `ros2 topic echo` 订阅时自动适配 QoS。

### vision_send JSON 示例 (压缩格式, < 64 bytes)

```json
{"e":0,"w":0,"b":15,"y":123,"p":-50,"r":10,"t":123456}
```

> yaw/pitch/roll 为 centi-degree 整数 (原始弧度 ×100 取整)。上位机需除以 100 恢复为度。

### vision_recv JSON 示例 (压缩格式, < 48 bytes)

```json
{"f":2,"s":2,"t":3,"y":157,"p":-30,"m":0}
```

> yaw/pitch 为 centi-degree 整数 (度×100)，下位机自动 ÷100 恢复。上位机发送时需 ×100 取整。

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
| | 串口被占用 | `sudo lsof /dev/ttyACM1` 查找占用进程 |
| | 下位机 VCP 未初始化 | 检查 USB 线缆连接，确认 `COMM_USE_VCP` 宏已定义 |
| `ros2 topic echo` 无输出 | publisher 未创建 | 检查 `freertos.c` 中 StartMicrorosTask 是否正常启动 |
| | 消息未发送 | `MICRO_ROS_ENABLED` 未定义，或 `microros_publish_vision()` 未执行 |
| 编译报找不到 micro-ROS 头文件 | 库未构建 | 确认 `lib/libmicroros.a` 存在 |
| | 头文件路径错误 | 确认 `microros_include/` 指针正确 |
| 固件溢出 | FreeRTOS 堆过大 | DTCMRAM 段是否生效 (查看 map: `.dtcmram` 应有内容) |
| 烧录后 agent 无法连接 | STM32 需重新枚举 USB | 按一下复位键，等待 USB 重新枚举后再启动 agent |
| 发布频率低 (< 50 Hz) | QoS 误用 RELIABLE | 确认 `freertos.c` 中 publisher 使用 `rclc_publisher_init_best_effort` |
| `ros2 topic echo` 报 QoS 不兼容 | subscriber 默认 RELIABLE | 正常现象，micro-ROS agent 已自动处理，数据仍可达 |
| vision_recv 收不到数据 | recv_msg 缓冲区未预分配 | 确认 `freertos.c` 中 `microros_recv_msg.data.data` 已 `malloc(256)` |
| | spin_some 间隔过长 | 确保 executor spin 间隔 ≤ 2 次迭代 (≤ 2ms) |
| | XRCE session 创建顺序 | 先启动 agent，再复位 STM32 |
