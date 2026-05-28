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

## 添加新的 Topic（发布/订阅）

以项目现有的 `vision_send`（发布）和 `vision_recv`（订阅）为例，逐步说明如何新增一个 micro-ROS topic。所有 micro-ROS 初始化代码写在下位机 `Core/Src/freertos.c` 的 `StartMicrorosTask()` 函数中（`#ifdef MICRO_ROS_ENABLED` 块内）。

### 代码写在哪

```
Core/Src/freertos.c   ← micro-ROS 节点、pub/sub、executor 全部在这里
├── 静态变量区（文件顶部 #ifdef MICRO_ROS_ENABLED 内）
│   ├── rcl_publisher_t    ← 每个 publisher 声明一个
│   ├── rcl_subscription_t ← 每个 subscriber 声明一个
│   ├── rclc_executor_t    ← 全局一个 executor
│   └── 消息缓冲区          ← String 类型必须 malloc 预分配
│
├── 回调函数（static void）
│   └── 收到消息后解析数据，写入应用层结构体
│
└── StartMicrorosTask()
    ├── 1. 注册 transport
    ├── 2. init support + node
    ├── 3. 初始化 publisher  (rclc_publisher_init_best_effort)
    ├── 4. 初始化 subscriber (rclc_subscription_init_best_effort)
    ├── 5. malloc 消息缓冲区
    ├── 6. 初始化 executor + 添加 subscription
    └── 7. 主循环: spin_some() + publish()
```

### 新增一个 Publisher（以 vision_send 为例）

**Step 1 — 声明变量**（文件顶部 `#ifdef MICRO_ROS_ENABLED` 内）:

```c
static rcl_publisher_t microros_pub_send;                  // publisher 句柄
static std_msgs__msg__String microros_send_msg;            // 消息体
```

**Step 2 — 初始化 publisher**（`StartMicrorosTask()` 内，node 创建之后）:

```c
rclc_publisher_init_best_effort(
    &microros_pub_send,
    &microros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),    // 消息类型
    "stm32h723/vision_send"                                // topic 名称
);
```

> **QoS 选择**: 高频数据（>50Hz）用 `_best_effort`，重要不丢的数据用 `_default`（RELIABLE）。本项目 vision 数据用 BEST_EFFORT 以达 ~400Hz。

**Step 3 — 预分配消息缓冲区**（仅 String 类型需要）:

```c
microros_send_msg.data.data = (char *)malloc(256);
microros_send_msg.data.size = 0;
microros_send_msg.data.capacity = 256;
```

> 如果使用 `Int32`、`Float32` 等定长类型，不需要 malloc，直接给字段赋值即可。

**Step 4 — 构造消息并发布**:

```c
rcl_ret_t microros_publish_vision(void)
{
    // 构造 JSON 字符串到 microros_send_msg.data.data
    char *buf = microros_send_msg.data.data;
    // ... 用 snprintf 或手写格式化填入 buf ...
    microros_send_msg.data.size = len;

    rcl_ret_t ret = rcl_publish(&microros_pub_send, &microros_send_msg, NULL);
    return ret;
}
```

如果使用定长类型（如 `std_msgs__msg__Int32`），更简单:

```c
std_msgs__msg__Int32 msg;
msg.data = 12345;
rcl_publish(&my_publisher, &msg, NULL);
```

**Step 5 — 在主循环中调用发布函数**（`StartMicrorosTask()` 的 `for(;;)` 循环内）:

```c
for (;;) {
    // ... spin_some 逻辑 ...
    microros_publish_vision();   // 每次循环都发布
    osDelay(1);
}
```

### 新增一个 Subscriber（以 vision_recv 为例）

**Step 1 — 声明变量**:

```c
static rcl_subscription_t microros_sub_recv;               // subscriber 句柄
static std_msgs__msg__String microros_recv_msg;            // 接收缓冲区
```

**Step 2 — 编写回调函数**:

```c
static void microros_vision_recv_cb(const void *msgin)
{
    const std_msgs__msg__String *in = (const std_msgs__msg__String *)msgin;
    if (in == NULL || in->data.data == NULL || in->data.size == 0) return;

    // 解析 JSON 或直接读取字段
    Vision_Recv_s *recv = VisionGetRecvData();
    int yaw_cdeg, pitch_cdeg;
    sscanf(in->data.data, "{\"y\":%d,\"p\":%d}", &yaw_cdeg, &pitch_cdeg);
    recv->yaw = yaw_cdeg / 100.0f;
    recv->pitch = pitch_cdeg / 100.0f;
}
```

> 回调在 `rclc_executor_spin_some()` 内被调用，处于 micro-ROS 任务上下文，不要做耗时操作。

**Step 3 — 初始化 subscriber**（`StartMicrorosTask()` 内）:

```c
rclc_subscription_init_best_effort(
    &microros_sub_recv,
    &microros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "stm32h723/vision_recv"
);
```

> subscriber 的 QoS 必须和 publisher 端一致。上位机用 `--qos-best-effort` 则下位机也用 `_best_effort`。

**Step 4 — 预分配接收缓冲区**（String 类型必须做）:

```c
microros_recv_msg.data.data = (char *)malloc(256);
microros_recv_msg.data.size = 0;
microros_recv_msg.data.capacity = 256;
```

> **不预分配是最常见的坑**: micro-ROS 反序列化 String 时需要目标缓冲区已就绪，否则回调静默不触发。

**Step 5 — 将 subscriber 加入 executor**:

```c
rclc_executor_init(&microros_executor, &microros_support.context,
                   2,                  // handles 数量 = pub数 + sub数
                   &microros_allocator);
rclc_executor_add_subscription(
    &microros_executor,
    &microros_sub_recv,
    &microros_recv_msg,
    microros_vision_recv_cb,           // 回调函数
    ON_NEW_DATA                        // 有新数据才触发
);
```

> `handles` 数量必须 ≥ (publisher 数 + subscriber 数)，否则 executor 初始化失败。

**Step 6 — 在主循环中 spin executor**:

```c
for (;;) {
    if (++cycle >= 2) {               // 每 2 次循环 spin 一次，平衡收发
        cycle = 0;
        rclc_executor_spin_some(&microros_executor, RCL_MS_TO_NS(1));
    }
    microros_publish_vision();
    osDelay(1);
}
```

### 完整模板：新增一个 Float32 Publisher

以下是从零添加一个新 topic 的完整 diff 示例 — 发布 `std_msgs/msg/Float32` 到 `stm32h723/battery_voltage`:

```c
// ===== 文件顶部变量声明区 =====
static rcl_publisher_t battery_pub;
static std_msgs__msg__Float32 battery_msg;

// ===== StartMicrorosTask() 内，node 初始化之后 =====
rclc_publisher_init_best_effort(
    &battery_pub,
    &microros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "stm32h723/battery_voltage"
);

// ===== 主循环的 for(;;) 内，发布处 =====
battery_msg.data = 24.0f;  // 从实际传感器读取
rcl_publish(&battery_pub, &battery_msg, NULL);
```

> 定长类型不需要 `malloc`，不需要处理 `data.size`/`data.capacity`，直接用。

### 完整模板：新增一个 Int32 Subscriber

接收 `std_msgs/msg/Int32` 控制指令:

```c
// ===== 文件顶部变量声明区 =====
static rcl_subscription_t cmd_sub;
static std_msgs__msg__Int32 cmd_msg;

// ===== 回调函数 =====
static void cmd_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *in = (const std_msgs__msg__Int32 *)msgin;
    if (in == NULL) return;
    // 直接用 in->data，无需 malloc/解析
    set_motor_speed(in->data);
}

// ===== StartMicrorosTask() 内 =====
rclc_subscription_init_default(         // _default = RELIABLE
    &cmd_sub,
    &microros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stm32h723/motor_cmd"
);

// executor handles 数量 +1 (原来是 2，现在 3)
rclc_executor_init(&microros_executor, &microros_support.context,
                   3, &microros_allocator);
rclc_executor_add_subscription(&microros_executor, &cmd_sub,
                               &cmd_msg, cmd_callback, ON_NEW_DATA);
```

### 常见消息类型速查

| ROS 2 类型 | C 类型 | 需预分配 | QoS 建议 |
| ----------- | ------ | -------- | -------- |
| `std_msgs/msg/String` | `std_msgs__msg__String` | **是** malloc data.data | 按需 |
| `std_msgs/msg/Int32` | `std_msgs__msg__Int32` | 否 | RELIABLE |
| `std_msgs/msg/Float32` | `std_msgs__msg__Float32` | 否 | BEST_EFFORT |
| `std_msgs/msg/Bool` | `std_msgs__msg__Bool` | 否 | RELIABLE |
| `std_msgs/msg/Int32MultiArray` | `std_msgs__msg__Int32MultiArray` | **是** malloc data.data | 按需 |
| `sensor_msgs/msg/Imu` | `sensor_msgs__msg__Imu` | 否 | BEST_EFFORT |

### 新增 Topic 检查清单

- [ ] 变量声明在 `#ifdef MICRO_ROS_ENABLED` 块内（否则关 micro-ROS 时编译报错）
- [ ] publisher 初始化在 `rclc_node_init_default` 之后
- [ ] String/MultiArray 类型已 `malloc` 缓冲区
- [ ] subscriber 已添加到 executor（`rclc_executor_add_subscription`）
- [ ] executor `handles` 数量 ≥ pub + sub 总数
- [ ] 回调函数标记为 `static`，不做耗时操作
- [ ] 编译前确认 `-DMICRO_ROS_ENABLED` 在 Makefile C_DEFS 中
- [ ] 上位机 `ros2 topic echo/list` 能发现新 topic

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
