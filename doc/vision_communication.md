# 通信数据帧格式

## 架构

```
STM32H723 (下位机) ←──micro-ROS XRCE-DDS──→ Ubuntu (上位机)
                     USB CDC VCP (FS 12Mbps)

  Topic: stm32h723/vision_send    下位机 → 上位机    ~400 Hz
  Topic: stm32h723/vision_recv    上位机 → 下位机    按需
```

## Topic: vision_send (下位机 → 上位机)

云台姿态 + 机器人状态 (BEST_EFFORT QoS, ~400 Hz)

### JSON 格式

```json
{"e":0,"w":0,"b":15,"y":123,"p":-50,"r":10,"t":123456}
```

单包 < 64 bytes，适配 USB FS 单帧传输。

### 字段定义

| Key | 字段 | 类型 | 编码 | 范围 | 示例 | 说明 |
|-----|------|------|------|------|------|------|
| e | enemy_color | uint8 | 原始值 | 0-2 | 0 | 0=红 1=蓝 |
| w | work_mode | uint8 | 原始值 | 0-2 | 0 | 工作模式 |
| b | bullet_speed | uint8 | 原始值 | 0-30 | 15 | 弹速 m/s |
| y | yaw | int16 | centi-degree (×100) | -18000~18000 | 123 | yaw=1.23° |
| p | pitch | int16 | centi-degree (×100) | -9000~9000 | -50 | pitch=-0.50° |
| r | roll | int16 | centi-degree (×100) | -18000~18000 | 10 | roll=0.10° |
| t | timestamp | uint32 | ms (HAL_GetTick) | 0~2^32-1 | 123456 | 系统时间戳 |

### 上位机解析示例

```python
import json
msg = json.loads(data)
yaw_deg   = msg['y'] / 100.0   # centi-degree → degree
pitch_deg = msg['p'] / 100.0
roll_deg  = msg['r'] / 100.0
```

```cpp
// C 解析
int yaw_cdeg, pitch_cdeg, roll_cdeg, e, w, b, t;
sscanf(data, "{\"e\":%d,\"w\":%d,\"b\":%d,\"y\":%d,\"p\":%d,\"r\":%d,\"t\":%d}",
       &e, &w, &b, &yaw_cdeg, &pitch_cdeg, &roll_cdeg, &t);
float yaw = yaw_cdeg / 100.0f;
float pitch = pitch_cdeg / 100.0f;
float roll = roll_cdeg / 100.0f;
```

> yaw/pitch/roll 使用 centi-degree 整数编码的原因：下位机使用 newlib-nano，`snprintf("%f")` 输出空串。×100 取整后可以用 `jtoa`（手写整数转字符串）序列化，同时压缩包体积。

---

## Topic: vision_recv (上位机 → 下位机)

视觉自瞄指令 (RELIABLE QoS)

### JSON 格式

```json
{"f":2,"s":2,"t":3,"y":157,"p":-30,"m":0}
```

单包 ~37 bytes（压缩前 ~85 bytes）。

### 字段定义

| Key | 字段 | 类型 | 编码 | 说明 |
|-----|------|------|------|------|
| f | fire_mode | uint8 | 原始值 | 0=停火 1=单发 2=连发 |
| s | target_state | uint8 | 原始值 | 0=无目标 1=锁定 2=跟踪 |
| t | target_type | uint8 | 原始值 | 视觉分类 |
| y | yaw | float | centi-degree (×100) | 157=1.57°，下位机自动÷100 |
| p | pitch | float | centi-degree (×100) | -30=-0.30°，下位机自动÷100 |
| m | fire_tem | uint8 | 原始值 | 0=不开火 1=开火 |

### 上位机发送示例

```bash
ros2 topic pub --once /stm32h723/vision_recv std_msgs/msg/String \
  '{"data":"{\"f\":2,\"s\":2,\"t\":3,\"y\":157,\"p\":-30,\"m\":0}"}'
```

> vision_recv 使用浮点字符串格式（上位机视觉程序原生），不在下位机做二次编码。

---

## 通信规格

| 属性 | vision_send | vision_recv |
|------|-------------|-------------|
| QoS | BEST_EFFORT | RELIABLE (默认) |
| 发布频率 | ~400 Hz | 按需 (视觉帧率) |
| 单包大小 | < 64 B | < 130 B |
| 有效带宽 | ~21 KB/s | 忽略 |
| USB FS 利用率 | ~47% | — |

### 延迟

| 方向 | 典型 | 最大 |
|------|------|------|
| 下位机→上位机 | < 2ms | ~10ms |
| 上位机→下位机 | < 2ms | — |

## 上位机 Agent

```bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 115200
```

| 参数 | 值 |
|------|------|
| 设备 | `/dev/ttyACM1` (STM32 VCP, **不是** ttyACM0 JLink) |
| 波特率 | 115200 (USB CDC 虚拟波特率，不影响实际吞吐) |
| QoS 兼容 | agent 自动适配 BEST_EFFORT publisher |
