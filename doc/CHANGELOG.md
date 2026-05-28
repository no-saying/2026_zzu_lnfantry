# 更新日志

## 2026-05-28 — 文档完善

### MICROROS_GUIDE 更新
- **新增 "添加新的 Topic（发布/订阅）" 教程章节**: Publisher 五步法、Subscriber 六步法、Float32/Int32 完整代码模板、消息类型速查表、新增 Topic 检查清单

## 2026-05-26 — DM 电机协议完善

### DM 电机协议重写 (`dmmotor.c/h`)
基于官方 [damiao_ros2](https://github.com/yerik0507/damiao_ros2) 完整协议实现:
- **电机型号支持**: 新增 `DM_Motor_Type_e` 枚举 (DM4310 ~ DMG6220 共 12 种型号)
- **per-type 限制查表**: `DM_Motor_Limit_s` 结构体 + `dm_motor_limits[]` 常量表，位置/速度/扭矩限制按型号区分 (如 DM4310: ±12.5 rad / ±30 rad/s / ±10 Nm)
- **控制模式切换**: 新增 `DM_Control_Mode_e` (MIT/位置-速度/速度/位置-力)，通过 `DMMotorSetControlMode()` 写 RID=10 切换
- **MIT 帧修正**: send 帧 `tx[3]`/`tx[6]` 增加 `& 0xF` 掩码保护；decode 使用 per-motor limits 替代硬编码 ±3.1416 rad
- **寄存器读写**: 新增 `DMMotorReadRegister()` / `DMMotorWriteRegister()`，通过 CAN ID 0x7FF + 命令 0x33/0x55 读写电机参数
- **参数保存**: 新增 `DMMotorSaveParams()`，CAN ID 0x7FF + 命令 0xAA 保存到电机 Flash
- **枚举重命名**: `DMMotor_Mode_e` → `DM_Motor_Cmd_e`，区分状态命令 (使能/停止/归零) 与控制模式

### motor_def.h
- `Motor_Type_e` 新增 `DM_MOTOR` 类型

## 2026-05-26 — 模块扩展与通信优化

### micro-ROS 优化
- **发布频率提升**: ~166 Hz → ~400 Hz，采用 spin_some 交错调度 (每 2 次迭代 spin 1ms)
- **vision_recv 修复**: 订阅消息缓冲区必须 `malloc(256)` 预分配，否则 micro-ROS 反序列化无目标缓冲区导致回调永不触发
- **QoS 统一**: vision_recv 也改为 BEST_EFFORT，避免 RELIABLE ACK 与高频 vision_send 竞争 XRCE 流
- **传输层写入优化**: CDC 发送重试从 20 次减为 3 次，每次 osDelay(1)，减少阻塞时间
- **pack(1) 安全解析**: `sscanf` 写入 `int` 局部变量后赋值到 packed 字段，避免 4 字节写入覆盖相邻内存

### 模块更新
- **GO 电机驱动重写** (`go_motor.c/h`):
  - 数据类型从 `int16`/`int32` 升级为 `float32`，控制精度提升
  - 新增 `GOMotor_Control_Setting_s` 结构体：Kp/Kd + 跳跃参数 + 加速度前馈
  - RS485 总线支持：新增 `GOMotorUSARTInstance` 封装 `USARTInstance + DaemonInstance`
  - 电机数量上限 GO_MOTOR_CNT 从 4 → 16
  - 新增 `SetMode()`、`SetPos()`、`Stop()`、`Enable()` 等独立控制 API
  - 控制报文 (`motor_ctrl`) 改为精确打包 (`#pragma pack(1)`)，pos_set 从 `int` 扩展为 `int32_t`
- **DM 电机** (`dmmotor.c/h`):
  - 新增 `DMMotorSetMode()` 接口
  - 电机数量上限 DM_MOTOR_CNT 4

### BSP 更新
- **bsp_usart**: 新增 RS485 总线支持 (`enable_485`/`id` 字段)、`USART_H7_SetBaudRateOnly()` 函数
- **Makefile**: 新增 `power_switch` 模块编译；`clean` 从 Windows `rd` 改为 Linux `rm -rf`

### 新增文件
| 文件 | 说明 |
|------|------|
| `doc/vision_communication.md` | 通信数据帧格式文档 (JSON 字段/编码/解析示例) |
| `tools/test_vision_recv.sh` | vision_recv 测试脚本 |
| `modules/power_switch/` | 电源开关模块 (USART/CAN 双路控制) |
| `modules/optical_flow/` | 光流传感器模块 (待集成) |
| `modules/power_meter/` | 功率计模块 (待集成) |

## 2026-05-26 — 通信性能优化 (166 Hz)

### 优化
- **QoS 切换**: vision_send publisher 从 RELIABLE 改为 BEST_EFFORT，频率从 ~1 Hz 提升至 ~166 Hz (166x)
- **JSON 压缩**: 键名缩短 (enemy_color→e, work_mode→w, bullet_speed→b, yaw→y, pitch→p, roll→r, +t 时间戳)
- **整数编码**: yaw/pitch/roll 改用 centi-degree 整数 (×100)，避免 newlib-nano `%f` 空输出问题，单包 < 64 bytes
- **手写 jtoa 格式化器**: 替代 snprintf，无堆分配，零格式化开销

### 修复
- **USB CDC 传输稳定性**: 回退 semaphore 方案为 osDelay(1) 轮询+20次重试，消除多写入者竞态导致的 USB 断连
- **USB 设备识别**: 确认 ttyACM0=JLink VCOM, ttyACM1=STM32 VCP，agent 需连接 ttyACM1

### 文件变更
| 文件 | 变更 |
|------|------|
| `Core/Src/freertos.c` | publisher→BEST_EFFORT, JSON压缩, jtoa, executor 10ms |
| `microros_transport/microros_transport.c` | VCP write 回退轮询+重试 |
| `application/cmd/robot_cmd.c` | MICRO_ROS 路径调用 microros_publish_vision |
| `application/robot_task.h` | 移除 INS 任务中的 vision publish |
| `MICROROS_GUIDE.md` | 更新 JSON 格式、QoS、频率、agent 命令 |

## 2026-05-26 — vision 通信统一 + 原生构建

### 修改
- **vision 通信统一**: `MICRO_ROS_ENABLED` 时 vision 数据完全走 ROS 2 topic，禁用 raw seasky 协议；未启用时回退到 seasky 串口
- **双 topic 模型**:
  - `/stm32h723/vision_send` — 下位机→上位机 (JSON: enemy_color, work_mode, bullet_speed, yaw, pitch, roll)
  - `/stm32h723/vision_recv` — 上位机→下位机 (JSON: fire_mode, target_state, target_type, yaw, pitch, fire_tem)
- **`freertos.c`**: 新增 vision subscriber + JSON 解析回调，publisher 改为 `microros_publish_vision()`
- **`master_process.c`**: seasky 协议用 `#if !defined(MICRO_ROS_ENABLED)` 守卫；新增 `VisionGetSendData()` / `VisionGetRecvData()` accessor
- **`robot_cmd.c`**: `MICRO_ROS_ENABLED` 时调用 `microros_publish_vision()`，否则 `VisionSend()`
- **`library_generation.sh`**: 适配 ELF 原生构建（自动检测工具链/workspace，无需 Docker）

### 新增
- **原生 libmicroros.a 构建**: 安装 `ros-humble-micro-ros-setup` + 本机工具链后直接 `bash library_generation.sh`
- **`MICROROS_GUIDE.md`**: 更新架构图、topic 表、切换说明、原生构建文档

## 2026-05-26 — micro-ROS 集成与 RAM 优化

### 新增
- **micro-ROS 集成**: 通过 USB VCP 或 USART10 DMA 与上位机 ROS 2 通信，publish `std_msgs/msg/String` 到 topic `stm32h723/vision_data`
- **自定义 XRCE 传输层** (`microros_transport/`): 实现 open/close/write/read 回调，支持 VCP 和 UART 双模式切换
- **USB CDC 多回调支持** (`usbd_cdc_if.c`): VCP 可同时服务 vision 通信和 micro-ROS 通信，互不冲突
- **`libmicroros.a` (35.8 MB)**: GCC 14.3 构建，69 个 ROS 2 包，位于 `lib/`
- **`print_cflags` Makefile target**: 供 Docker 构建容器读取项目编译参数

### 修改
- **RAM 优化** (`STM32H723VGTx_FLASH.ld`):
  - 新增 `.dtcmram` 段，将 FreeRTOS 堆 (`ucHeap`) 从 RAM 移至 DTCMRAM (128KB TCM)
  - RAM 使用率: 99.99% → 61.92% (释放约 49KB)
  - DTCMRAM 使用: 0% → 38.07%
- **FreeRTOSConfig.h**: `configTOTAL_HEAP_SIZE` 25600 → 49900 (堆已移入 DTCMRAM 故可安全增大)
- **Makefile**:
  - 新增 `-DMICRO_ROS_ENABLED`、`-DCOMM_USE_VCP` 编译宏
  - 新增 micro-ROS 源文件、头文件路径、链接库
  - GCC 14 兼容选项 (`-Wno-error=incompatible-pointer-types` 等)
  - `MICROROS_TOTAL_HEAP_SIZE=8192` (自定义 micro-ROS 堆, 当前未启用)
- **`freertos.c`**: 新增 `StartMicrorosTask`，初始化 micro-ROS node/publisher/executor
- **`master_process.c`**: vision 通信重构，VCP 模式下取消对硬件 UART handle 的依赖，`VisionInit()` 改为无参
- **`robot_cmd.c`**: 适配新 `VisionInit()` 签名
- **`robot_def.h`**: 通信介质选择以 Makefile `C_DEFS` 为准，头文件仅保留文档注释

### 修复
- **GCC 14.3 构建兼容**: 新编译器将 `-Wincompatible-pointer-types` / `-Wint-conversion` / `-Wimplicit-function-declaration` 升级为 error，通过 Makefile 标志降级（不影响用户原有代码）
- **链接器重定位不兼容**: Docker 内置 GCC 10.3 构建的 `.a` 与宿主机 GCC 14.3 链接器不兼容，改用宿主机工具链挂载至 Docker 重新构建
- **`--whole-archive` 导致多余符号**: 改为 `--start-group/--end-group` 链接方式，避免拉入 test_msgs 等无用对象产生重定位错误

### 文件清单

| 文件 | 变更 |
|---|---|
| `Core/Src/freertos.c` | +98 行 — micro-ROS 任务、publisher |
| `Makefile` | +34 行 — micro-ROS 源文件/头文件/库/编译选项 |
| `STM32H723VGTx_FLASH.ld` | +8 行 — DTCMRAM 段 |
| `Core/Inc/FreeRTOSConfig.h` | 堆大小调整 |
| `USB_DEVICE/App/usbd_cdc_if.c` | +26 行 — USB CDC 多回调支持 |
| `USB_DEVICE/App/usbd_cdc_if.h` | +3 行 — USBCallback typedef、CDCInitRxbufferNcallback |
| `modules/master_machine/master_process.c` | -127/+— Vision 通信重构 |
| `modules/master_machine/master_process.h` | VisionInit 签名变更 |
| `application/cmd/robot_cmd.c` | 适配新 VisionInit |
| `application/robot_def.h` | 通信选择宏整理 |
| `microros_transport/` | **新增** — XRCE 传输层 |
| `micro_ros_stm32cubemx_utils/extra_sources/` | **新增** — 时间/分配器/内存管理 |
| `microros_include/` | **新增** (符号链接) |
| `lib/libmicroros.a` | **新增** — micro-ROS 静态库 (69 包) |
