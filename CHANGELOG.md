# 更新日志

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
