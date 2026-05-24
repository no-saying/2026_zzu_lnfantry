# 中州烛龙战队 — 步兵机器人电控框架

本项目由**中州烛龙战队**基于湖南大学（湖大）开源基础框架改造而来，针对 RoboMaster 步兵机器人进行了深度适配和优化。

<p align='right'>743009492@qq.com</p>

## 项目来源

本框架的演进链路：
- **湖大开源基础框架**：https://gitee.com/hnuyuelurm/basic_framework — 提供了完整的 STM32CubeMX + VSCode + Ozone 开发流教学和框架设计思想
- **NCHU 南昌航空大学洪鹰战队**：https://gitee.com/LitzJ/basic_framework_mc02 — 基于湖大框架的 MC02 硬件平台实现

本仓库在以上开源项目的基础上，针对中州烛龙战队的步兵机器人进行了定制化改造，包括底盘运动控制、云台控制、射击控制、功率管理等模块的参数调整和功能优化。

## 硬件平台

| 项目     | 型号/参数                        |
| -------- | -------------------------------- |
| 主控     | STM32H723VGTx (Cortex-M7, 550MHz) |
| IMU      | BMI088 (SPI)                     |
| 电机     | DJI M3508 / M2006 / GM6020       |
| 电调     | C620 / C610                      |
| 通信     | CAN × 2, USART × 3, USB (CDC)    |
| 调试     | SEGGER RTT (J-Link / CMSIS-DAP)  |

## 软件架构

```
application/    应用层 — 机器人行为逻辑 (cmd / gimbal / chassis / shoot)
  ├── robot_cmd    机器人大脑，接收遥控器/视觉指令，转化为控制目标
  ├── gimbal       yaw/pitch 双轴云台控制
  ├── chassis      麦克纳姆轮运动学解算 + 超级电容功率限制
  └── shoot        摩擦轮 + 拨盘 + 弹舱盖控制
modules/        模块层 — 功能模块 (motor / imu / referee / remote ...)
bsp/            板级驱动 — 硬件抽象 (can / spi / usart / pwm / i2c / gpio ...)
Core/           STM32CubeMX 生成代码 + FreeRTOS
```

### 核心特性

- **发布-订阅通信** — 基于 Message Center 的解耦架构，应用间通过话题发布/订阅进行数据交换，无全局变量、无相互包含
- **双板兼容** — 通过条件编译支持单板 (`ONE_BOARD`)、云台板 (`GIMBAL_BOARD`)、底盘板 (`CHASSIS_BOARD`) 三种模式，板间通过 CAN 通信
- **多模式云台** — 支持遥控器、键鼠、视觉自瞄三种控制模式，可切换 IMU 陀螺仪反馈或电机编码器反馈
- **多闭环电机控制** — 电流环 / 速度环 / 位置环串级 PID，支持积分分离、微分先行、输出滤波等优化
- **IMU 姿态解算** — 基于四元数 EKF 的 1kHz 姿态估计，提供 yaw/pitch/roll 全姿态数据
- **功率管理** — 裁判系统功率限制 + 超级电容辅助供电，实时动态限幅
- **守护进程** — Daemon 模块监测各模块在线状态，离线自动进入安全模式
- **Buffer 工具** — 消除电控-视觉通信中因时间戳缺失导致的相位差，可保存 0~20 周期历史数据
- **日志系统** — 基于 SEGGER RTT 的三级日志 (INFO / WARNING / ERROR)，支持浮点数输出

### FreeRTOS 任务分配

| 任务          | 频率      | 优先级           | 职责                 |
| ------------- | --------- | ---------------- | -------------------- |
| INS Task      | 1 kHz     | Above Normal     | IMU 姿态解算         |
| Motor Task    | 1 kHz     | Normal           | 电机闭环计算 + CAN 发送 |
| Robot Task    | 200 Hz    | Normal           | 整车应用逻辑总调度   |
| Daemon Task   | 100 Hz    | Normal           | 模块离线检测 + 蜂鸣器 |
| UI Task       | 按需      | Normal           | 裁判系统 UI 刷新     |

## 开发环境

### 工具链

| 工具          | 用途              |
| ------------- | ----------------- |
| STM32CubeMX   | 硬件初始化代码生成 |
| VSCode        | 代码编辑与调试     |
| Ozone         | 图形化调试 (J-Link) |
| arm-none-eabi-gcc | 交叉编译       |
| OpenOCD / J-Flash | 固件烧录       |

### 快速开始

```bash
# 1. 编译
make

# 2. 烧录 (DAP-Link)
make download_dap

# 3. 烧录 (J-Link)
make download_jlink

# 4. 清理
make clean
```

### 调试

使用 VSCode + Ozone 进行调试，通过 `launch.json` 的 `debug-jlink` 任务启动。RTT 日志输出在 Ozone 的 Console / Terminal 中查看。

## 机器人参数配置

所有机器人参数集中在 [application/robot_def.h](application/robot_def.h) 中，**更换机器人前必须先核对并修改**：

- 云台参数：yaw 归中编码器值、pitch 水平编码器值、限位角度
- 底盘参数：轴距、轮距、轮半径、减速比
- 发射参数：拨盘单发角度、减速比、载弹量
- 板级定义：`GIMBAL_BOARD` / `CHASSIS_BOARD` / `ONE_BOARD`

## 目录结构

```
├── application/        应用层
│   ├── cmd/            robot_cmd (大脑)
│   ├── gimbal/         云台控制
│   ├── chassis/        底盘控制 + 功率限制
│   ├── shoot/          发射控制
│   ├── robot.c/h       整车抽象
│   ├── robot_def.h     机器人参数定义 ★
│   └── robot_task.h    RTOS 任务初始化
├── modules/            功能模块
│   ├── algorithm/      PID, Kalman, EKF, LQR, CRC
│   ├── motor/          DJI/HT/LK/GO/DM/舵机/步进 电机驱动
│   ├── imu/            INS 姿态解算 + BMI088 驱动
│   ├── referee/        裁判系统通信 + UI
│   ├── remote/         遥控器/键鼠 数据解析
│   ├── message_center/ 发布-订阅消息中心
│   ├── daemon/         守护进程
│   ├── super_cap/      超级电容通信
│   ├── can_comm/       板间 CAN 通信
│   ├── master_machine/ 上位机通信 (Seasky 协议)
│   ├── buffer/         数据缓冲工具
│   ├── alarm/          蜂鸣器
│   ├── BMI088/         BMI088 传感器驱动
│   └── bluetooth/      HC05 蓝牙
├── bsp/                板级驱动
│   ├── can/            CAN 驱动
│   ├── spi/            SPI 驱动
│   ├── usart/          串口驱动
│   ├── iic/            I2C 驱动
│   ├── pwm/            PWM 驱动
│   ├── gpio/           GPIO 驱动
│   ├── adc/            ADC 驱动
│   ├── dwt/            延时/计时
│   ├── flash/          片上 Flash
│   ├── log/            RTT 日志
│   └── usb/            USB CDC
├── Core/               STM32CubeMX 生成
├── Drivers/            HAL 库 + CMSIS
├── Middlewares/        FreeRTOS, USB, DSP, SEGGER RTT
├── Makefile            编译脚本
└── Basic_Framework_MC02.ioc  CubeMX 工程
```

## 更多文档

各模块的详细说明见对应目录下的 `.md` 文件：
- [APP 层应用编写指引](application/APP层应用编写指引.md)
- [application 总览](application/application.md)
- [robot_cmd](application/cmd/robot_cmd.md)
- [gimbal](application/gimbal/gimbal.md)
- [chassis](application/chassis/chassis.md)
- [shoot](application/shoot/shoot.md)
- [module 总览](modules/module.md)
- [bsp 总览](bsp/bsp.md)
