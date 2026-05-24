# 中州烛龙战队 · 2026 步兵狗腿

本项目由**中州烛龙战队**基于原框架修改，用于 2026 赛季 RoboMaster 步兵机器人的底盘（狗腿）控制。

---

## 项目起源

本框架基于 [NCHU 南昌航空大学洪鹰战队](https://gitee.com/LitzJ/basic_framework_mc02) 的开源代码改造而来，同时参考了 [湖南大学跃鹿战队](https://gitee.com/hnuyuelurm/basic_framework) 的基础框架设计。感谢他们的开源贡献。

## 开发环境

| 工具 | 说明 |
|------|------|
| STM32CubeMX | 硬件配置与代码生成 |
| VSCode | 主力编辑器 |
| Ozone | 调试工具 |
| MinGW32 + arm-none-eabi-objcopy | 编译工具链 |

开发流参考：[湖南大学基础框架教学](https://gitee.com/hnuyuelurm/basic_framework)

## 新增功能

- **通信 Buffer**：消除电控与视觉通信时因缺少时间戳导致的相位差，可保存变量 0~20 个周期的历史数据
- **功率限制**：对底盘输出进行功率约束，满足比赛规则要求
