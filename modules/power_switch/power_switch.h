#ifndef __POWER_SWITCH_H
#define __POWER_SWITCH_H

#include "main.h"
#include <bsp_usart.h>
#include <can_comm.h>
#include "cmsis_os.h"

// 外部变量声明
extern volatile uint8_t switch_flag;
extern uint8_t last_switch_flag;
extern USARTInstance *switch_usart;
extern CANCommInstance *switch_can;

static const uint8_t data_on_usart[4] = {0xAA, 0xEE, 0x55, 0x55};
static const uint8_t data_off_usart[4] = {0xAA, 0xEE, 0xF0, 0xF0};
static const uint8_t data_on_can[2] = {0x55, 0x55};
static const uint8_t data_off_can[2] = {0xF0, 0xF0};

// 初始化函数
void power_switch_init(USARTInstance *usart_instance, CANComm_Init_Config_s *comm_config);

// 开关控制任务
void PowerSwitchTask(void const *argument);

// 创建开关任务（类似GOMotorControlInit）
void PowerSwitchControlInit(void);

// 控制函数
void switch_on(void);
void switch_off(void);

#endif /* __POWER_SWITCH_H */