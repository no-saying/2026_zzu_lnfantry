#include "power_switch.h"
#include "memory.h"
#include "string.h"

// 全局变量定义
volatile uint8_t switch_flag = 0;
uint8_t last_switch_flag=0;
USARTInstance *switch_usart = NULL;
CANCommInstance *switch_can = NULL;

// 任务句柄
static osThreadId switch_task_handle = NULL;

// 发送数据

// 初始化（保持原有函数不变）
void power_switch_init(USARTInstance *usart_instance, CANComm_Init_Config_s *comm_config)
{
    if(usart_instance != NULL)
        switch_usart = usart_instance;
    else if(comm_config != NULL)
        switch_can = CANCommInit(comm_config);
    
    // 初始化标志为关闭状态
    switch_flag = 0;
}

// 开关任务（参考GOMotorTask风格）
void PowerSwitchTask(void const *argument)
{
    // 初始延时（参考GOMotorTask的8ms）
    HAL_Delay(8);
    
    uint8_t last_switch_state = 0;
    
    // 初始发送关闭命令
    if(switch_can != NULL)
    {
        CANCommSend(switch_can, data_off_can, CAN_DATA_UINT8_ARRAY);
    }
    if(switch_usart != NULL)
    {
        USARTSend(switch_usart, data_off_usart, 4, USART_TRANSFER_DMA);
    }
    
    while (1)
    {
        // 检查开关状态是否变化
        //if(last_switch_state != switch_flag)
        {
            if(switch_flag == 1)
            {
                // 发送开启命令
                if(switch_can != NULL)
                {
                    CANCommSend(switch_can, data_on_can, CAN_DATA_UINT8_ARRAY);
                }
                if(switch_usart != NULL)
                {
                    USARTSend(switch_usart, data_on_usart, 4, USART_TRANSFER_DMA);
                }
            }
            else
            {
                // 发送关闭命令
                if(switch_can != NULL)
                {
                    CANCommSend(switch_can, data_off_can, CAN_DATA_UINT8_ARRAY);
                }
                if(switch_usart != NULL)
                {
                    USARTSend(switch_usart, data_off_usart, 4, USART_TRANSFER_DMA);
                }
            }
            
            last_switch_state = switch_flag;
        }
        
        // 任务延时（参考GOMotorTask的1ms + osDelay(4)）
        HAL_Delay(1);
        osDelay(4);
    }
}

// 创建开关任务（参考GOMotorControlInit风格）
void PowerSwitchControlInit(void)
{
    // 只有初始化了USART或CAN才创建任务
    if(switch_usart == NULL && switch_can == NULL)
        return;
    
    // 创建任务（参考您的命名方式）
    char switch_task_name[] = "switch";
    
    // 使用与GOMotorControlInit相同的任务创建方式
    osThreadDef(switch_task_name, PowerSwitchTask, osPriorityNormal, 0, 128);
    switch_task_handle = osThreadCreate(osThread(switch_task_name), NULL);
}

// 控制函数（保持原有接口）
void switch_on(void)
{
    switch_flag = 1;
    //USARTSend(switch_usart, data_on_usart, 4, USART_TRANSFER_DMA);
}

void switch_off(void)
{
    switch_flag = 0;
    //USARTSend(switch_usart, data_off_usart, 4, USART_TRANSFER_DMA);
}