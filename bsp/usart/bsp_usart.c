/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"
#include <usart.h>
#include "stm32h7xx_hal.h"  // H7系列专属头文件
#include "stm32h7xx_hal_usart.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx,idx_485;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}
/*普通串口初始化*/
USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");
    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");
    /*
    if(init_config->enable_485==0)
    {
        if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");
        for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
            if (usart_instance[i]->usart_handle == init_config->usart_handle)
                while (1)
                    LOGERROR("[bsp_usart] USART instance already registered!");
    }
    else
    {
        if (idx_485 >= DEVICE_485_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");
        for (uint8_t i = 0; i < idx_485; i++) // 检查是否已经注册过
                if (usart485_instance[i]->id == init_config->id&&usart485_instance[i]->usart_handle == init_config->usart_handle)
                    while (1)
                        LOGERROR("[bsp_usart] USART485 ID [%d]already registered!",init_config->id);
    }
    */
    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->id = init_config->id;
    instance->enable_485 = init_config->enable_485;
    usart_instance[idx++] = instance;
    /*
    if(init_config->enable_485==0)
    {
        usart_instance[idx++] = instance;
    }
    else
    {
        usart485_instance[idx_485++] = instance;
    }
    */
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}
/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState & HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /*
    for (uint8_t i = 0; i < idx_485; ++i)
    {
        if (huart == usart485_instance[i]->usart_handle)
        {
            if (usart485_instance[i]->module_callback != NULL)
            {
                usart485_instance[i]->module_callback();
                memset(usart485_instance[i]->recv_buff, 0, Size);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart485_instance[i]->usart_handle, 
                                       usart485_instance[i]->recv_buff, 
                                       usart485_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart485_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
    */
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
    /*
    for (uint8_t i = 0; i < idx_485; ++i)
    {
        if (huart == usart485_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart485_instance[i]->usart_handle, usart485_instance[i]->recv_buff, usart485_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart485_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            LOGWARNING("[bsp_usart] USART485 error callback triggered");
            return;
        }
    }
    */
}
/**
 * @brief  STM32H7 USART专用：动态修改波特率（兼容所有固件库版本）
 * @param  huart: USART句柄指针
 * @param  new_baud: 新波特率（如115200）
 * @retval HAL_StatusTypeDef: 操作结果
 */
HAL_StatusTypeDef USART_H7_SetBaudRateOnly(UART_HandleTypeDef *huart, uint32_t new_baud)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    // -------------------------- 步骤1：保存核心中断状态（移除CSIE） --------------------------
    // CR1：RXNE/TXE/TC/PE中断（通用，所有H7版本都支持）
    uint32_t cr1_it = huart->Instance->CR1 & (USART_CR1_RXNEIE | USART_CR1_TXEIE | 
                                              USART_CR1_TCIE | USART_CR1_PEIE);
    // CR3：错误中断（EIE）+ CTS/RTS流控中断（可选，按需保留）
    uint32_t cr3_it = huart->Instance->CR3 & (USART_CR3_EIE);
    
    // -------------------------- 步骤2：关闭核心中断（无CSIE） --------------------------
    // 关闭CR1中的收发/奇偶错中断
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE | UART_IT_TXE | UART_IT_TC | UART_IT_PE);
    // 关闭CR3中的错误中断
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    
    // -------------------------- 步骤3：修改波特率（核心逻辑不变） --------------------------
    UART_InitTypeDef usart_init_backup = huart->Init;
    usart_init_backup.BaudRate = new_baud;
    huart->Init = usart_init_backup;
    
    __HAL_UART_DISABLE(huart);  // 关闭USART外设
    
    // 等待收发完成，避免数据丢失
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET);
    
    ret = HAL_UART_Init(huart); // 重新初始化
    
    __HAL_UART_ENABLE(huart);   // 重新使能USART外设
    
    // -------------------------- 步骤4：恢复核心中断 --------------------------
    if (cr1_it & USART_CR1_RXNEIE) __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    if (cr1_it & USART_CR1_TXEIE)  __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
    if (cr1_it & USART_CR1_TCIE)   __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
    if (cr1_it & USART_CR1_PEIE)   __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
    if (cr3_it) __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
    
    return ret;
}
