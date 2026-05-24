#include "mtf01.h"
#include "daemon.h"
#include "bsp_log.h"
#include <stdlib.h>

// 静态变量定义
static MTF01_Recv_s recv_data;
static USARTInstance *mtf01_usart_instance = NULL;
static DaemonInstance *mtf01_daemon_instance = NULL;
static void (*mtf01_application_callback)(void) = NULL;

// 解析状态机
static MICOLINK_MSG_t parse_msg;

// 添加静态函数声明
static uint8_t micolink_check_sum(MICOLINK_MSG_t* msg);
static uint8_t micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);
/*
 * 离线回调函数
 */
static void MTF01OfflineCallback(void *id)
{
    if (mtf01_usart_instance != NULL)
    {
        USARTServiceInit(mtf01_usart_instance);
    }
    LOGWARNING("[MTF01] Sensor offline, restart communication.");
}

/*
 * 串口接收回调函数
 */
static void DecodeMTF01Data()
{
    if (mtf01_usart_instance == NULL) return;
    
    // 获取接收到的数据长度
    uint16_t recv_size = mtf01_usart_instance->recv_buff_size;
    
    // 逐个字节解析
    for (uint16_t i = 0; i < recv_size; i++)
    {
        //if (mtf01_usart_instance->recv_buff[i] == 0x00) break; // 跳过空数据
        
        // 解析单个字节
        if (micolink_parse_char(&parse_msg, mtf01_usart_instance->recv_buff[i]))
        {
            // 成功解析一帧数据，喂狗并处理数据
            DaemonReload(mtf01_daemon_instance);
            
            // 处理数据
            switch(parse_msg.msg_id)
            {
                case MICOLINK_MSG_ID_RANGE_SENSOR:
                {
                    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
                    memcpy(&payload, parse_msg.payload, parse_msg.len);
                    
                    // 填充接收数据结构
                    recv_data.timestamp = payload.time_ms;
                    recv_data.distance = payload.distance;
                    recv_data.strength = payload.strength;
                    recv_data.precision = payload.precision;
                    recv_data.tof_status = payload.tof_status;
                    recv_data.flow_vel_x = payload.flow_vel_x;
                    recv_data.flow_vel_y = payload.flow_vel_y;
                    recv_data.flow_quality = payload.flow_quality;
                    recv_data.flow_status = payload.flow_status;
                    recv_data.is_valid = (payload.distance >= 10); // 距离大于等于10mm才有效
                    
                    // 调用应用回调
                    if (mtf01_application_callback != NULL)
                    {
                        mtf01_application_callback();
                    }
                    
                    // 可选：输出调试信息
                    LOGINFO("[MTF01] Distance: %dmm, Flow: (%d, %d), Valid: %d",
                           recv_data.distance, recv_data.flow_vel_x, 
                           recv_data.flow_vel_y, recv_data.is_valid);
                    break;
                }
                default:
                    LOGWARNING("[MTF01] Unknown message ID: 0x%02X", parse_msg.msg_id);
                    break;
            }
        }
    }
    
    // 清空接收缓冲区（由HAL回调完成，这里不需要）
}

/*
 * 初始化MTF01传感器
 */
MTF01_Recv_s *MTF01Init(UART_HandleTypeDef *_handle, void (*application_callback)(void))
{
    // 初始化接收数据结构
    memset(&recv_data, 0, sizeof(MTF01_Recv_s));
    memset(&parse_msg, 0, sizeof(MICOLINK_MSG_t));
    
    // 注册串口实例
    USART_Init_Config_s conf;
    conf.id = 0;
    conf.enable_485 = 0;
    conf.module_callback = DecodeMTF01Data;
    conf.recv_buff_size = MICOLINK_MAX_LEN; // 使用最大帧长度作为缓冲区大小
    conf.usart_handle = _handle;
    
    mtf01_usart_instance = USARTRegister(&conf);
    
    if (mtf01_usart_instance == NULL)
    {
        LOGERROR("[MTF01] Failed to register USART instance");
        return NULL;
    }
    
    // 保存应用回调
    mtf01_application_callback = application_callback;
    
    // 注册看门狗
    Daemon_Init_Config_s daemon_conf = {
        .callback = MTF01OfflineCallback,
        .owner_id = mtf01_usart_instance,
        .reload_count = 200, // 根据实际通信频率调整
    };
    
    mtf01_daemon_instance = DaemonRegister(&daemon_conf);
    
    if (mtf01_daemon_instance == NULL)
    {
        LOGWARNING("[MTF01] Failed to register daemon instance");
    }
    
    LOGINFO("[MTF01] Initialized successfully");
    return &recv_data;
}

/*
 * 字节解析状态机
 */
uint8_t micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:     // 帧头
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;
        
    case 1:     // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;
    
    case 2:     // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;
    
    case 3:     // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;
    
    case 4:     // 包序列
        msg->seq = data;
        msg->status++;
        break;
    
    case 5:     // 负载长度
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
        {
            msg->status = 0;
            msg->payload_cnt = 0;
            LOGWARNING("[MTF01] Payload length too large: %d", msg->len);
        }
        else
            msg->status++;
        break;
        
    case 6:     // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt >= msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;
        
    case 7:     // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return 1; // 解析成功
        }
        else
        {
            LOGWARNING("[MTF01] Checksum error");
            return 0;
        }
        
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }
    
    return 0;
}

/*
 * 校验和验证
 */
static uint8_t micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6; // head + dev_id + sys_id + msg_id + seq + len
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;
    
    // 复制头部和payload
    temp[0] = msg->head;
    temp[1] = msg->dev_id;
    temp[2] = msg->sys_id;
    temp[3] = msg->msg_id;
    temp[4] = msg->seq;
    temp[5] = msg->len;
    
    if (msg->len > 0)
    {
        memcpy(&temp[6], msg->payload, msg->len);
    }
    
    // 计算校验和
    for(uint8_t i = 0; i < length; i++)
    {
        checksum += temp[i];
    }
    
    return (checksum == msg->checksum);
}