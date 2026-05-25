/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

Vision_Recv_s recv_data;
Vision_Send_s send_data;
#ifdef COMM_USE_VCP
static DaemonInstance *vision_daemon_instance;
#endif

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/* expose send/recv buffers for micro-ROS transport */
Vision_Send_s *VisionGetSendData(void) { return &send_data; }
Vision_Recv_s *VisionGetRecvData(void) { return &recv_data; }

/* ──── raw seasky serial (disabled when micro-ROS is enabled) ──── */
#if defined(COMM_USE_VCP) && !defined(MICRO_ROS_ENABLED)

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void VisionOfflineCallback(void *id)
{
    recv_data.fire_tem=0;
    LOGWARNING("[vision] vision offline, restart communication.");
}

static void DecodeVision(uint16_t len)
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance);
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.yaw);
}

Vision_Recv_s *VisionInit(void)
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(void)
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    flag_register = 30 << 8 | 0b00000001;
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#elif defined(MICRO_ROS_ENABLED)
/* micro-ROS handles vision I/O — raw serial disabled */

Vision_Recv_s *VisionInit(void)
{
    return &recv_data;
}

void VisionSend(void)
{
    /* data published via microros_publish_vision() called in RobotCMDTask */
}

#endif /* COMM_USE_VCP && !MICRO_ROS_ENABLED */
