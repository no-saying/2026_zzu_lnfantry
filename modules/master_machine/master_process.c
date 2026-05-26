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

/* ──── vision communication transport selection ──── */

#if defined(MICRO_ROS_ENABLED)
/* micro-ROS handles vision I/O over ROS 2 topics — raw serial disabled */

Vision_Recv_s *VisionInit(void)
{
    return &recv_data;
}

void VisionSend(void)
{
    /* data is published via microros_publish_vision() in RobotCMDTask */
}

#elif defined(COMM_USE_VCP)
/* Raw seasky protocol over USB VCP — original Hero behaviour */

#include "bsp_usb.h"
static uint8_t vis_recv_buff[APP_RX_DATA_SIZE];

static void VisionOfflineCallback(void *id)
{
    recv_data.fire_tem=0;
    LOGWARNING("[vision] vision offline, restart communication.");
}

Vision_Recv_s *VisionInit(void)
{
    USB_Init_Config_s conf = {0};
    USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionRecvPoll(void)
{
    uint32_t len = CDC_ReadRxData(vis_recv_buff, sizeof(vis_recv_buff));
    if (len > 0) {
        uint16_t flag_register;
        DaemonReload(vision_daemon_instance);
        get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.yaw);
    }
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

#else
#error "No communication transport defined! Define MICRO_ROS_ENABLED or COMM_USE_VCP in Makefile C_DEFS."
#endif
