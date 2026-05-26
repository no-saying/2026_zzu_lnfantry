/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

void USBInit(USB_Init_Config_s usb_conf)
{
    (void)usb_conf;
    LOGINFO("USB init success");
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_HS(buffer, len); // 发送
}
