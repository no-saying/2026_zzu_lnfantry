#include "microros_transport.h"

#if defined(COMM_USE_VCP)

/* ====== USB CDC (VCP) transport via bsp_usb (polling) ====== */
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include <string.h>

static bool transport_opened = false;

bool microros_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    if (transport_opened) return true;

    USB_Init_Config_s conf = {0};
    USBInit(conf);
    transport_opened = true;
    return true;
}

bool microros_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    transport_opened = false;
    return true;
}

size_t microros_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)transport;
    if (errcode != NULL) *errcode = 0;
    if (len > APP_TX_DATA_SIZE) len = APP_TX_DATA_SIZE;

    int retries = 20;
    uint8_t ret;
    while (retries--) {
        ret = CDC_Transmit_HS((uint8_t *)buf, (uint16_t)len);
        if (ret == USBD_OK) return len;
        if (ret == USBD_BUSY) {
            osDelay(1);
            continue;
        }
        break;
    }

    if (errcode != NULL) *errcode = 1;
    return 0;
}

size_t microros_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *errcode)
{
    (void)transport;
    if (errcode != NULL) *errcode = 0;

    uint32_t n;
    int elapsed = 0;

    while ((n = CDC_ReadRxData(buf, (uint32_t)len)) == 0) {
        if (timeout_ms >= 0 && elapsed >= timeout_ms) return 0;
        osDelay(1);
        elapsed++;
    }

    return (size_t)n;
}

#else /* COMM_USE_UART (default) — USART10 DMA */

/* ====== USART10 DMA transport ====== */
#include "usart.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#define MROS_UART       USART10
#define MROS_RX_BUF_MAX 512

static uint8_t rx_buf[MROS_RX_BUF_MAX];
static volatile size_t rx_len = 0;
static SemaphoreHandle_t rx_sem = NULL;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == MROS_UART) {
        rx_len = size;
        if (rx_sem != NULL) {
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(rx_sem, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}

bool microros_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    rx_sem = xSemaphoreCreateBinary();
    if (rx_sem == NULL) return false;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buf, MROS_RX_BUF_MAX);
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    return true;
}

bool microros_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    HAL_UART_DMAStop(&huart10);
    if (rx_sem != NULL) {
        vSemaphoreDelete(rx_sem);
        rx_sem = NULL;
    }
    return true;
}

size_t microros_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)transport;
    if (errcode != NULL) *errcode = 0;
    HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart10, (uint8_t *)buf, len, 100);
    return (ret == HAL_OK) ? len : 0;
}

size_t microros_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *errcode)
{
    (void)transport;
    if (errcode != NULL) *errcode = 0;
    if (rx_sem == NULL) return 0;
    if (xSemaphoreTake(rx_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return 0;
    }
    size_t n = (rx_len < len) ? rx_len : len;
    memcpy(buf, rx_buf, n);
    rx_len = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buf, MROS_RX_BUF_MAX);
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    return n;
}

#endif /* COMM_USE_VCP / COMM_USE_UART */
