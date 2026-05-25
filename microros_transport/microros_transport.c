#include "microros_transport.h"

#if defined(COMM_USE_VCP)

/* ====== USB CDC (VCP) transport via bsp_usb ====== */
#include "bsp_usb.h"
#include "cmsis_os.h"
#include <string.h>

#define MROS_RX_BUF_MAX 512

static uint8_t rx_buf[MROS_RX_BUF_MAX];
static volatile size_t rx_len = 0;
static SemaphoreHandle_t rx_sem = NULL;

static void microros_usb_rx_cbk(uint16_t len)
{
    extern uint8_t UserRxBufferHS[];
    size_t n = (len < MROS_RX_BUF_MAX) ? len : MROS_RX_BUF_MAX;
    memcpy(rx_buf, UserRxBufferHS, n);
    rx_len = n;
    if (rx_sem != NULL) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(rx_sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

bool microros_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    rx_sem = xSemaphoreCreateBinary();
    if (rx_sem == NULL) return false;

    USB_Init_Config_s conf = {.rx_cbk = microros_usb_rx_cbk};
    USBInit(conf);
    return true;
}

bool microros_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
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
    if (len > APP_TX_DATA_SIZE) len = APP_TX_DATA_SIZE;
    USBTransmit((uint8_t *)buf, (uint16_t)len);
    return len;
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
    return n;
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
