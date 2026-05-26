/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "microros_transport.h"
#ifdef MICRO_ROS_ENABLED
#include "usb_device.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/time_sync.h>
#include <std_msgs/msg/string.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId microrosTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMicrorosTask(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#ifdef MICRO_ROS_ENABLED
  osThreadDef(microrosTask, StartMicrorosTask, osPriorityNormal, 0, 8192);
  microrosTaskHandle = osThreadCreate(osThread(microrosTask), NULL);
#endif
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
#ifndef MICRO_ROS_ENABLED
  MX_USB_DEVICE_Init();
#endif
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#ifdef MICRO_ROS_ENABLED

#include "master_process.h"
#include <stdio.h>
#include <string.h>

static rcl_allocator_t microros_allocator;
static rclc_support_t microros_support;
static rcl_node_t microros_node;
static rcl_publisher_t microros_pub_send;
static rcl_subscription_t microros_sub_recv;
static rclc_executor_t microros_executor;
static std_msgs__msg__String microros_send_msg;
static std_msgs__msg__String microros_recv_msg;

/* ──── vision receive callback (called by executor when agent forwards data) ──── */
static void microros_vision_recv_cb(const void *msgin)
{
    const std_msgs__msg__String *in = (const std_msgs__msg__String *)msgin;
    if (in == NULL || in->data.data == NULL || in->data.size == 0) return;

    Vision_Recv_s *recv = VisionGetRecvData();

    /* parse JSON: {"f":N,"s":N,"t":N,"y":N,"p":N,"m":N}  yaw/pitch in centi-degrees */
    char buf[128];
    size_t n = in->data.size < sizeof(buf) - 1 ? in->data.size : sizeof(buf) - 1;
    memcpy(buf, in->data.data, n);
    buf[n] = '\0';

    int fm, ts, tt, yaw_cdeg, pitch_cdeg, fire_tem;
    if (sscanf(buf, "{\"f\":%d,\"s\":%d,\"t\":%d,\"y\":%d,\"p\":%d,\"m\":%d}",
               &fm, &ts, &tt, &yaw_cdeg, &pitch_cdeg, &fire_tem) == 6) {
        recv->fire_mode = (Fire_Mode_e)fm;
        recv->target_state = (Target_State_e)ts;
        recv->target_type = (Target_Type_e)tt;
        recv->yaw = yaw_cdeg / 100.0f;
        recv->pitch = pitch_cdeg / 100.0f;
        recv->fire_tem = (uint8_t)fire_tem;
    }
}

void StartMicrorosTask(void const *argument)
{
    (void)argument;

    /* 0. ensure USB CDC is initialised (defaultTask may not have run yet) */
    MX_USB_DEVICE_Init();
    osDelay(200);  /* wait for USB enumeration on host side */

    /* 1. register custom transport */
    rmw_uros_set_custom_transport(
        true, NULL,
        microros_transport_open,
        microros_transport_close,
        microros_transport_write,
        microros_transport_read
    );

    /* 2. init micro-ROS with retry (agent may not be running yet) */
    microros_allocator = rcl_get_default_allocator();

    rcl_ret_t rc = RCL_RET_ERROR;
    while (rc != RCL_RET_OK) {
        rc = rclc_support_init(&microros_support, 0, NULL, &microros_allocator);
        if (rc == RCL_RET_OK) break;
        osDelay(500);
    }

    rclc_node_init_default(&microros_node, "stm32h723_node", "", &microros_support);

    /* 3. publisher: STM32 → vision computer (gimbal attitude + flags) */
    rclc_publisher_init_best_effort(
        &microros_pub_send,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "stm32h723/vision_send"
    );

    /* 4. subscriber: vision computer → STM32 (target info + fire) */
    rclc_subscription_init_best_effort(
        &microros_sub_recv,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "stm32h723/vision_recv"
    );

    /* 5. pre-allocate message buffers (required for micro-ROS string deserialization) */
    microros_send_msg.data.data = (char *)malloc(256);
    microros_send_msg.data.size = 0;
    microros_send_msg.data.capacity = 256;

    microros_recv_msg.data.data = (char *)malloc(256);
    microros_recv_msg.data.size = 0;
    microros_recv_msg.data.capacity = 256;

    /* 6. executor — handles subscription callbacks */
    rclc_executor_init(&microros_executor, &microros_support.context, 2, &microros_allocator);
    rclc_executor_add_subscription(
        &microros_executor, &microros_sub_recv, &microros_recv_msg,
        microros_vision_recv_cb, ON_NEW_DATA
    );

    int cycle = 0;
    for (;;) {
        if (++cycle >= 2) {
            cycle = 0;
            rclc_executor_spin_some(&microros_executor, RCL_MS_TO_NS(1));
        }
        microros_publish_vision();
        osDelay(1);
    }
}

/* publish gimbal attitude + flags to vision computer (JSON over std_msgs/String) */
/* floats encoded as centi-degrees (x100); hand-rolled formatter avoids snprintf overhead */
static int jtoa(char *dst, int val)
{
    if (val == 0) { dst[0] = '0'; return 1; }
    char tmp[12], *p = tmp;
    int neg = val < 0;
    if (neg) val = -val;
    while (val) { *p++ = '0' + (val % 10); val /= 10; }
    char *q = dst;
    if (neg) *q++ = '-';
    while (p > tmp) *q++ = *--p;
    return q - dst;
}

rcl_ret_t microros_publish_vision(void)
{
    Vision_Send_s *send = VisionGetSendData();
    char *buf = microros_send_msg.data.data;
    if (buf == NULL) return RCL_RET_ERROR;

    char *p = buf;
    memcpy(p, "{\"e\":", 5); p += 5;
    p += jtoa(p, (int)send->enemy_color);
    memcpy(p, ",\"w\":", 5); p += 5;
    p += jtoa(p, (int)send->work_mode);
    memcpy(p, ",\"b\":", 5); p += 5;
    p += jtoa(p, (int)send->bullet_speed);
    memcpy(p, ",\"y\":", 5); p += 5;
    p += jtoa(p, (int)(send->yaw * 100.0f));
    memcpy(p, ",\"p\":", 5); p += 5;
    p += jtoa(p, (int)(send->pitch * 100.0f));
    memcpy(p, ",\"r\":", 5); p += 5;
    p += jtoa(p, (int)(send->roll * 100.0f));
    memcpy(p, ",\"t\":", 5); p += 5;
    p += jtoa(p, (int)HAL_GetTick());
    *p++ = '}';

    microros_send_msg.data.size = p - buf;

    rcl_ret_t ret = rcl_publish(&microros_pub_send, &microros_send_msg, NULL);
    return ret;
}

#else /* MICRO_ROS_ENABLED not defined — stub task */

void StartMicrorosTask(void const *argument)
{
    (void)argument;
    for (;;) { osDelay(1000); }
}

#endif /* MICRO_ROS_ENABLED */

/* USER CODE END Application */
