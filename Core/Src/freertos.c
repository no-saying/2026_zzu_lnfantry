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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/custom_transport.h>
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
  osThreadDef(microrosTask, StartMicrorosTask, osPriorityAboveNormal, 0, 2048);
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
  MX_USB_DEVICE_Init();
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

    /* parse JSON: {"fire_mode":N,"target_state":N,"target_type":N,"yaw":F,"pitch":F,"fire_tem":N} */
    char buf[256];
    size_t n = in->data.size < sizeof(buf) - 1 ? in->data.size : sizeof(buf) - 1;
    memcpy(buf, in->data.data, n);
    buf[n] = '\0';

    sscanf(buf, "{\"fire_mode\":%d,\"target_state\":%d,\"target_type\":%d,\"yaw\":%f,\"pitch\":%f,\"fire_tem\":%hhu}",
           (int *)&recv->fire_mode,
           (int *)&recv->target_state,
           (int *)&recv->target_type,
           &recv->yaw,
           &recv->pitch,
           &recv->fire_tem);
}

void StartMicrorosTask(void const *argument)
{
    (void)argument;

    /* 1. register custom transport */
    rmw_uros_set_custom_transport(
        false, NULL,
        microros_transport_open,
        microros_transport_close,
        microros_transport_write,
        microros_transport_read
    );

    /* 2. init micro-ROS */
    microros_allocator = rcl_get_default_allocator();

    if (rclc_support_init(&microros_support, 0, NULL, &microros_allocator) != RCL_RET_OK) {
        for (;;) { osDelay(1000); }
    }
    rclc_node_init_default(&microros_node, "stm32h723_node", "", &microros_support);

    /* 3. publisher: STM32 → vision computer (gimbal attitude + flags) */
    rclc_publisher_init_default(
        &microros_pub_send,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "stm32h723/vision_send"
    );

    /* 4. subscriber: vision computer → STM32 (target info + fire) */
    rclc_subscription_init_default(
        &microros_sub_recv,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "stm32h723/vision_recv"
    );

    /* 5. executor — handles subscription callbacks */
    rclc_executor_init(&microros_executor, &microros_support.context, 2, &microros_allocator);
    rclc_executor_add_subscription(
        &microros_executor, &microros_sub_recv, &microros_recv_msg,
        microros_vision_recv_cb, ON_NEW_DATA
    );

    /* 6. pre-allocate send message */
    microros_send_msg.data.data = (char *)malloc(256);
    microros_send_msg.data.size = 0;
    microros_send_msg.data.capacity = 256;

    for (;;) {
        rclc_executor_spin_some(&microros_executor, RCL_MS_TO_NS(5));
        osDelay(1);
    }
}

/* publish gimbal attitude + flags to vision computer (JSON over std_msgs/String) */
rcl_ret_t microros_publish_vision(void)
{
    Vision_Send_s *send = VisionGetSendData();
    if (microros_send_msg.data.data == NULL) return RCL_RET_ERROR;

    int n = snprintf(microros_send_msg.data.data, microros_send_msg.data.capacity,
        "{\"enemy_color\":%d,\"work_mode\":%d,\"bullet_speed\":%d,\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f}",
        send->enemy_color, send->work_mode, send->bullet_speed,
        send->yaw, send->pitch, send->roll);
    if (n < 0) return RCL_RET_ERROR;
    microros_send_msg.data.size = n;

    return rcl_publish(&microros_pub_send, &microros_send_msg, NULL);
}

#else /* MICRO_ROS_ENABLED not defined — stub task */

void StartMicrorosTask(void const *argument)
{
    (void)argument;
    for (;;) { osDelay(1000); }
}

#endif /* MICRO_ROS_ENABLED */

/* USER CODE END Application */
