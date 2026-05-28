#ifndef GO_MOTOR_H
#define GO_MOTOR_H

#include "bsp_usart.h"
#include "motor_def.h"
#include "ins_task.h"
#include <stdint.h>
#include "daemon.h"

#define GO_MOTOR_CNT 16
#define DEVICE_485_CNT 16     // 485总线负载数量16个

// 修复：补充枚举类型名，解决空声明警告
typedef enum MotorMode_e
{
    lock,
    foc,
    cali
} MotorMode_e;

typedef struct SetMode
{
    float32_t tqr_ft;
    float32_t spd_des;
    float32_t pos_des;
    float32_t Kp;
    float32_t Kd;
    uint8_t ID;
    uint8_t Mode;
}Motor_Conf;

typedef struct messsure
{
    float32_t tqr_fb;
    float32_t rot_spd;
    float32_t pos;
    int8_t tem;
    uint8_t wrong;
}Motor_messure;

#pragma pack(1)
typedef struct motor_control
{
    uint8_t head[2];
    uint8_t Set;
    int16_t tqr_set;
    int16_t spd_set;
    int32_t pos_set;
    int16_t Kpos;
    int16_t Kspd;
    uint16_t crc;
}motor_ctrl;
#pragma pack(0)

// 目前仅跳跃
typedef struct Control_Setting_s
{
    float32_t Kpos;
    float32_t Kspd;
    float32_t pos_comm;
    float32_t jump_rad;
    int8_t motor_reverse_flag;
    int8_t motor_position_flag;
    float32_t k_accel_x,k_accel_y;
    float32_t debug_view;//仅调试用，不调试勿用
}GOMotor_Control_Setting_s;

typedef struct GOMotorUSARTInstance
{
    USARTInstance *motor_usart;
    DaemonInstance *daemon;
}GOMotorUSARTInstance;

// 修复：统一 GOMotorInstance 结构体定义（与 .c 初始化顺序、成员完全匹配）
typedef struct GOMotorInstance
{
    Motor_Conf ctrl_set;                  // 控制配置（与 .c 初始化顺序一致）
    GOMotor_Control_Setting_s controller; // 内置PID配置（关键：移到前面，与 .c 赋值顺序匹配）
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    uint8_t enable_pid;
    uint8_t enable_accel_feedback;
    Motor_messure mes;                      // 测量值
    motor_ctrl ctrl_send;                   // 发送数据
    Motor_Type_e motor_type;                // 电机类型
    Motor_Working_Type_e stop_flag;         // 启停标志
    uint8_t rs485_flag;                     // 485通信标志
    unsigned int feed_cnt;                  // 反馈计数器
    float dt;                               // 时间间隔
    float accel_filtered;   // ⭐ 每个电机独立滤波
}GOMotorInstance;

typedef struct
{
    Motor_Conf conf;
    Motor_Type_e motor_type;
    USART_Init_Config_s usart_init_config;
    GOMotor_Control_Setting_s controller_setting_init_config;
    Motor_Controller_Init_s c_ontroller_param_init_config;
    Motor_Control_Setting_s c_ontroller_setting_init_config;
} Other_Motor_Init_Config_s;

// 修复：补充成员末尾分号，解决结构体末尾无分号警告
typedef struct go_motor
{
    float *x_accel_feedback_ptr,*y_accel_feedback_ptr,*z_accel_feedback_ptr;
    float *pitch,*roll;
    float x_accel,y_accel;
    int8_t *v_flag;//速度方向
}Chassis_accel_ptr;

// 补充缺失的函数声明（避免编译警告）
void init_accel_ptr(attitude_t *imu_data,int8_t *v_flag);
void GOMotorEnable(GOMotorInstance* _instance);
void GOMotorStop(GOMotorInstance* _instance);
void GOMotorSetSpeed(GOMotorInstance* _instance, float32_t speed_rad_s,float32_t kd);
void GOMotorSetPosition(GOMotorInstance* _instance, float32_t position_rad,float32_t kp);
void GOMotorSetTorque(GOMotorInstance* _instance, float torque_nm);
void GOMotorcontrol(GOMotorInstance* _instance);
GOMotorInstance *GOmotorInit(Other_Motor_Init_Config_s config);
void GoMotorSetRef(GOMotorInstance *motor, float ref);
void GOMotorControlInit(void);

#endif // GO_MOTOR_H