#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "controller.h"
#include "motor_def.h"

/* 各型号 DM 电机限制参数 (p_max, v_max, t_max) */
const DM_Motor_Limit_s dm_motor_limits[DM_MOTOR_TYPE_NUM] = {
    {12.5f, 30.0f,  10.0f},   // DM4310
    {12.5f, 50.0f,  10.0f},   // DM4310_48V
    {12.5f, 10.0f,  28.0f},   // DM4340
    {12.5f, 10.0f,  28.0f},   // DM4340_48V
    {12.5f, 45.0f,  12.0f},   // DM6006
    {12.5f, 45.0f,  20.0f},   // DM8006
    {12.5f, 45.0f,  54.0f},   // DM8009
    {12.5f, 25.0f, 200.0f},   // DM10010L
    {12.5f, 20.0f, 200.0f},   // DM10010
    {12.5f, 280.0f,  1.0f},   // DMH3510
    {12.5f, 45.0f,  10.0f},   // DMH6215
    {12.5f, 45.0f,  10.0f}    // DMG6220
};

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

/* float → uint 线性映射 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* uint → float 线性映射 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DMMotorSetMode(DM_Motor_Cmd_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorSetDMType(DMMotorInstance *motor, DM_Motor_Type_e type)
{
    if (type >= DM_MOTOR_TYPE_NUM)
        type = DM4310;
    motor->dm_motor_type = type;
    motor->limits = dm_motor_limits[type];
}

/* 解析 MIT 模式反馈帧 (bytes 1-5: position/velocity/torque) */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);
    DM_Motor_Limit_s *lim = &(motor->limits);

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;

    /* 位置 16-bit (bytes 1-2) */
    uint16_t tmp16 = (uint16_t)(rxbuff[1] << 8) | rxbuff[2];
    measure->position = uint_to_float(tmp16, -lim->p_max, lim->p_max, 16);

    /* 速度 12-bit (byte 3 全8位 + byte 4 高4位) */
    tmp16 = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp16, -lim->v_max, lim->v_max, 12);

    /* 扭矩 12-bit (byte 4 低4位 + byte 5 全8位) */
    tmp16 = (uint16_t)((rxbuff[4] & 0x0f) << 8) | rxbuff[5];
    measure->torque = uint_to_float(tmp16, -lim->t_max, lim->t_max, 12);

    /* bytes 6-7: 温度, 部分固件版本可能为保留字节 */
    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    (void)motor_ptr;
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    /* 默认 DM4310, 用户可通过 DMMotorSetDMType() 更改 */
    DMMotorSetDMType(motor, DM4310);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/* 切换控制模式: 通过 RID=10 写入 mode 值 */
void DMMotorSetControlMode(DMMotorInstance *motor, DM_Control_Mode_e mode)
{
    uint8_t data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
    DMMotorWriteRegister(motor, 10, data);
}

/* 读电机寄存器 (CAN ID 0x7FF, 命令 0x33) */
float DMMotorReadRegister(DMMotorInstance *motor, uint8_t rid)
{
    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(can->tx_id & 0xff);
    tx[1] = (uint8_t)((can->tx_id >> 8) & 0xff);
    tx[2] = 0x33;
    tx[3] = rid;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;

    /* 通过 ID 0x7FF 发送寄存器读命令 */
    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
    CANTransmit(can, 1);
    can->tx_id = saved_id;
    return 0.0f;  // 异步操作, 结果在下一帧反馈中获取
}

/* 写电机寄存器 (CAN ID 0x7FF, 命令 0x55) */
void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t rid, const uint8_t data[4])
{
    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(can->tx_id & 0xff);
    tx[1] = (uint8_t)((can->tx_id >> 8) & 0xff);
    tx[2] = 0x55;
    tx[3] = rid;
    tx[4] = data[0];
    tx[5] = data[1];
    tx[6] = data[2];
    tx[7] = data[3];

    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
    CANTransmit(can, 1);
    can->tx_id = saved_id;
}

/* 保存所有参数到 Flash (CAN ID 0x7FF, 命令 0xAA, RID=0x01) */
void DMMotorSaveParams(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);  // 先停电机
    DWT_Delay(0.01);

    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(can->tx_id & 0xff);
    tx[1] = (uint8_t)((can->tx_id >> 8) & 0xff);
    tx[2] = 0xAA;
    tx[3] = 0x01;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;

    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
    CANTransmit(can, 1);
    can->tx_id = saved_id;

    osDelay(100);  // 等待写入完成
}

void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DM_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    float pid_ref_p, set;
    (void)pid_ref_p;

    while (1)
    {
        measure = &motor->measure;
        float pid_ref = motor->pid_ref;

        /* 角度环 */
        float pid_measure = measure->position;
        pid_ref_p = PIDCalculate_Ecd(&motor->angle_PID, pid_measure, pid_ref) * 0.5f;

        if (fabsf(pid_ref_p) < 0.001f)
            pid_ref_p = 0.0f;

        set = pid_ref_p;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1.0f;

        /* 限制扭矩在电机允许范围内 */
        if (set > motor->limits.t_max)
            set = motor->limits.t_max;
        else if (set < -motor->limits.t_max)
            set = -motor->limits.t_max;

        /* MIT 模式打包: pos=0, vel=0, Kp=0, Kd=0 (纯扭矩控制) */
        motor_send_mailbox.position_des = float_to_uint(0.0f, -motor->limits.p_max, motor->limits.p_max, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0.0f, -motor->limits.v_max, motor->limits.v_max, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;
        motor_send_mailbox.torque_des = float_to_uint(set, -motor->limits.t_max, motor->limits.t_max, 12);

        if (motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0.0f, -motor->limits.t_max, motor->limits.t_max, 12);

        /* 按 MIT 协议打包 8-byte CAN 帧 */
        uint8_t *tx = motor->motor_can_instace->tx_buff;
        tx[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        tx[1] = (uint8_t)(motor_send_mailbox.position_des);
        tx[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        tx[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | ((motor_send_mailbox.Kp >> 8) & 0xF));
        tx[4] = (uint8_t)(motor_send_mailbox.Kp);
        tx[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        tx[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | ((motor_send_mailbox.torque_des >> 8) & 0xF));
        tx[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}

void DMMotorControlInit(void)
{
    char dm_task_name[5] = "dm";
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 256);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}
