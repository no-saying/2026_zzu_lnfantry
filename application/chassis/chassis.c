#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "go_motor.h"
#include "mtf01.h"
#include "message_center.h"
#include "power_switch.h"
#include "referee_task.h"
#include "ins_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "buzzer.h"
#include "CAN_supercap_communication.h"
#include "chassis_power_control.h"
#define __CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/* 根据 robot_def.h 中的宏自动计算的参数 */
#define HALF_WHEEL_BASE     (WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH    (TRACK_WIDTH / 2.0f)
#define PERIMETER_WHEEL     (RADIUS_WHEEL * 2 * PI)

/* 麦克纳姆轮运动学系数（根据云台中心偏移计算） */
#define LF_CENTER  ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER  ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER  ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER  ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/* 底盘全局数据结构（单例，仅在文件内可见） */
typedef struct {
    /* 电机实例 */
    DJIMotorInstance *wheel_motors[4];      // 索引：0-LF, 1-RF, 2-LB, 3-RB
    GOMotorInstance *leg_motors[4];          // 索引：0-RF, 1-LF, 2-RB, 3-LB（与原代码一致）
    
    
    /* 控制命令与反馈 */
    Chassis_Ctrl_Cmd_s cmd;
    chassis_mode_leg leg_mode;                // 腿模式（通过CAN独立接收）
    Chassis_Power_Data_s power_cmd;
    Chassis_Upload_Data_s feedback;
    Gimbal_Upload_Data_s gimbal_feedback;
    chassis_peek_mode peek_mode;
    /* 状态标志与计时（仅用于腿模式） */
    uint8_t supercap_enable;         //超电启动标志位
    uint8_t switch_enable;           //悬挂启动标志位
    uint8_t enable_chassis_bo;       // 悬挂使能标志
    uint8_t flag_1, flag_2;           // 回正力矩状态标志
    uint16_t time_delay;               // 未使用（可移除，保留兼容）
    uint16_t time_jump;                 // 回正力矩延时计数器
    uint8_t flag_flop;                 // 翻车标志
    uint8_t flag_behind;               // 后腿悬挂标志
    uint8_t v_flag;                     // 速度方向标志（-1,0,1）
    uint8_t v_cnt;                       // 速度方向计数器
    uint8_t feedback_time;               // 加速度反馈渐变计数器
    uint8_t it;                           // 循环计数器（用于roll控制分频）

    /* 运动学中间变量 */
    float chassis_vx, chassis_vy;        // 底盘系速度
    float vt[4];                          // 四个轮子的速度参考值（顺序同wheel_motors）
    float real_wz;                         // 实际旋转速度（用于前馈）

    /* 姿态相关 */
    attitude_t *imu_data;                  // 指向IMU数据（底盘板）
    float pitch_ref, roll_ref;               // 俯仰/横滚控制量
    float y_accel;                            // 水平加速度（用于悬挂策略）
    float hang_height;                      //悬挂高度

    /* PID控制器 */
    PIDInstance follow_pid;      // 跟随云台PID
    PIDInstance pitch_pid;       // 俯仰控制PID
    PIDInstance roll_pid;        // 横滚控制PID

    /* 外部模块实例 */
    referee_info_t *referee;
    Referee_Interactive_info_t ui_data;
    SuperCap_s *supercap;
    BuzzzerInstance *buzzer;

#ifdef CHASSIS_BOARD
    /* 双板通信相关 */
    Subscriber_t *sub_cmd;
    Subscriber_t *sub_power;
    Publisher_t *pub;
    MTF01_Recv_s *mtf01;
    CANCommInstance *can_comm[4];          // 分别接收 vx/vy, wz/offset, mode, leg_mode
    Chassis_Ctrl_Cmd_1s cmd_1;
    Chassis_Ctrl_Cmd_2s cmd_2;
    Chassis_Ctrl_Cmd_3s cmd_3;
    Chassis_Ctrl_Cmd_4s cmd_4;
#endif

#ifdef ONE_BOARD
    /* 单板发布订阅 */
    Subscriber_t *sub_cmd;
    Subscriber_t *sub_power;
    Publisher_t *pub;
    Subscriber_t *sub_gimbal;
#endif
} Chassis_t;

static Chassis_t chassis;   // 单例底盘对象

/* 私有函数声明 */
static void ChassisWheelMotorsInit(void);
static void ChassisLegMotorsInit(void);
static void ChassisYawMotorInit(void);
static void ChassisSuperCapInit(void);
static void ChassisCommInit(void);
static void MergeChassisCommands(void);
static void ChassisModeHandler(void);
static void HandleModeNoFollow(void);
static void HandleModeFollow(void);
static void HandleModeRotate(void);
static void HandleLegMode(void);
static void enable_accel(uint8_t flag);
static void inverse_transform(void);
static void HangControl(void);
static void MecanumCalculate(void);
static void LimitOutput(void);
static void EstimateSpeed(void);
static void UpdateUIData(void);

/* 初始化入口 */
void ChassisInit(void)
{
#ifdef CHASSIS_BOARD
    chassis.imu_data = INS_Init();
    init_accel_ptr(chassis.imu_data, &chassis.v_flag);
    chassis.mtf01 = MTF01Init(&huart10, NULL);
    power_switch_init(&huart3,NULL);
#endif

    /* 初始化各子模块 */
    ChassisWheelMotorsInit();
    ChassisLegMotorsInit();
    ChassisSuperCapInit();
    ChassisCommInit();

    /* PID 初始化 */
    PID_Init_Config_s follow_config = {
        .Kp = 68, .Ki = 0, .Kd = 5,
        .Derivative_LPF_RC = 0,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = 15000, .MaxOut_ = -15000,
        .DeadBand = 3
    };
    PIDInit(&chassis.follow_pid, &follow_config);

    // 老代码的慢速 PID 参数
    PID_Init_Config_s pitch_config = {
        .Kp = 0.2, .Ki = 0, .Kd = 0.2,
        .Derivative_LPF_RC = 0.01,
        .IntegralLimit = 0.02,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = 0.25, .MaxOut_ = -0.25,
        .DeadBand = 0.01
    };
    PIDInit(&chassis.pitch_pid, &pitch_config);

    PID_Init_Config_s roll_config = {
        .Kp = 0.4, .Ki = 0, .Kd = 0.1,
        .Derivative_LPF_RC = 0,
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = 0.04, .MaxOut_ = -0.04,
        .DeadBand = 0.045
    };
    PIDInit(&chassis.roll_pid, &roll_config);

    /* 裁判系统及 UI */
    #ifdef CHASSIS_BOARD
        USART_H7_SetBaudRateOnly(&huart1,115200);
        chassis.referee = UITaskInit(&huart1, &chassis.ui_data);
    #endif // DEBUG
    /* 蜂鸣器 */
    Buzzer_config_s buzzer_cfg = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0,
        .octave = OCTAVE_2,
    };
    chassis.buzzer = BuzzerRegister(&buzzer_cfg);

}

/* 轮毂电机初始化 */
static void ChassisWheelMotorsInit(void)
{
    Motor_Init_Config_s cfg = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = { .Kp = 6, .Ki = 0, .Kd = 0, .IntegralLimit = 3000, .DeadBand = 10,
                           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                           .MaxOut = 32000, .MaxOut_ = -32000 },
            .current_PID = { .Kp = 0.5, .Ki = 0, .Kd = 0, .IntegralLimit = 5000,
                             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                             .MaxOut = 16000, .MaxOut_ = -16000 }
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE   // 默认反转
        },
        .motor_type = M3508,
    };

    /* 设置各电机 tx_id：LF(2), RF(4), LB(3), RB(1) */
    uint32_t tx_ids[4] = {2, 4, 3, 1};
    for (int i = 0; i < 4; i++) {
        cfg.can_init_config.tx_id = tx_ids[i];
        chassis.wheel_motors[i] = DJIMotorInit(&cfg);
    }

    /* 调整后两个电机（LB, RB）的速度环参数 */
    for (int i = 2; i < 4; i++) {
        DJIMotorInstance *motor = chassis.wheel_motors[i];
        motor->motor_controller.speed_PID.Kp = 25;
        motor->motor_controller.speed_PID.MaxOut = 40000;
        motor->motor_controller.speed_PID.MaxOut_ = -40000;
        motor->motor_controller.current_PID.Kp = 0.2;
        motor->motor_controller.current_PID.DeadBand = 2500;
    }
}

/* 腿关节电机初始化（修复野指针浮点数BUG版） */
static void ChassisLegMotorsInit(void)
{
    Other_Motor_Init_Config_s cfg = {
        .conf = { .ID = 0, .Kp = 0, .Kd = 0, .Mode = lock },
        .usart_init_config = {
            .enable_485 = 1,
            .module_callback = NULL,
            .recv_buff_size = 16,
            .usart_handle = &huart3
        },
        .c_ontroller_param_init_config = {
            .angle_PID = { .Kp = 35, .Ki = 0, .Kd = 0, .DeadBand = 0.10, .Derivative_LPF_RC = 0.003,
                           .Output_LPF_RC = 0.02, .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                           .IntegralLimit = 100, .MaxOut = 800, .MaxOut_ = -800 },
            .speed_PID = { .Kp = 0.015, .Ki = 0, .Kd = 0, .Output_LPF_RC = 0.005, .DeadBand = 10,
                           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                           .IntegralLimit = 3000, .MaxOut = 80.0, .MaxOut_ = -80.0 }
        },
        .c_ontroller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
        .motor_type = GOM8010
    };

    // RF (索引0) pos_comm = 4.888
    cfg.conf.ID = 0;
    cfg.usart_init_config.id = 0;
    cfg.controller_setting_init_config.k_accel_y = 0.0005;
    cfg.controller_setting_init_config.motor_reverse_flag = 1;
    cfg.controller_setting_init_config.motor_position_flag = 1;
    cfg.controller_setting_init_config.Kpos = 0.2;
    cfg.controller_setting_init_config.pos_comm = 4.888;
    cfg.c_ontroller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis.leg_motors[0] = GOmotorInit(cfg);
    GoMotorSetRef(chassis.leg_motors[0], 4.888f); // 直接传常量！

    // LF (索引1) pos_comm = 0.406
    cfg.conf.ID = 2;
    cfg.usart_init_config.id = 2;
    cfg.controller_setting_init_config.motor_reverse_flag = -1;
    cfg.controller_setting_init_config.motor_position_flag = 1;
    cfg.controller_setting_init_config.Kpos = 0.2;
    cfg.controller_setting_init_config.pos_comm = 0.159;
    cfg.c_ontroller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis.leg_motors[1] = GOmotorInit(cfg);
    GoMotorSetRef(chassis.leg_motors[1], 0.159f); // 直接传常量！

    // RB (索引2) pos_comm = 0.392
    cfg.controller_setting_init_config.k_accel_y = 0.0007;
    cfg.c_ontroller_param_init_config.speed_PID.Kp = 0.017;
    cfg.conf.ID = 3;
    cfg.usart_init_config.id = 3;
    cfg.controller_setting_init_config.motor_reverse_flag = 1;
    cfg.controller_setting_init_config.motor_position_flag = -1;
    cfg.controller_setting_init_config.Kpos = 0.15;
    cfg.controller_setting_init_config.pos_comm = 0.360;
    cfg.c_ontroller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis.leg_motors[2] = GOmotorInit(cfg);
    GoMotorSetRef(chassis.leg_motors[2], 0.360f); // 直接传常量！

    // LB (索引3) pos_comm = 2.812
    cfg.conf.ID = 1;
    cfg.usart_init_config.id = 1;
    cfg.controller_setting_init_config.motor_reverse_flag = -1;
    cfg.controller_setting_init_config.motor_position_flag = -1;
    cfg.controller_setting_init_config.Kpos = 0.15;
    cfg.controller_setting_init_config.pos_comm = 3.463;
    cfg.c_ontroller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis.leg_motors[3] = GOmotorInit(cfg);
    GoMotorSetRef(chassis.leg_motors[3], 3.463f); // ✅ 修复在这里！
}
/* 超级电容初始化 */
static void ChassisSuperCapInit(void)
{
    chassis.supercap = SuperCapInit(&hcan1);
    chassis.supercap->TX_Temp.Enable = ENABLE;
    chassis.supercap->TX_Temp.Powerlimit = 45;
    chassis.supercap->TX_Temp.ChargePower = 45;
    // 重置功率动态监测系统
}

/* 通信模块初始化（双板或单板） */
static void ChassisCommInit(void)
{
#ifdef CHASSIS_BOARD
    // 第一个CAN通信，接收 vx, vy
    CANComm_Init_Config_s comm_conf1 = {
        .can_config = { .can_handle = &hcan2, .tx_id = 0x311, .rx_id = 0x312 },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_1s)
    };
    chassis.can_comm[0] = CANCommInit(&comm_conf1);

    // 第二个CAN通信，接收 wz, offset_angle
    CANComm_Init_Config_s comm_conf2 = {
        .can_config = { .can_handle = &hcan2, .tx_id = 0x315, .rx_id = 0x313 },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_2s)
    };
    chassis.can_comm[1] = CANCommInit(&comm_conf2);

    // 第三个CAN通信，接收 chassis_mode 传递爆发使能
    CANComm_Init_Config_s comm_conf3 = {
        .can_config = { .can_handle = &hcan2, .tx_id = 0x316, .rx_id = 0x314 },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_3s)
    };
    chassis.can_comm[2] = CANCommInit(&comm_conf3);
    // 第四个CAN通信，接收 chassis_leg_mode
    CANComm_Init_Config_s comm_conf4 = {
        .can_config = { .can_handle = &hcan2, .tx_id = 0x319, .rx_id = 0x317 },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_4s)
    };
    chassis.can_comm[3] = CANCommInit(&comm_conf4);
    chassis.sub_cmd   = SubRegister("chassis_cmd",   sizeof(Chassis_Ctrl_Cmd_s));
    chassis.sub_power = SubRegister("power_cmd",     sizeof(Chassis_Power_Data_s));
    chassis.pub       = PubRegister("chassis_feed",  sizeof(Chassis_Upload_Data_s));
#endif

#ifdef ONE_BOARD
    chassis.sub_cmd   = SubRegister("chassis_cmd",   sizeof(Chassis_Ctrl_Cmd_s));
    chassis.sub_power = SubRegister("power_cmd",     sizeof(Chassis_Power_Data_s));
    chassis.pub       = PubRegister("chassis_feed",  sizeof(Chassis_Upload_Data_s));
    chassis.sub_gimbal= SubRegister("gimbal_feed",   sizeof(Gimbal_Upload_Data_s));
#endif
}

/* 合并 CAN 命令：正确接收腿模式 */
static void MergeChassisCommands(void)
{
#ifdef CHASSIS_BOARD
    if (CANCommGet(chassis.can_comm[0], CAN_DATA_MIXED, &chassis.cmd_1)) {
        chassis.cmd.vx = -chassis.cmd_1.vx;
        chassis.cmd.vy = -chassis.cmd_1.vy;
    }
    if (CANCommGet(chassis.can_comm[1], CAN_DATA_MIXED, &chassis.cmd_2)) {
        chassis.cmd.wz = chassis.cmd_2.wz;
        chassis.cmd.offset_angle = chassis.cmd_2.offset_angle;
    }
    if (CANCommGet(chassis.can_comm[2], CAN_DATA_MIXED, &chassis.cmd_3)) {
        chassis.cmd.chassis_mode = chassis.cmd_3.chassis_mode;
        chassis.supercap_enable = chassis.cmd_3.supercap_enable;
        chassis.peek_mode = chassis.cmd_3.peek_mode;
        //set_supercap_enable(chassis.supercap_enable);  // 传递爆发使能
    }
    if (CANCommGet(chassis.can_comm[3], CAN_DATA_MIXED, &chassis.cmd_4)) {
        chassis.switch_enable = chassis.cmd_4.leg_switch;
        chassis_mode_leg old_mode = chassis.leg_mode;
        chassis.leg_mode = chassis.cmd_4.chassis_mode_leg;
        chassis.hang_height = chassis.cmd_4.hang_height;
        if (chassis.leg_mode == CHASSIS_HANGOFF && old_mode != CHASSIS_HANGOFF) {
            chassis.feedback_time = 0;
        }
    }
#endif
}

/* 使能腿电机的加速度反馈 */
static void enable_accel(uint8_t flag)
{
    for (int i = 0; i < 4; i++) {
        if (chassis.leg_motors[i]) {
            chassis.leg_motors[i]->enable_accel_feedback = flag;
        }
    }
}

/* 将加速度转换到水平方向（原 inverse_transform 函数） */
static void inverse_transform(void)
{
    float dev_y = chassis.imu_data->Accel[2];
    float dev_z = chassis.imu_data->Accel[1];
    float pitch_deg = chassis.imu_data->Pitch;
    float compensated_pitch_deg = pitch_deg + 88.0f;
    float pitch_rad = compensated_pitch_deg * DEGREE_2_RAD;
    pitch_rad = -pitch_rad;
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    chassis.y_accel = - (cp * dev_y - sp * dev_z);
}

/* 底盘模式处理主函数（仅控制轮子，不再干涉腿模式） */
static void ChassisModeHandler(void)
{
    static chassis_mode_leg last_leg_mode = CHASSIS_HANGOFF;

    last_leg_mode = chassis.leg_mode;

    switch (chassis.cmd.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            HandleModeNoFollow();
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            HandleModeFollow();
            break;
        case CHASSIS_ROTATE:
            HandleModeRotate();
            break;
        default:
            break;
    }
}

/* 无跟随模式 */
static void HandleModeNoFollow(void)
{
    chassis.cmd.wz = 0;
}

static float angle_ref=0.0;
/* 跟随云台模式 */
static void HandleModeFollow(void)
{

    chassis.cmd.wz = PIDCalculate(&chassis.follow_pid, -chassis.cmd.offset_angle, 0)
                     - 0.5f * REAL_WZ_RAT * chassis.gimbal_feedback.gimbal_imu_data.Gyro[2];
}

/* 自旋模式 */
static void HandleModeRotate(void)
{
    chassis.cmd.wz = 6000;
}
static float32_t taget=0.0;

static void HandleLegMode(void)
{
    // 静态变量：保存上一状态，用于检测是否需要重新开始平滑运动
    static chassis_mode_leg last_leg_mode = CHASSIS_HANGOFF;
    static float last_hang_height = 0.0f;
    static uint8_t last_switch_enable = 0;

    // 翻车保护（不变）
    if (chassis.imu_data->Roll < -30 || chassis.imu_data->Roll > 30 ||
        chassis.imu_data->Pitch < -150 || chassis.imu_data->Pitch > -10 ||
        chassis.flag_flop == 1) {
        chassis.flag_flop = 1;
        return;
    }

    // ===================== 【核心优化1】计数器正确重置逻辑 =====================
    // 只要 模式 / 高度 / 总开关 任何一个变了，立刻重新开始平滑运动
    if (chassis.leg_mode != last_leg_mode ||
        chassis.hang_height != last_hang_height ||
        chassis.switch_enable != last_switch_enable)
    {
        last_leg_mode = chassis.leg_mode;       // 记录新模式
        last_hang_height = chassis.hang_height; // 记录新高度
        last_switch_enable = chassis.switch_enable;
        chassis.feedback_time = 0; // 计数器归零 → 重新平滑渐变
    }

    // ===================== 【核心优化2】计数规则统一：0~50，满了保持50 =====================
    if (chassis.feedback_time < 50) {
        chassis.feedback_time++;
    }

    // ===================== 模式执行 =====================
    switch (chassis.leg_mode) {
        case CHASSIS_JUMP:
            enable_accel(0);
            // 跳跃使用平滑目标，不突变
            GoMotorSetRef(chassis.leg_motors[2], chassis.leg_motors[2]->controller.pos_comm);
            GoMotorSetRef(chassis.leg_motors[3], chassis.leg_motors[3]->controller.pos_comm);
            chassis.enable_chassis_bo = 1;
            break;

        case CHASSIS_HANGOFF: 
        {
            chassis.enable_chassis_bo = 0;
            switch(chassis.peek_mode)
            {
                case peek_none:
                {
                    float target0 = chassis.leg_motors[0]->mes.pos +
                                    (chassis.leg_motors[0]->controller.pos_comm - chassis.leg_motors[0]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target1 = chassis.leg_motors[1]->mes.pos +
                                    (chassis.leg_motors[1]->controller.pos_comm - chassis.leg_motors[1]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[0], target0);
                    GoMotorSetRef(chassis.leg_motors[1], target1);

                    float target2 = chassis.leg_motors[2]->mes.pos +
                                    (chassis.leg_motors[2]->controller.pos_comm - chassis.leg_motors[2]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target3 = chassis.leg_motors[3]->mes.pos +
                                    (chassis.leg_motors[3]->controller.pos_comm - chassis.leg_motors[3]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[2], target2);
                    GoMotorSetRef(chassis.leg_motors[3], target3);
                    break;
                }
                case  peek_left:
                {
                    float target0 = chassis.leg_motors[0]->mes.pos +
                                    (chassis.leg_motors[0]->controller.pos_comm-5.0 - chassis.leg_motors[0]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target1 = chassis.leg_motors[1]->mes.pos +
                                    (chassis.leg_motors[1]->controller.pos_comm - chassis.leg_motors[1]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[0], target0);
                    GoMotorSetRef(chassis.leg_motors[1], target1);

                    float target2 = chassis.leg_motors[2]->mes.pos +
                                    (chassis.leg_motors[2]->controller.pos_comm - chassis.leg_motors[2]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target3 = chassis.leg_motors[3]->mes.pos +
                                    (chassis.leg_motors[3]->controller.pos_comm-5.0 - chassis.leg_motors[3]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[2], target2);
                    GoMotorSetRef(chassis.leg_motors[3], target3);
                    break;
                }
                case peek_right:
                {
                    float target0 = chassis.leg_motors[0]->mes.pos +
                                    (chassis.leg_motors[0]->controller.pos_comm - chassis.leg_motors[0]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target1 = chassis.leg_motors[1]->mes.pos +
                                    (chassis.leg_motors[1]->controller.pos_comm+5.0 - chassis.leg_motors[1]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[0], target0);
                    GoMotorSetRef(chassis.leg_motors[1], target1);

                    float target2 = chassis.leg_motors[2]->mes.pos +
                                    (chassis.leg_motors[2]->controller.pos_comm+5.0 - chassis.leg_motors[2]->mes.pos) / 50.0f * chassis.feedback_time;
                    float target3 = chassis.leg_motors[3]->mes.pos +
                                    (chassis.leg_motors[3]->controller.pos_comm- chassis.leg_motors[3]->mes.pos) / 50.0f * chassis.feedback_time;
                    GoMotorSetRef(chassis.leg_motors[2], target2);
                    GoMotorSetRef(chassis.leg_motors[3], target3);
                    break;
                }
            }
            break;
            // 【优化3】软开启加速度反馈，不抖动
        }

        case CHASSIS_HANG:
            chassis.enable_chassis_bo = 1;

            // 【优化3】软开启加速度反馈
            enable_accel(chassis.feedback_time >= 45 ? 1 : 0);

            if (!chassis.flag_behind) {
                #ifdef CHASSIS_BOARD
                // 【优化4】统一50步平滑，轨迹丝滑
                float target_rb = chassis.leg_motors[2]->mes.pos +
                    (chassis.leg_motors[2]->controller.pos_comm + chassis.cmd_4.hang_height - chassis.leg_motors[2]->mes.pos) / 50.0f * chassis.feedback_time;
                float target_lb = chassis.leg_motors[3]->mes.pos +
                    (chassis.leg_motors[3]->controller.pos_comm - chassis.cmd_4.hang_height - chassis.leg_motors[3]->mes.pos) / 50.0f * chassis.feedback_time;

                GoMotorSetRef(chassis.leg_motors[2], target_rb);
                GoMotorSetRef(chassis.leg_motors[3], target_lb);
                #endif
            }
            break;

        default:
            break;
    }
}

/* 悬挂姿态控制（优化版） */
static void HangControl(void)
{
#ifdef CHASSIS_BOARD
    if (!chassis.enable_chassis_bo) return;

    // 计算俯仰误差修正量（仍基于当前姿态）
    chassis.pitch_ref = PIDCalculate(&chassis.pitch_pid,
                                      chassis.imu_data->Pitch * DEGREE_2_RAD, -1.5312f);
    inverse_transform();

    // 渐变系数：进入悬挂模式后50个周期内逐渐增强修正
    float factor = (chassis.feedback_time < 50) ? (chassis.feedback_time / 50.0f) : 1.0f;
    float smoothed_pitch_ref = chassis.pitch_ref * factor;

    // 根据加速度情况选择控制策略
    if (chassis.y_accel > 3.0f && chassis.v_flag == -1) {
        // 后腿参与修正（系数与原代码一致）
        /*目的是减缓加减速带来的抬头点头问题，不过目前效果不好，且悬挂越高效果越差*/
        GoMotorSetRef(chassis.leg_motors[0], 
                      chassis.leg_motors[0]->motor_controller.pid_ref + smoothed_pitch_ref * 0.8f);
        GoMotorSetRef(chassis.leg_motors[1], 
                      chassis.leg_motors[1]->motor_controller.pid_ref - smoothed_pitch_ref * 0.8f);
        GoMotorSetRef(chassis.leg_motors[2], 
                      chassis.leg_motors[2]->motor_controller.pid_ref - smoothed_pitch_ref * 1.2f);
        GoMotorSetRef(chassis.leg_motors[3], 
                      chassis.leg_motors[3]->motor_controller.pid_ref + smoothed_pitch_ref * 1.2f);
        chassis.flag_behind = 1;
    } else {
        // 仅前腿修正（后腿维持原平滑目标）
        chassis.flag_behind = 0;
        GoMotorSetRef(chassis.leg_motors[0], 
                      chassis.leg_motors[0]->motor_controller.pid_ref - smoothed_pitch_ref);
        GoMotorSetRef(chassis.leg_motors[1], 
                      chassis.leg_motors[1]->motor_controller.pid_ref + smoothed_pitch_ref);
        // 后腿不叠加修正，保持 HandleLegMode 设置的平滑目标
    }
#endif
}

/* 麦克纳姆轮正运动学解算 */
static void MecanumCalculate(void)
{
    if (chassis.chassis_vy > 0.1f) {
        chassis.v_flag = 1;
        chassis.v_cnt = 0;
    } else if (chassis.chassis_vy < -0.1f) {
        chassis.v_flag = -1;
        chassis.v_cnt = 0;
    } else {
        if (++chassis.v_cnt > 8) {
            chassis.v_flag = 0;
            chassis.v_cnt = 0;
        }
    }
    if(fabsf(chassis.chassis_vy)>10&&fabsf(chassis.chassis_vx)>10)
    {
        chassis.cmd.wz /= 3.0;
    }
    chassis.vt[0] = (-chassis.chassis_vx - chassis.chassis_vy - chassis.cmd.wz * LF_CENTER);  // LF
    chassis.vt[1] = (-chassis.chassis_vx + chassis.chassis_vy - chassis.cmd.wz * RF_CENTER);  // RF
    chassis.vt[2] = ( chassis.chassis_vx - chassis.chassis_vy - chassis.cmd.wz * LB_CENTER);  // LB
    chassis.vt[3] = ( chassis.chassis_vx + chassis.chassis_vy - chassis.cmd.wz * RB_CENTER);  // RB
}
/* 速度估计（待实现） */
static void EstimateSpeed(void)
{
}

/* 更新 UI 数据 */
static void UpdateUIData(void)
{
#ifdef CHASSIS_BOARD
    chassis.ui_data.friction_mode = chassis.cmd_4.chassis_mode_leg;
    chassis.ui_data.height = chassis.cmd_4.hang_height;
    chassis.ui_data.switch_flag = chassis.cmd_4.leg_switch;
    chassis.ui_data.enable_super_cap = chassis.cmd_3.supercap_enable;
    chassis.ui_data.peek_mode = chassis.cmd_3.peek_mode;
    chassis.ui_data.super_cap.buffer_energy = (float)chassis.supercap->CAN_SuperCapRXData.SuperCapEnergy;
    chassis.ui_data.super_cap.SuperCapReady = (chassis.supercap->CAN_SuperCapRXData.SuperCapReady == READY) ? 1 : 0;
    chassis.ui_data.super_cap.chassis_power_mx = chassis.power_cmd.chassis_power_mx;
    chassis.ui_data.Chassis_Power_Data.chassis_power_mx = chassis.power_cmd.chassis_power_mx;
    chassis.ui_data.reverse_flag =chassis.cmd_4.reverse_flag;
    for(uint8_t i=0;i<4;i++)
        chassis.ui_data.leg_motor.motor[i] = chassis.leg_motors[i]->mes.tem;
#endif // CHASSIS_BOARD
}
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;()
    for(uint8_t i=0;i<4;i++)
    {
        chassis.wheel_motors[i]->motor_controller.current_PID.MaxOut = chassis.power_cmd.motor_current_up[i];
        chassis.wheel_motors[i]->motor_controller.current_PID.MaxOut_ = chassis.power_cmd.motor_current_down[i];
    }

}
/* 机器人底盘控制核心任务 */
void ChassisTask(void)
{
#ifdef ONE_BOARD
    SubGetMessage(chassis.sub_cmd,   &chassis.cmd);
    SubGetMessage(chassis.sub_power, &chassis.power_cmd);
    SubGetMessage(chassis.sub_gimbal,&chassis.gimbal_feedback);
#endif
#ifdef CHASSIS_BOARD
    SubGetMessage(chassis.sub_cmd,   &chassis.cmd);
    SubGetMessage(chassis.sub_power, &chassis.power_cmd);
    MergeChassisCommands();
    if(chassis.cmd_3.ui_init_enable==1)
    {
        MyUIInit();
    }
#endif
    //MyUIInit();
    // 急停或翻车处理
    if (chassis.cmd.chassis_mode == CHASSIS_ZERO_FORCE) {
        for (int i = 0; i < 4; i++) {
            DJIMotorStop(chassis.wheel_motors[i]);
            GOMotorStop(chassis.leg_motors[i]);
        }
        chassis.enable_chassis_bo = 0;
        chassis.buzzer->alarm_state = ALARM_ON;
    } 
    else {
        for (int i = 0; i < 4; i++) {
            DJIMotorEnable(chassis.wheel_motors[i]);
            GOMotorEnable(chassis.leg_motors[i]);
        }
        chassis.buzzer->alarm_state = ALARM_OFF;
        HangControl();   // 悬挂控制（仅当使能时内部判断）
    }

    // 检测轮毂电机是否至少有一个在线（利用守护线程）
    uint8_t any_wheel_online = 0;
    for (int i = 0; i < 4; i++) {
        DJIMotorInstance *motor = chassis.wheel_motors[i];
        if (motor && motor->daemon && DaemonIsOnline(motor->daemon)) {
            any_wheel_online = 1;
            break;
        }
    }

    if (!any_wheel_online) {
        // 轮毂电机全部离线 → 强制关断关节电机
        switch_off();
    } else {
        // 有电机在线 → 按原 switch_enable 状态控制
        if (chassis.switch_enable == 1)
            switch_on();
        else
            switch_off();
    }

    // 模式处理（轮子控制）
    ChassisModeHandler();

    // 腿模式处理（独立控制）
    HandleLegMode();

    // 将云台系速度转换到底盘系
    float cos_theta = arm_cos_f32(chassis.cmd.offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis.cmd.offset_angle * DEGREE_2_RAD);
    chassis.chassis_vy = -chassis.cmd.vx * cos_theta + chassis.cmd.vy * sin_theta;
    chassis.chassis_vx = -chassis.cmd.vx * sin_theta - chassis.cmd.vy * cos_theta;
    //超电使能
#ifdef CHASSIS_BOARD

    chassis.supercap_enable = chassis.cmd_3.supercap_enable;
#endif
    chassis.supercap->TX_Temp.ChargePower = chassis.referee->GameRobotState.chassis_power_limit;
    chassis.supercap->TX_Temp.Charge = 1;
    // 超级电容能量百分比（0~100）
    if(chassis.supercap->CAN_SuperCapRXData.SuperCapReady == READY)
    {
        if(((float)chassis.supercap->CAN_SuperCapRXData.SuperCapEnergy)>10.0&&chassis.supercap_enable==1)
        {
            chassis.supercap->TX_Temp.Powerlimit = 150;
            chassis.chassis_vx = chassis.chassis_vx*2;
            chassis.chassis_vy = chassis.chassis_vy*2;
            chassis.cmd.wz = chassis.cmd.wz*2;
        }
        else
        {
            chassis.supercap->TX_Temp.Powerlimit = chassis.referee->GameRobotState.chassis_power_limit;
        }    
    }
    else
    {
        chassis.supercap->TX_Temp.Powerlimit = chassis.referee->GameRobotState.chassis_power_limit;
    }

    // 运动学解算
    MecanumCalculate();
    LimitChassisOutput();
    // 4. 设置电机速度参考值（电机内部速度环将自动计算电流）
    for (int i = 0; i < 4; i++) {
        DJIMotorSetRef(chassis.wheel_motors[i], chassis.vt[i]);
    }
    // 5. 发送超级电容命令
    SuperCapSend(chassis.supercap);
    // ========== 功率控制部分结束 ==========    
    // 速度估计（预留）
    EstimateSpeed();

    // 更新UI及反馈数据

    // 发布反馈消息
#ifdef ONE_BOARD
    chassis.feedback.rest_heat = (chassis.referee->GameRobotState.shooter_barrel_heat_limit - chassis.referee->PowerHeatData.shooter_17mm_1_barrel_heat) / referee_data->GameRobotState.shooter_barrel_heat_limit;

    chassis.feedback.chassis_level = chassis.referee->GameRobotState.robot_level;
    chassis.feedback.chassis_power_limit = chassis.referee->GameRobotState.chassis_power_limit;

    for(uint8_t i=0;i<4;i++)
    {
        chassis.feedback.motor_speed[i] = chassis.wheel_motors[i]->measure.speed_aps / 6;
        chassis.feedback.motor_current[i] = chassis.wheel_motors[i]->measure.real_current;
    }

    chassis.feedback.real_wz = chassis.cmd.wz / REAL_WZ_RAT;
    PubPushMessage(chassis.pub, (void *)&chassis.feedback);
#endif
#ifdef CHASSIS_BOARD
    chassis.feedback.rest_heat = (chassis.referee->GameRobotState.shooter_barrel_heat_limit - chassis.referee->PowerHeatData.shooter_17mm_1_barrel_heat) / chassis.referee->GameRobotState.shooter_barrel_heat_limit;

    chassis.feedback.chassis_level = chassis.referee->GameRobotState.robot_level;
    chassis.feedback.chassis_power_limit = 90;//chassis.referee->GameRobotState.chassis_power_limit;

    for(uint8_t i=0;i<4;i++)
    {
        chassis.feedback.motor_speed[i] = chassis.wheel_motors[i]->measure.speed_aps / 6;
        chassis.feedback.motor_current[i] = chassis.wheel_motors[i]->measure.real_current;
    }

    chassis.feedback.real_wz = chassis.cmd.wz / REAL_WZ_RAT;
    PubPushMessage(chassis.pub, (void *)&chassis.feedback);
    UpdateUIData();
    // 双板模式下可添加CAN发送反馈的代码
#endif
}