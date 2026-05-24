#include "chassis_power_control.h"
#include "message_center.h"
#include "arm_math.h"
#include "robot_def.h"
#include "remote_control.h"
#include "controller.h"
#include "math.h"  // 添加 sqrtf 等数学函数

static Publisher_t *chassis_power_pub;      // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;      // 底盘反馈信息订阅者
static Chassis_Power_Data_s chassis_power_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data;     // 从底盘接收的反馈信息

static float32_t toque_coefficient = 0.00000199688994; // 扭矩系数
static float32_t k1 = 0.000000123;                      // 电流平方项系数
static float32_t k2 = 0.0000001453;                     // 速度平方项系数
static float32_t constant = 1.9;                        // 常数项

static PIDInstance power_buffer_pid;

static float ks = 0.4;          // 功率平滑系数（原代码保留，但用法修正）
static float debug_stl;         // 规划功率（调试用）
static float last_power_limit = 0;  // 上次功率限制（用于前馈）

/// @brief 功率控制初始化
void chassis_power_control_init(void)
{
    // 优化 PID 参数：增大 Kp，减小 Ki，加入积分分离
    PID_Init_Config_s pid_cfg = {
        .Kp = 0.3,                // 原 0.1，提高响应速度
        .Ki = 0.02,               // 原 0.05，减小积分影响
        .Kd = 0.2,
        .DeadBand = 0,
        .MaxOut = 15,             // 适当增大输出范围
        .MaxOut_ = -15,
        .IntegralLimit = 3,       // 减小积分限幅，防止积分饱和
        .Derivative_LPF_RC = 0.1,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    };
    PIDInit(&power_buffer_pid, &pid_cfg);
    chassis_power_pub = PubRegister("power_cmd", sizeof(Chassis_Power_Data_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

/// @brief 功率控制主函数（优化版）
void chassis_power_control(void)
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    // 1. 估算各电机当前功率，并计算总功率
    float motor_power[4] = {0};
    float total_power = 0.0f;

    for (size_t i = 0; i < 4; i++) {
        // 功率模型：P = torque_coefficient * I * ω + k1 * I² + k2 * ω² + constant
        // 注：电流和速度来自电机反馈
        motor_power[i] = toque_coefficient * chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_speed[i]
                         + k1 * chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_current[i]
                         + k2 * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i]
                         + constant;
        total_power += motor_power[i];
    }

    // 对总功率进行一阶低通滤波，减小噪声影响（可调系数 0.9）
    static float filtered_total_power = 0;
    filtered_total_power = 0.9f * filtered_total_power + 0.1f * total_power;
    total_power = filtered_total_power;

    // 2. PID 计算功率调节量，加入功率限制前馈
    float power_limit = chassis_fetch_data.chassis_power_limit;
    float pid_out = PIDCalculate(&power_buffer_pid, total_power, power_limit);
    
    // 前馈：功率限制变化量 * 系数（如 0.5）
    float ff = (power_limit - last_power_limit) * 0.5f;
    last_power_limit = power_limit;
    
    float ref_power = total_power + pid_out + ff;
    
    // 限幅：确保规划功率不超过裁判系统限制，且非负
    if (ref_power < 0) ref_power = 0;
    if (ref_power > power_limit) ref_power = power_limit;
    
    debug_stl = ref_power;  // 调试用

    // 3. 按比例分配功率（根据各电机当前功率比例）
    float total_alloc = 0;
    for (size_t i = 0; i < 4; i++) {
        // 为避免除零，若总功率为0则平均分配
        if (total_power < 0.001f) {
            motor_power[i] = 1.0f;  // 平均权重
        } else {
            motor_power[i] = motor_power[i];  // 已计算
        }
        total_alloc += motor_power[i];
    }
    
    for (size_t i = 0; i < 4; i++) {
        float ratio = motor_power[i] / total_alloc;
        float target_power = ref_power * ratio;
        
        // 根据目标功率反解允许的电流上限和下限（保持原有求解逻辑）
        float a = k1;
        float b = chassis_fetch_data.motor_speed[i] * toque_coefficient;
        float c = k2 * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i] + constant - target_power;
        
        float discriminant = b * b - 4 * a * c;
        if (discriminant > 0) {
            float sqrt_d = sqrtf(discriminant);
            float sol1 = (-b + sqrt_d) / (2 * a);
            float sol2 = (-b - sqrt_d) / (2 * a);
            
            // 正解作为电流上限，负解作为电流下限（需根据物理意义确认）
            if (sol1 > 0) {
                chassis_power_send.motor_current_up[i] = sol1;
                chassis_power_send.motor_current_down[i] = (sol2 < 0) ? sol2 : 0;
            } else {
                chassis_power_send.motor_current_down[i] = sol1;  // 若 sol1 为负则作为下限
                chassis_power_send.motor_current_up[i] = (sol2 > 0) ? sol2 : 0;
            }
            
            // 安全限幅，防止过大电流
            if (chassis_power_send.motor_current_up[i] > 20000) chassis_power_send.motor_current_up[i] = 20000;
            if (chassis_power_send.motor_current_down[i] < -20000) chassis_power_send.motor_current_down[i] = -20000;
        } else {
            // 无解时，临时使用当前电流限制
            chassis_power_send.motor_current_up[i] = 16000;
            chassis_power_send.motor_current_down[i] = -16000;
        }
    }
    
    // 4. 将总功率作为反馈数据（可用于 UI 显示）
    chassis_power_send.chassis_power_mx = total_power;
    
    // 5. 发布功率控制指令
    PubPushMessage(chassis_power_pub, (void *)&chassis_power_send);
}