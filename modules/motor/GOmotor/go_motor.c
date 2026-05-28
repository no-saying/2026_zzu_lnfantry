#include "go_motor.h"
#include "memory.h"
#include "controller.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "crc16.h"
#include "crc_ccitt.h"
#include "power_switch.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* GO电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
//用加速度计进行前馈【2】为前后前进为正，【0】为左右
//死区为0.7，会有飘
//备忘录：
//加速度负时，车体向前伸，后轮抬腿，前轮收腿
//加速度正时，车体向后伸，后轮收腿，前轮抬腿
static Chassis_accel_ptr accel_ptr;
static GOMotorUSARTInstance go_motor_usart;
static uint8_t go_motor_usart_enable=0;
static GOMotorInstance *go_motor_instance[GO_MOTOR_CNT] = {NULL};// 会在control任务中遍历该指针数组进行pid计算
static osThreadId go_task_handle[GO_MOTOR_CNT];
static uint8_t go_motor_id[GO_MOTOR_CNT];

// 加速度前馈初始化
void init_accel_ptr(attitude_t *imu_data,int8_t *v_flag)
{
    accel_ptr.x_accel_feedback_ptr = imu_data->Accel+0;
    accel_ptr.z_accel_feedback_ptr = imu_data->Accel+1;
    accel_ptr.y_accel_feedback_ptr = imu_data->Accel+2;
    accel_ptr.pitch = &imu_data->Pitch;
    accel_ptr.roll  = &imu_data->Roll;
    accel_ptr.v_flag = v_flag;
    return;
}

// 正变换：从设备坐标系转换到地面坐标系
// 坐标系定义：
// 设备坐标系：
//   x_accel_feedback_ptr: 左右方向加速度，向右为正
//   y_accel_feedback_ptr: 前后方向加速度，向前为正
//   z_accel_feedback_ptr: 竖直方向加速度，向上为正
// 旋转定义：
//   roll轴朝后，从o点向roll轴无穷远处看，roll角度变大的话，roll轴顺时针转
//   pitch轴朝左，从o点向pitch轴无穷远处看，pitch角度变大pitch顺时针转
static void inverse_transform() {
    // 获取设备原始加速度
    float dev_x = *accel_ptr.x_accel_feedback_ptr;  // 左右，右为正
    float dev_y = *accel_ptr.y_accel_feedback_ptr;  // 前后，前为正
    float dev_z = *accel_ptr.z_accel_feedback_ptr;  // 垂直，上为正
    
    // 获取姿态角并转换为弧度
    // 注意：roll和pitch都是顺时针为正（与数学正方向相反）
    float roll_deg = *accel_ptr.roll;    // 绕Y轴（前后轴）旋转，顺时针为正
    float pitch_deg = *accel_ptr.pitch;  // 绕X轴（左右轴）旋转，顺时针为正
    
    // 补偿安装偏移：安装倾斜了-88度，所以实际俯仰角需要加88度
    // 这样当设备物理水平时，补偿后的pitch=0
    float compensated_pitch_deg = pitch_deg + 88.0f;
    
    // 转换为弧度
    float roll_rad = roll_deg * DEGREE_2_RAD;
    float pitch_rad = compensated_pitch_deg * DEGREE_2_RAD;
    
    // 由于用户定义的角度顺时针为正，而标准旋转矩阵使用逆时针为正
    // 所以我们需要取负号
    roll_rad = -roll_rad;     // 转换为数学正方向（逆时针为正）
    pitch_rad = -pitch_rad;   // 转换为数学正方向（逆时针为正）
    
    // 计算三角函数
    float cp = cosf(pitch_rad);  // cos(pitch)
    float sp = sinf(pitch_rad);  // sin(pitch)
    float cr = cosf(roll_rad);   // cos(roll)
    float sr = sinf(roll_rad);   // sin(roll)
    
    // 旋转矩阵：从设备坐标系到地面坐标系
    // 地面坐标系：X前，Y左，Z上
    
    // 正变换：先绕X轴旋转pitch，再绕Y轴旋转roll
    // [地面前后, 地面左右, 地面垂直]^T = R_y(roll) * R_x(pitch) * [设备X, 设备Y, 设备Z]^T
    
    // 地面前后加速度（后为正）
    accel_ptr.x_accel = (cr * dev_x + sr * sp * dev_y + sr * cp * dev_z);
    
    // 地面左右加速度（前为正）
    accel_ptr.y_accel = (-1)*(cp * dev_y - sp * dev_z);
    
    // 如果需要垂直加速度（上为正）：
    // float ground_z = -sr * dev_x + cr * sp * dev_y + cr * cp * dev_z;
}

// go_motor.c 中的DecodeGOIMotor函数修改
static void DecodeGOIMotor(USARTInstance *_instance)
{
    uint8_t* recv_buff = go_motor_usart.motor_usart->recv_buff;
    
    // 检查包头 (接收包头是0xFD 0xEE)
    if(recv_buff[0] == 0xFD && recv_buff[1] == 0xEE)
    {
        // 提取电机ID (低4位)
        uint8_t motor_id = recv_buff[2] & 0x0F;
        
        // 计算CRC (前14字节)
        uint16_t calculated_crc = crc_ccitt(recv_buff, 14);
        // 注意：CRC在接收数据中是**小端序**！先低字节后高字节
        uint16_t received_crc = (recv_buff[15] << 8) | recv_buff[14];  // 正确的小端序读取
        
        if(calculated_crc == received_crc)
        {
            // 查找对应的电机实例
            for (size_t i = 0; i < idx; i++)
            {
                if(go_motor_instance[i]->ctrl_set.ID == motor_id)
                {
                    DaemonReload(go_motor_usart.daemon);
                    
                    // 更新时间戳
                    go_motor_instance[i]->dt = DWT_GetDeltaT(&go_motor_instance[i]->feed_cnt);
                    
                    // 解析转矩反馈 (2字节，小端序，Q8格式)
                    int16_t tqr_raw = (recv_buff[4] << 8) | recv_buff[3];  // 小端序！
                    go_motor_instance[i]->mes.tqr_fb = (float)tqr_raw / 256.0f;
                    
                    // 解析速度反馈 (2字节，小端序，Q8格式，转换为rad/s)
                    int16_t spd_raw = (recv_buff[6] << 8) | recv_buff[5];  // 小端序！
                    go_motor_instance[i]->mes.rot_spd = (float)spd_raw * 2.0f * PI / 256.0f;
                    
                    // 解析位置反馈 (4字节，小端序，Q15格式)
                    int32_t pos_raw = ((int32_t)recv_buff[10] << 24) |  
                                      ((int32_t)recv_buff[9] << 16) |   
                                      ((int32_t)recv_buff[8] << 8) | 
                                      (int32_t)recv_buff[7];
                    go_motor_instance[i]->mes.pos = (float)pos_raw * 2.0f * PI / 32768.0f;
                    
                    // 解析温度 (1字节，有符号)
                    go_motor_instance[i]->mes.tem = (int8_t)recv_buff[11];
                    
                    // 解析错误标志 (第12字节的高3位)
                    go_motor_instance[i]->mes.wrong = (recv_buff[12] >> 5) & 0x07;
                    
                    // 设置485标志
                    go_motor_instance[i]->rs485_flag = 1;
                    break;
                }
            }
        }
    }
    
    // 清空接收缓冲区
    memset(recv_buff, 0, go_motor_usart.motor_usart->recv_buff_size);
}

static void GOMotorLostCallback(void *motor_ptr)
{

}

static void GOMotorPacker(GOMotorInstance* _instance)
{
    // 构建发送数据结构
    _instance->ctrl_send.head[0] = 0xFE;  // 发送包头
    _instance->ctrl_send.head[1] = 0xEE;
    
    // 设置模式字节 (低4位: ID, 中间3位: Mode, 最高1位: none=0)
    // 官方是: id:4, status:3, none:1
    _instance->ctrl_send.Set = (_instance->ctrl_set.ID & 0x0F) |          // 低4位
                               ((_instance->ctrl_set.Mode & 0x07) << 4);  // 中间3位
    
    // 转矩期望 (Q8格式: 直接乘以256)
    // 官方: motor_s->T * 256
    _instance->ctrl_send.tqr_set = (int16_t)(_instance->ctrl_set.tqr_ft * 256.0f);
    
    // 速度期望 (Q8格式: 除以2π再乘以256)
    // 官方: motor_s->W / 6.2832f * 256
    _instance->ctrl_send.spd_set = (int16_t)(_instance->ctrl_set.spd_des * 256.0f / (2.0f * PI));
    
    // 位置期望 (Q15格式: 除以2π再乘以32768)
    // 官方: motor_s->Pos / 6.2832f * 32768
    _instance->ctrl_send.pos_set = (int32_t)(_instance->ctrl_set.pos_des * 32768.0f / (2.0f * PI));
    
    // 位置刚度 (Q15格式: 除以25.6再乘以32768)
    // 官方: motor_s->K_P / 25.6f * 32768
    _instance->ctrl_send.Kpos = (int16_t)(_instance->ctrl_set.Kp * 32768.0f / 25.6f);
    
    // 速度刚度 (Q15格式: 除以25.6再乘以32768)
    // 官方: motor_s->K_W / 25.6f * 32768
    _instance->ctrl_send.Kspd = (int16_t)(_instance->ctrl_set.Kd * 32768.0f / 25.6f);
    
    // 计算CRC (前15字节)
    uint8_t temp_buffer[15];
    memcpy(temp_buffer, &_instance->ctrl_send, 15);
    
    // 注意：官方crc_ccitt函数的第一个参数是0，但你的实现默认crc初始值为0
    _instance->ctrl_send.crc = crc_ccitt(temp_buffer, 15);
}

static void GOMotorSend(GOMotorInstance* _instance)
{
    GOMotorPacker(_instance);
    if(_instance->rs485_flag == 1)
    {
            USARTSend(go_motor_usart.motor_usart, (uint8_t*)&_instance->ctrl_send, 17, USART_TRANSFER_DMA);
            _instance->rs485_flag = 0;
    }
}

void GOMotorEnable(GOMotorInstance* _instance)
{
    _instance->stop_flag = MOTOR_ENALBED;
    _instance->ctrl_set.Mode = foc;
    _instance->rs485_flag = 1;
    _instance->enable_pid = 1;
    //GOMotorSend(_instance);
}

void GOMotorStop(GOMotorInstance* _instance)
{
    _instance->rs485_flag = 1;
    _instance->enable_pid = 0;
    GOMotorSetTorque(_instance,0.0);
    //GOMotorSend(_instance);
}

GOMotorInstance *GOmotorInit(Other_Motor_Init_Config_s config)
{
    GOMotorInstance *instance = (GOMotorInstance*)malloc(sizeof(GOMotorInstance));
    memset(instance, 0, sizeof(GOMotorInstance));
    if (idx >= GO_MOTOR_CNT) // 超过最大实例数
        while (1)
            LOGERROR("GO MOTOR exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
                if (go_motor_id[i] == config.usart_init_config.id)
                    while (1)
                        LOGERROR("[bsp_usart] USART485 ID [%d]already registered!",config.usart_init_config.id);
    
    // 修复：结构体赋值顺序与 .h 定义完全一致，避免偏移
    instance->ctrl_set.ID = config.conf.ID;
    instance->ctrl_set.Kp = config.conf.Kp;
    instance->ctrl_set.Kd = config.conf.Kd;
    instance->ctrl_set.Mode = config.conf.Mode;
    
    // 赋值 controller（GOMotor_Control_Setting_s 类型）
    instance->controller.jump_rad = config.controller_setting_init_config.jump_rad;
    instance->controller.motor_reverse_flag = config.controller_setting_init_config.motor_reverse_flag;
    instance->controller.motor_position_flag = config.controller_setting_init_config.motor_position_flag;
    instance->controller.k_accel_x = config.controller_setting_init_config.k_accel_x;
    instance->controller.k_accel_y = config.controller_setting_init_config.k_accel_y;
    instance->controller.Kpos = config.controller_setting_init_config.Kpos;
    instance->controller.Kspd = config.controller_setting_init_config.Kspd;
    instance->controller.pos_comm = config.controller_setting_init_config.pos_comm;
    
    // 赋值 motor_settings（Motor_Control_Setting_s 类型）
    instance->motor_settings = config.c_ontroller_setting_init_config; // 正反转,闭环类型等
    
    // PID初始化
    PIDInit(&instance->motor_controller.current_PID, &config.c_ontroller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config.c_ontroller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config.c_ontroller_param_init_config.angle_PID);

    instance->motor_controller.other_angle_feedback_ptr = &instance->mes.pos;
    instance->motor_controller.other_speed_feedback_ptr = &instance->mes.rot_spd;
    instance->motor_controller.current_feedforward_ptr = &instance->mes.tqr_fb;
    instance->accel_filtered =0.0;
    if(go_motor_usart_enable==0)
    {
        config.usart_init_config.module_callback = DecodeGOIMotor;
        config.usart_init_config.recv_buff_size = 16*sizeof(char);
        go_motor_usart.motor_usart = USARTRegister(&config.usart_init_config);
        power_switch_init(go_motor_usart.motor_usart,NULL);
        // 注册守护线程
        Daemon_Init_Config_s daemon_config = {
            .callback = GOMotorLostCallback,
            .owner_id = instance,
            .reload_count = 20, // 200ms未收到数据则丢失
        };
        go_motor_usart.daemon = DaemonRegister(&daemon_config);
        go_motor_usart_enable=1;
    }
    go_motor_instance[idx++] = instance;
    GOMotorEnable(instance);
    return instance;
}

// 位置控制模式
void GOMotorSetPosition(GOMotorInstance* _instance, float32_t position_rad,float32_t kp)
{
    _instance->rs485_flag = 1;
    _instance->ctrl_set.pos_des = position_rad * 6.33f; // 考虑减速比
    _instance->ctrl_set.spd_des = 0.0f;
    _instance->ctrl_set.tqr_ft = 0.0f;
    _instance->ctrl_set.Kp = kp; // 示例刚度
    _instance->ctrl_set.Kd = 0.0f;
}

// 速度控制模式（速度为0，纯阻尼） 
void GOMotorSetSpeed(GOMotorInstance* _instance, float32_t speed_rad_s,float32_t kd)
{
    _instance->rs485_flag = 1;
    _instance->ctrl_set.pos_des = 0.0f;
    _instance->ctrl_set.spd_des = speed_rad_s * 6.33f; // 考虑减速比
    _instance->ctrl_set.tqr_ft = 0.0f;
    _instance->ctrl_set.Kp = 0.0f;
    _instance->ctrl_set.Kd = kd; // 示例阻尼
}

// 力矩控制模式（力矩为0，克服摩擦）
void GOMotorSetTorque(GOMotorInstance* _instance, float32_t torque_nm)
{
    _instance->rs485_flag = 1;
    _instance->ctrl_set.pos_des = 0.0f;
    _instance->ctrl_set.spd_des = 0.0f;
    _instance->ctrl_set.tqr_ft = torque_nm;
    _instance->ctrl_set.Kp = 0.0f;
    _instance->ctrl_set.Kd = 0.0f;
}

// 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
Motor_Control_Setting_s *motor_setting; // 电机控制参数
Motor_Controller_s *motor_controller;   // 电机控制器
GOMotor_Control_Setting_s *motor_controller_;
Motor_messure *measure;
static float accel_k;

// 设置参考值
// 终极安全版，彻底解决野值问题
void GoMotorSetRef(GOMotorInstance *motor, float ref)
{
    // 空指针直接退出
    if(motor == NULL) return;

    // 只做一件事：赋值！不访问任何可能损坏的成员
    motor->motor_controller.pid_ref = ref;
}
//自定义pid控制函数
void GOMotorcontrol(GOMotorInstance* _instance)
{
    if(_instance->enable_pid==0)
    {
        //GOMotorSetTorque(_instance,0);
        return;
    }
    motor_controller = &_instance->motor_controller;
    motor_setting = &_instance->motor_settings;
    motor_controller_ = &_instance->controller;
    measure = &_instance->mes;
    float pid_measure=0, pid_ref=motor_controller->pid_ref; 
    if((pid_ref-*motor_controller->other_angle_feedback_ptr)*motor_controller_->motor_reverse_flag>0)
    {
        switch(_instance->ctrl_set.ID)
        {
            case 0:
                motor_controller->speed_PID.Kp=0.018;
            break;
            case 2:
                motor_controller->speed_PID.Kp=0.018;
            break;
            case 1:
                motor_controller->speed_PID.Kp=0.022;
            break;
            case 3:
                motor_controller->speed_PID.Kp=0.022;
            break;
        }
    }
    else
    {
        switch(_instance->ctrl_set.ID)
        {
            case 0:
                motor_controller->speed_PID.Kp=0.015;
            break;
            case 2:
                motor_controller->speed_PID.Kp=0.015;
            break;
            case 1:
                motor_controller->speed_PID.Kp=0.020;
            break;
            case 3:
                motor_controller->speed_PID.Kp=0.020;
            break;
        }
    }
    // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED){
                if (motor_controller->other_angle_feedback_ptr != NULL) {
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                } else {
                    // 错误处理
                }
            }
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->tqr_fb, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;
        // 获取最终输出
        if(_instance->enable_accel_feedback == 1)
        {
            inverse_transform();
            float accel = accel_ptr.y_accel;

            // ⭐ 每个电机独立滤波（修复串扰）
            _instance->accel_filtered = 0.8f * _instance->accel_filtered + 0.2f * accel;
            float accel_filtered = _instance->accel_filtered;

            // 死区
            if (fabsf(accel_filtered) < 0.2f)
                accel_filtered = 0.0f;

            float accel_ff = -motor_controller_->k_accel_y * accel_filtered;
            pid_ref += accel_ff;

            // 限幅
            if (pid_ref > 50.0f)  pid_ref = 50.0f;
            if (pid_ref < -50.0f) pid_ref = -50.0f;
        }
        GOMotorSetTorque(_instance,pid_ref);
    return;
}

static uint8_t switch_tim_flag=0;

//@Todo: 任务调度
void GOMotorTask(void const *argument)
{
    GOMotorInstance *motor = (GOMotorInstance *)argument;
    HAL_Delay(8);
    while (1)
    {
        if(switch_tim_flag>=5)
        {
            switch_tim_flag=0;
            if(switch_flag==1)
            {
                USARTSend(switch_usart, data_on_usart, 4, USART_TRANSFER_DMA);
            }
            else
            {
                USARTSend(switch_usart, data_off_usart, 4, USART_TRANSFER_DMA);
            }
        }
        else
        {
            switch_tim_flag+=1;
            GOMotorcontrol(motor);
            //GOMotorStop(motor);
            GOMotorSend(motor);
        }
        HAL_Delay(1);
        //DWT_Delay(0.001);//防止通信堵塞，待修改
        osDelay(3);
    }
}

void GOMotorControlInit()
{
    char go_task_name[5] = "go";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char go_id_buff[2] = {0};
        __itoa(i, go_id_buff, 10);
        strcat(go_task_name, go_id_buff);
        // 修复：任务栈大小从128改为512，避免栈溢出（核心！）
        osThreadDef(go_task_name, GOMotorTask, osPriorityNormal, 0, 512);
        go_task_handle[i] = osThreadCreate(osThread(go_task_name), go_motor_instance[i]);
    }
}