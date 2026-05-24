/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "general_def.h"
#include "master_process.h"
#include "arm_math.h"

static INS_t INS;
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    while (BMI088Init(&hspi2, 1) != BMI088_NO_ERROR)
        ;
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // imu heat init
    PID_Init_Config_s config = {.MaxOut = 800,
                                .IntegralLimit = 80,
                                .DeadBand = 0,
                                .Kp = 400,
                                .Ki = 5,
                                .Kd = 0,
                                .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

/*要以1khz运行*/
void INS_Task(void)
{
    static uint32_t count = 0;
    static uint8_t ekf_init_flag = 0; // EKF初始化标志
    const float gravity[3] = {0, 0, 9.81f}; // 导航系重力沿Z轴向下

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // 上电首次执行：重置EKF四元数，消除初始偏置
    if (ekf_init_flag == 0)
    {
        // 重置四元数为单位矩阵（初始姿态：yaw/roll/pitch=0）
        QEKF_INS.q[0] = 1.0f;
        QEKF_INS.q[1] = 0.0f;
        QEKF_INS.q[2] = 0.0f;
        QEKF_INS.q[3] = 0.0f;
        ekf_init_flag = 1; // 只初始化一次
    }

    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);

        // 原始数据读取
        float raw_accel[3] = {BMI088.Accel[X], BMI088.Accel[Y], BMI088.Accel[Z]};
        float raw_gyro[3] = {BMI088.Gyro[X], BMI088.Gyro[Y], BMI088.Gyro[Z]};

        // 关键修正：假设你的roll轴实际是Y轴（而非X轴），绕Y轴逆时针旋转90度
        // 若仍不对，可切换回X轴（注释Y轴逻辑，启用X轴逻辑）
#ifdef ENABLE_ROLL_90_ROTATION
        float rotated_accel[3];
        float rotated_gyro[3];

        // ===== 方案1：roll轴为Y轴（优先尝试，解决初始角度错误）=====
        // 绕Y轴逆时针90度的旋转矩阵（右手定则：拇指沿Y轴正向）
        // [X; Y; Z] = [0  0  1; 
        //              0  1  0; 
        //             -1  0  0] * 原始坐标
        rotated_accel[X] = raw_accel[Z];    // X' = Z
        rotated_accel[Y] = raw_accel[Y];    // Y轴不变（roll轴）
        rotated_accel[Z] = -raw_accel[X];   // Z' = -X
        
        rotated_gyro[X] = raw_gyro[Z];      // 陀螺仪同步修正
        rotated_gyro[Y] = raw_gyro[Y];
        rotated_gyro[Z] = -raw_gyro[X];

        // ===== 方案2：若方案1仍错，改用roll轴为X轴（逆时针-90度，即顺时针90度）=====
        // 取消下面注释，注释方案1即可
        // rotated_accel[X] = raw_accel[X];
        // rotated_accel[Y] = -raw_accel[Z];  // Y' = -Z（顺时针90度）
        // rotated_accel[Z] = raw_accel[Y];   // Z' = Y
        // rotated_gyro[X] = raw_gyro[X];
        // rotated_gyro[Y] = -raw_gyro[Z];
        // rotated_gyro[Z] = raw_gyro[Y];

        // 赋值旋转后的数据
        INS.Accel[X] = rotated_accel[X];
        INS.Accel[Y] = rotated_accel[Y];
        INS.Accel[Z] = rotated_accel[Z];
        INS.Gyro[X] = rotated_gyro[X];
        INS.Gyro[Y] = rotated_gyro[Y];
        INS.Gyro[Z] = rotated_gyro[Z];
#else
        // 原始坐标赋值
        INS.Accel[X] = raw_accel[X];
        INS.Accel[Y] = raw_accel[Y];
        INS.Accel[Z] = raw_accel[Z];
        INS.Gyro[X] = raw_gyro[X];
        INS.Gyro[Y] = raw_gyro[Y];
        INS.Gyro[Z] = raw_gyro[Z];
#endif

        // 安装误差修正（保持不变）
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // EKF更新四元数（核心姿态解算）
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], 
                                 INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系→导航系转换（保持不变）
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 重力向量转换+运动加速度计算（保持不变）
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + 
                                   INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

        // 姿态角赋值（恢复原始逻辑，避免额外偏置）
        INS.Yaw = QEKF_INS.Yaw;
#ifdef ENABLE_ROLL_90_ROTATION
        INS.Pitch = QEKF_INS.Pitch;
#else
        INS.Pitch = -QEKF_INS.Pitch;
#endif
        INS.Roll = QEKF_INS.Roll;
        INS.Gyro[2] = QEKF_INS.Gyro[2];
        INS.YawTotalAngle = -QEKF_INS.YawTotalAngle;

        VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
    }

    // 以下代码保持不变
    if ((count % 2) == 0)
    {
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 调试打印：查看初始角度，确认是否接近0
        // 建议添加串口打印，比如：
        // printf("Yaw:%.1f, Roll:%.1f, Pitch:%.1f\r\n", INS.Yaw, INS.Roll, INS.Pitch);
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
