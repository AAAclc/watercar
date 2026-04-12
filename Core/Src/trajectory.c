#include "trajectory.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "math.h"

// 系统状态
static SysState_Typedef sys_state = STATE_IDLE;
// 目标坐标
static float target_x = 0.0f, target_y = 0.0f;
// PID控制器
static PosPID_Typedef x_pid, y_pid, theta_pid;
static IncPID_Typedef speed_pid[4];

/**
 * @brief  轨迹控制初始化（PID参数初始化）
 */
void Trajectory_Init(void)
{
    // 位置环PID初始化（输出：m/s）
    PosPID_Init(&x_pid, 4.0f, 0.1f, 0.2f, MAX_VEL, MAX_VEL, -MAX_VEL);
    PosPID_Init(&y_pid, 4.0f, 0.1f, 0.2f, MAX_VEL, MAX_VEL, -MAX_VEL);
    // 航向环PID初始化（输出：rad/s）
    PosPID_Init(&theta_pid, 5.0f, 0.0f, 0.1f, MAX_OMEGA, MAX_OMEGA, -MAX_OMEGA);
    // 速度环PID初始化（输出：PWM值）
    for(uint8_t i=0; i<4; i++)
    {
        IncPID_Init(&speed_pid[i], 8.0f, 0.5f, 0.1f, PWM_MAX, PWM_MIN);
    }
    sys_state = STATE_IDLE;
}

/**
 * @brief  梯形加减速规划
 * @param  distance: 当前距离目标点的距离
 * @retval 最大允许速度
 */
static float Speed_Plan(float distance)
{
    // 计算减速距离
    float decel_dis = (MAX_VEL * MAX_VEL) / (2 * DECEL_MAX);
    float target_vel;

    if(distance > decel_dis)
    {
        // 加速/匀速阶段
        target_vel = sqrtf(2 * ACC_MAX * distance);
        if(target_vel > MAX_VEL) target_vel = MAX_VEL;
    }
    else
    {
        // 减速阶段
        target_vel = sqrtf(2 * DECEL_MAX * distance);
    }
    // 最小速度限制
    if(target_vel < 0.05f && distance > ARRIVE_DIS) target_vel = 0.05f;
    return target_vel;
}

/**
 * @brief  设置目标坐标并启动巡航
 * @param  target_x: 目标x坐标，单位m
 * @param  target_y: 目标y坐标，单位m
 */
void Trajectory_SetTarget(float target_x, float target_y)
{
    // 更新目标坐标
    target_x = target_x;
    target_y = target_y;
    // 复位PID积分
    PID_Reset(NULL, &x_pid);
    PID_Reset(NULL, &y_pid);
    PID_Reset(NULL, &theta_pid);
    for(uint8_t i=0; i<4; i++) PID_Reset(&speed_pid[i], NULL);
    // 切换为运行状态
    sys_state = STATE_RUNNING;
}

/**
 * @brief  停止运动
 */
void Trajectory_Stop(void)
{
    sys_state = STATE_IDLE;
    Motor_StopAll();
    // 复位PID
    PID_Reset(NULL, &x_pid);
    PID_Reset(NULL, &y_pid);
    PID_Reset(NULL, &theta_pid);
    for(uint8_t i=0; i<4; i++) PID_Reset(&speed_pid[i], NULL);
}

/**
 * @brief  复位原点（当前位置设为(0,0)）
 */
void Trajectory_ResetOrigin(void)
{
    Trajectory_Stop();
    Odom_Reset();
}

/**
 * @brief  轨迹控制主循环（必须10ms固定周期调用）
 */
void Trajectory_ControlLoop(void)
{
    Odom_Typedef *odom = Odom_GetInfo();
    float wheel_target_rpm[4];
    ChassisSpeed_Typedef target_speed;

    // 空闲/到达状态：不执行控制
    if(sys_state == STATE_IDLE || sys_state == STATE_ARRIVED) return;

    // 1. 计算当前与目标点的偏差
    float err_x = target_x - odom->x;
    float err_y = target_y - odom->y;
    float distance = sqrtf(err_x*err_x + err_y*err_y);

    // 2. 到达判断
    if(distance < ARRIVE_DIS)
    {
        sys_state = STATE_ARRIVED;
        Motor_StopAll();
        return;
    }

    // 3. 速度规划
    float vel_limit = Speed_Plan(distance);

    // 4. 位置环PID计算
    x_pid.target = target_x;
    x_pid.current = odom->x;
    PosPID_Calc(&x_pid);

    y_pid.target = target_y;
    y_pid.current = odom->y;
    PosPID_Calc(&y_pid);

    // 航向环PID：保持车头朝前（theta=0）
    theta_pid.target = 0.0f;
    theta_pid.current = odom->theta;
    PosPID_Calc(&theta_pid);

    // 5. 合速度限幅
    target_speed.vx = x_pid.output;
    target_speed.vy = y_pid.output;
    float vel_total = sqrtf(target_speed.vx*target_speed.vx + target_speed.vy*target_speed.vy);
    if(vel_total > vel_limit)
    {
        target_speed.vx = target_speed.vx * vel_limit / vel_total;
        target_speed.vy = target_speed.vy * vel_limit / vel_total;
    }
    target_speed.omega = theta_pid.output;

    // 6. 逆运动学解算，得到轮子目标转速
    Mecanum_InverseCalc(&target_speed, wheel_target_rpm);

    // 7. 速度环PID计算，输出PWM
    for(uint8_t i=0; i<4; i++)
    {
        speed_pid[i].target = wheel_target_rpm[i];
        speed_pid[i].current = Encoder_GetRPM(i);
        IncPID_Calc(&speed_pid[i]);
        Motor_SetPWM(i, (int32_t)speed_pid[i].output);
    }
}

/**
 * @brief  获取系统当前状态
 * @retval 系统状态枚举
 */
SysState_Typedef Trajectory_GetState(void)
{
    return sys_state;
}