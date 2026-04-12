#include "pid.h"
#include "math.h"

/**
 * @brief  增量式PID初始化
 * @param  pid: PID结构体指针
 * @param  Kp/Ki/Kd: PID参数
 * @param  output_max/output_min: 输出限幅
 */
void IncPID_Init(IncPID_Typedef *pid, float Kp, float Ki, float Kd, float output_max, float output_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  增量式PID计算
 * @param  pid: PID结构体指针
 */
void IncPID_Calc(IncPID_Typedef *pid)
{
    pid->err[0] = pid->target - pid->current;
    // 增量式核心公式
    float delta_u = pid->Kp * (pid->err[0] - pid->err[1])
                  + pid->Ki * pid->err[0]
                  + pid->Kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);
    pid->output += delta_u;
    // 输出限幅
    if(pid->output > pid->output_max) pid->output = pid->output_max;
    if(pid->output < pid->output_min) pid->output = pid->output_min;
    // 误差更新
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
}

/**
 * @brief  位置式PID初始化
 * @param  pid: PID结构体指针
 * @param  Kp/Ki/Kd: PID参数
 * @param  integral_max: 积分限幅
 * @param  output_max/output_min: 输出限幅
 */
void PosPID_Init(PosPID_Typedef *pid, float Kp, float Ki, float Kd, float integral_max, float output_max, float output_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->err = pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  位置式PID计算
 * @param  pid: PID结构体指针
 */
void PosPID_Calc(PosPID_Typedef *pid)
{
    pid->err = pid->target - pid->current;
    // 积分限幅（仅小误差时积分，避免积分饱和）
    if(fabsf(pid->err) < 0.02f)
    {
        pid->integral += pid->err;
        if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
        if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    }
    // 位置式核心公式
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->integral
                + pid->Kd * (pid->err - pid->err_last);
    // 输出限幅
    if(pid->output > pid->output_max) pid->output = pid->output_max;
    if(pid->output < pid->output_min) pid->output = pid->output_min;
    // 误差更新
    pid->err_last = pid->err;
}

/**
 * @brief  PID参数复位清零
 * @param  inc_pid: 增量式PID指针（NULL则不处理）
 * @param  pos_pid: 位置式PID指针（NULL则不处理）
 */
void PID_Reset(IncPID_Typedef *inc_pid, PosPID_Typedef *pos_pid)
{
    if(inc_pid != NULL)
    {
        inc_pid->target = 0.0f;
        inc_pid->err[0] = inc_pid->err[1] = inc_pid->err[2] = 0.0f;
        inc_pid->output = 0.0f;
    }
    if(pos_pid != NULL)
    {
        pos_pid->target = 0.0f;
        pos_pid->err = pos_pid->err_last = 0.0f;
        pos_pid->integral = 0.0f;
        pos_pid->output = 0.0f;
    }
}