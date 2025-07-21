#include "line.h"
#include "jy61p.h"
#include "pid.h"
#include "motor.h"

/* 循迹PID控制相关变量 */
static _pid line_follow_pid;
static float base_speed = 100.0f;    // 基础速度
static float max_correction = 50.0f; // 最大转向修正值

/**
 * @brief       灰度巡线补偿值获取
 * @param       无
 * @retval      巡线偏移的值
 */
int32_t line_err(void)
{
    uint8_t hw1_val, hw2_val, hw3_val, hw4_val;
    int32_t line_num;

    /* 读取并记录电平状态，这里根据实际01代表的情况修改，可能相反，传感器越多效果越好 */
    if (HW1 == 0)
    {
        hw1_val = 1;
    }
    else
    {
        hw1_val = 0;
    }
    if (HW2 == 0)
    {
        hw2_val = 1;
    }
    else
    {
        hw2_val = 0;
    }
    if (HW3 == 0)
    {
        hw3_val = 1;
    }
    else
    {
        hw3_val = 0;
    }
    if (HW4 == 0)
    {
        hw4_val = 1;
    }
    else
    {
        hw4_val = 0;
    }

    /* 巡线值 */
    if (hw1_val == 0 && hw2_val == 0 && hw3_val == 0 && hw4_val == 0)
        line_num = 0;
    if (hw1_val == 1 && hw2_val == 0 && hw3_val == 0 && hw4_val == 0)
        line_num = 10;
    if (hw1_val == 0 && hw2_val == 1 && hw3_val == 0 && hw4_val == 0)
        line_num = 20;
    if (hw1_val == 0 && hw2_val == 0 && hw3_val == 1 && hw4_val == 0)
        line_num = -10;
    if (hw1_val == 0 && hw2_val == 0 && hw3_val == 0 && hw4_val == 1)
        line_num = -20;

    if (hw1_val == 0 && hw2_val == 1 && hw3_val == 1 && hw4_val == 0)
        line_num = 0;
    if (hw1_val == 1 && hw2_val == 1 && hw3_val == 0 && hw4_val == 0)
        line_num = 15;
    if (hw1_val == 0 && hw2_val == 0 && hw3_val == 1 && hw4_val == 1)
        line_num = -15;

    return line_num;
}

/**
 * @brief       循迹PID参数初始化
 * @param       无
 * @retval      无
 */
void line_follow_pid_init(void)
{
    /* 循迹PID参数初始化 */
    line_follow_pid.target_val = 0.0f; // 目标值为0，表示小车应该在线的中央
    line_follow_pid.actual_val = 0.0f;
    line_follow_pid.err = 0.0f;
    line_follow_pid.err_last = 0.0f;
    line_follow_pid.integral = 0.0f;

    /* PID参数设置（需要根据实际情况调试） */
    line_follow_pid.Kp = 2.0f; // 比例系数
    line_follow_pid.Ki = 0.1f; // 积分系数
    line_follow_pid.Kd = 0.5f; // 微分系数
}

/**
 * @brief       循迹PID控制计算
 * @param       line_error: 循迹偏差值（从line_err()函数获取）
 * @retval      PID输出值（转向修正量）
 */
float line_follow_pid_calculate(int32_t line_error)
{
    float pid_output;

    /* 设置目标值和实际值 */
    line_follow_pid.target_val = 0.0f;              // 目标是偏差为0
    line_follow_pid.actual_val = (float)line_error; // 当前偏差

    /* 计算误差 */
    line_follow_pid.err = line_follow_pid.target_val - line_follow_pid.actual_val;

    /* 积分项累加 */
    line_follow_pid.integral += line_follow_pid.err;

    /* 积分限幅防止积分饱和 */
    if (line_follow_pid.integral > 100.0f)
        line_follow_pid.integral = 100.0f;
    else if (line_follow_pid.integral < -100.0f)
        line_follow_pid.integral = -100.0f;

    /* PID计算 */
    pid_output = line_follow_pid.Kp * line_follow_pid.err +
                 line_follow_pid.Ki * line_follow_pid.integral +
                 line_follow_pid.Kd * (line_follow_pid.err - line_follow_pid.err_last);

    /* 更新上次误差 */
    line_follow_pid.err_last = line_follow_pid.err;

    /* 输出限幅 */
    if (pid_output > max_correction)
        pid_output = max_correction;
    else if (pid_output < -max_correction)
        pid_output = -max_correction;

    return pid_output;
}

/**
 * @brief       循迹控制主函数
 * @param       无
 * @retval      无
 * @note        此函数应在定时器中断中调用，建议20ms周期
 */
void line_follow_control(void)
{
    int32_t line_error;
    float pid_correction;
    float left_motor_speed, right_motor_speed;

    /* 1. 获取循迹偏差 */
    line_error = line_err();

    /* 2. PID计算得到转向修正量 */
    pid_correction = line_follow_pid_calculate(line_error);

    /* 3. 根据PID输出计算左右电机速度 */
    left_motor_speed = base_speed - pid_correction;  // 左偏时减少左轮速度
    right_motor_speed = base_speed + pid_correction; // 左偏时增加右轮速度

    /* 4. 速度限幅 */
    if (left_motor_speed > 200.0f)
        left_motor_speed = 200.0f;
    if (left_motor_speed < 0.0f)
        left_motor_speed = 0.0f;
    if (right_motor_speed > 200.0f)
        right_motor_speed = 200.0f;
    if (right_motor_speed < 0.0f)
        right_motor_speed = 0.0f;

    /* 5. 设置电机速度目标值（这里需要调用您的电机控制函数） */
    set_pid_target(&g_pid_speed1, left_motor_speed);
    set_pid_target(&g_pid_speed2, right_motor_speed);
}

/**
 * @brief       设置循迹基础速度
 * @param       speed: 基础速度值
 * @retval      无
 */
void set_line_follow_base_speed(float speed)
{
    base_speed = speed;
}

/**
 * @brief       设置循迹PID参数
 * @param       kp: 比例系数
 * @param       ki: 积分系数
 * @param       kd: 微分系数
 * @retval      无
 */
void set_line_follow_pid_params(float kp, float ki, float kd)
{
    line_follow_pid.Kp = kp;
    line_follow_pid.Ki = ki;
    line_follow_pid.Kd = kd;
}

/**
 * @brief       角度巡线补偿值获取
 * @param       无
 * @retval      角度偏移量
 */
/* 以0度为基准 */
int32_t yaw_err0(void)
{
    int32_t yaw_num;
    if ((g_yaw_jy61 > 0) && (g_yaw_jy61 < 90))
        yaw_num = g_yaw_jy61;
    else if ((g_yaw_jy61 > 270) && (g_yaw_jy61 < 360))
        yaw_num = g_yaw_jy61 - 360;
    else
        yaw_num = 0;
    return yaw_num;
}
/* 掉头后以180度为基准 */
int32_t yaw_err180(void)
{
    int32_t yaw_num;
    if ((g_yaw_jy61 > 180) && (g_yaw_jy61 < 270))
        yaw_num = g_yaw_jy61 - 180;
    else if ((g_yaw_jy61 > 90) && (g_yaw_jy61 < 180))
        yaw_num = g_yaw_jy61 - 180;
    else
        yaw_num = 0;
    return yaw_num;
}
