#include "pid.h"
#define limit_abs(x, limit) ((x) > (limit) ? (limit) : ((x) < -(limit) ? -(limit) : (x)))

// 定义全局变量
_pid g_pid_speed1, g_pid_speed2;
_pid g_pid_location1, g_pid_location2;
_pid g_pid_turn_angle;
_pid g_pid_line;
_pid g_pid_straight;

/**
 * @brief  PID参数初始化
 * @note   无
 * @retval 无
 */
void pid_param_init(void)
{
  /* 电机1参数初始化 */
  /* ***** */
  /* 位置相关初始化参数 */
  g_pid_location1.target_val = 0.0;
  g_pid_location1.actual_val = 0.0;
  g_pid_location1.err = 0.0;
  g_pid_location1.err_last = 0.0;
  g_pid_location1.integral = 0.0;

  g_pid_location1.Kp = 0.3;
  g_pid_location1.Ki = 0.0;
  g_pid_location1.Kd = 0.0;

  /* 速度相关初始化参数 */
  g_pid_speed1.target_val = 0.0;
  g_pid_speed1.actual_val = 0.0;
  g_pid_speed1.err = 0.0;
  g_pid_speed1.err_last = 0.0;
  g_pid_speed1.integral = 0.0;

  g_pid_speed1.Kp = 6.0;
  g_pid_speed1.Ki = 8.5;
  g_pid_speed1.Kd = 0.0;
  /* ***** */

  /* 电机2参数初始化 */
  /* ***** */
  /* 位置相关初始化参数 */
  g_pid_location2.target_val = 0.0;
  g_pid_location2.actual_val = 0.0;
  g_pid_location2.err = 0.0;
  g_pid_location2.err_last = 0.0;
  g_pid_location2.integral = 0.0;

  g_pid_location2.Kp = 0.3;
  g_pid_location2.Ki = 0.0;
  g_pid_location2.Kd = 0.0;

  /* 速度相关初始化参数 */
  g_pid_speed2.target_val = 0.0;
  g_pid_speed2.actual_val = 0.0;
  g_pid_speed2.err = 0.0;
  g_pid_speed2.err_last = 0.0;
  g_pid_speed2.integral = 0.0;

  g_pid_speed2.Kp = 6.0;
  g_pid_speed2.Ki = 8.5;
  g_pid_speed2.Kd = 0.0;
  /* ***** */

  /* 角度环相关初始化参数 */
  g_pid_turn_angle.target_val = 0.0;
  g_pid_turn_angle.actual_val = 0.0;
  g_pid_turn_angle.err = 0.0;
  g_pid_turn_angle.err_last = 0.0;
  g_pid_turn_angle.integral = 0.0;

  g_pid_turn_angle.Kp = 1.2;
  g_pid_turn_angle.Ki = 0.0;
  g_pid_turn_angle.Kd = 0.0;

  /* 转向速度相关初始化参数 */
  g_pid_line.target_val = 0.0;
  g_pid_line.actual_val = 0.0;
  g_pid_line.err = 0.0;
  g_pid_line.err_last = 0.0;
  g_pid_line.integral = 0.0;

  g_pid_line.Kp = 4.0;
  g_pid_line.Ki = 0.0;
  g_pid_line.Kd = 1.0;

  g_pid_straight.Kp = 2.0; // 恢复到合理的比例参数
  g_pid_straight.Ki = 0.1; // 恢复到小的积分项
  g_pid_straight.Kd = 0.3; // 恢复到合理的微分参数
  g_pid_straight.target_val = 0.0;
  g_pid_straight.actual_val = 0.0;
  g_pid_straight.err = 0.0;
  g_pid_straight.err_last = 0.0;
  g_pid_straight.integral = 0.0;

  // #if defined(PID_ASSISTANT_EN)
  //     float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
  //     set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
  // #endif
}

/**
 * @brief  设置目标值
 * @param  pid结构体,temp_val目标值
 *	@note   无
 * @retval 无
 */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val; // 设置当前的目标值
}

/**
 * @brief  获取目标值
 * @param  pid结构体
 * @note   无
 * @retval pid目标值
 */
float get_pid_target(_pid *pid)
{
  return pid->target_val; // 设置当前的目标值
}

/**
 * @brief       pid结构体, p,i,d值
 * @param       无
 * @retval      无
 */
void set_p_i_d(_pid *pid, float p, float i, float d)
{
  pid->Kp = p; // 设置比例系数 P
  pid->Ki = i; // 设置积分系数 I
  pid->Kd = d; // 设置微分系数 D
}

/**
 * @brief  位置PID算法实现
 * @param  actual_val:实际值
 * @note   无
 * @retval 通过PID计算后的输出
 */
float location_pid_realize(_pid *pid, float actual_val) // 位置环光个Kp好像也可以
{
  /*计算目标值与实际值的误差*/
  pid->err = pid->target_val - actual_val;

  //    /* 设定闭环死区 */   //外环死区可以不要         //调试时注释掉
  //    if((pid->err >= -0.1) && (pid->err <= 0.1))
  //    {
  //      pid->err = 0;
  //      pid->integral = 0;
  //    }

  pid->integral += pid->err; // 误差累积

  /*PID算法实现*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*误差传递*/
  pid->err_last = pid->err;

  /*返回当前实际值*/
  return pid->actual_val;
}

/**
 * @brief  速度PID算法实现
 * @param  actual_val:实际值
 * @note   无
 * @retval 通过PID计算后的输出
 */
float speed_pid_realize(_pid *pid, float actual_val)
{
  /*计算目标值与实际值的误差*/
  pid->err = pid->target_val - actual_val;

  /* 设定闭环死区 */
  /* 误差在-0.5-0.5之间时不再当做误差进行计算 */ // 调试时注释掉
  if ((pid->err < 0.5f) && (pid->err > -0.5f))   // 差1这么多可以吗？运行1分钟，位置差为1个轮子的周长
  {
    pid->err = 0.0f;
  }

  pid->integral += pid->err; // 误差累积

  /*积分限幅*/ // 调试时注释掉
  if (pid->integral >= 1000)
  {
    pid->integral = 1000;
  } // 待确定
  else if (pid->integral < -1000)
  {
    pid->integral = -1000;
  }

  /*PID算法实现*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*误差传递*/
  pid->err_last = pid->err;

  /*返回当前实际值*/
  return pid->actual_val; // 这个值是速度环的输出值，并不是当前电机的实际值
}

/**
 * @brief       角度环PID算法实现
 * @param       actual_val:实际值
 * @retval      通过PID计算后的输出
 */
float turn_angle_pid_realize(_pid *pid, float actual_val)
{
  /*计算目标值与实际值的误差*/
  pid->err = pid->target_val - actual_val;

  float raw_err = pid->target_val - actual_val;

  /* 设定闭环死区 */ // 外环死区可以不要
  //    if((pid->err >= -0.5f) && (pid->err <= 0.5f))
  //    {
  //      pid->err = 0;
  //      pid->integral = 0;
  //    }

  if (raw_err > 180.0f) // 如果误差大于180度，说明需要逆时针转动
  {
    pid->err -= 358.0f; // 将误差调整到-180到180度之间
  }
  else if (raw_err < -180.0f) // 如果误差小于-180度，说明需要顺时针转动
  {
    pid->err += 358.0f; // 将误差调整到-180到180度之间
  }

  if ((pid->err >= 358.0f) || (pid->err <= -358.0f)) // 修改这里的值可以解决陀螺仪边界值突变问题
  {
    pid->err = 0;
    pid->integral = 0;
  }

  pid->integral += pid->err; // 误差累积

  limit_abs(pid->integral, 1000); // 积分限幅

  /*PID算法实现*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*误差传递*/
  pid->err_last = pid->err;

  /*返回当前实际值*/
  return pid->actual_val;
}

/**
 * @brief       巡线环PID实现
 * @param       actual_val:实际值
 * @retval      通过PID计算后的输出
 */
// float line_pid_realize(_pid *pid, float actual_val)
// {
//     /*计算目标值与实际值的误差*/
//     pid->err=pid->target_val-actual_val;

//     if((pid->err<1 ) && (pid->err>-1))   //假如以最大允许速度偏差运行1分钟，输出轴最大偏差为半圈
//     {
//         pid->err = 1;
//     }

//     pid->integral += pid->err;    // 误差累积

//     /*积分限幅*/
//     if (pid->integral >= 1000) {pid->integral =1000;}
//     else if (pid->integral < -1000)  {pid->integral = -1000;}

//     /*PID算法实现*/
//     pid->actual_val = pid->Kp*pid->err
//                         +pid->Ki*pid->integral
//                             +pid->Kd*(pid->err-pid->err_last);

//     /*误差传递*/
//     pid->err_last=pid->err;

//     /*返回当前实际值*/
//     return pid->actual_val;
// }

float straight_pid_realize(_pid *p, float current)
{
  /*
  该函数已经在参数表中给出target，不需要与pid_set_target配套使用
  */
  p->err = p->target_val - current; // 计算误差

  // 角度误差归一化到[-180, 180]范围
  while (p->err > 180.0f)
    p->err -= 360.0f;
  while (p->err < -180.0f)
    p->err += 360.0f;

  // 积分项累积（添加积分限幅防止积分饱和）
  p->integral += p->err;
  if (p->integral > 50.0f)
    p->integral = 50.0f;
  if (p->integral < -50.0f)
    p->integral = -50.0f;

  // PID计算
  p->actual_val = p->Kp * p->err +
                  p->Ki * p->integral +
                  p->Kd * (p->err - p->err_last);

  // 输出限幅，防止角度补偿过大
  p->actual_val = limit_abs(p->actual_val, 800.0f);

  // 更新上次误差
  p->err_last = p->err;

  return p->actual_val; // 返回PID输出值
}
