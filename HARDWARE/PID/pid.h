#ifndef  __PID_H
#define  __PID_H

#include  "main.h"

/* pid参数结构体 */
typedef struct
{
    float target_val;               //目标值
    float actual_val;               //实际值
    float err;                      //定义偏差值
    float err_last;                 //定义上一个偏差值
    float Kp,Ki,Kd;                 //定义比例、积分、微分系数
    float integral;                 //定义积分值
}_pid;

extern uint8_t g_straight_target_angle;
extern uint8_t g_current_angle;
/* 外部变量声明,pid结构体变量 */
extern _pid g_pid_speed1,g_pid_speed2;
extern _pid g_pid_location1,g_pid_location2;
extern _pid g_pid_turn_angle;
extern _pid g_pid_line;

/* pid参数设置函数 */
void pid_param_init(void);
void set_pid_target(_pid *pid,float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid,float p,float i,float d);
void update_current_angle(void);
void set_straight_target_angle(float angle);
/* pid计算函数 */
float location_pid_realize(_pid *pid,float actual_val);
float speed_pid_realize(_pid *pid,float actual_val);
float turn_angle_pid_realize(_pid *pid, float actual_val);
float line_pid_realize(_pid *pid, float actual_val);
float straight_angle_pid_calculate(void);

#endif
