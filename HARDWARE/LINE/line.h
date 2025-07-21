#ifndef __LINE_H
#define __LINE_H

#include "main.h"
#include "pid.h"

#define HW1 HAL_GPIO_ReadPin(HW1_GPIO_Port, HW1_Pin) // 读取红外对管连接的GPIO电平
#define HW2 HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin)
#define HW3 HAL_GPIO_ReadPin(HW3_GPIO_Port, HW3_Pin)
#define HW4 HAL_GPIO_ReadPin(HW4_GPIO_Port, HW4_Pin)

/* 原有函数声明 */
int32_t line_err(void);
int32_t yaw_err0(void);
int32_t yaw_err180(void);

/* 新增的循迹PID控制函数声明 */
void line_follow_pid_init(void);
float line_follow_pid_calculate(int32_t line_error);
void line_follow_control(void);
void set_line_follow_base_speed(float speed);
void set_line_follow_pid_params(float kp, float ki, float kd);

#endif
