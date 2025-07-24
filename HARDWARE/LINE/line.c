#include "line.h"
#include "jy61p.h"
#include "control.h"
// 根据最大速度7200，放大PID参数，提升补偿效果
#define LINE_KP 8.0f   // 推荐比例参数（原1.2，放大到8）
#define LINE_KI 0.03f  // 推荐积分参数（原0.01，略微提升）
#define LINE_KD 0.15f  // 推荐微分参数（原0.08，略微提升）

static float line_pid_integral = 0;
static float line_pid_last_err = 0;

// 灰度寻迹PID控制函数
float line_pid_realize(int err)
{
    float p, i, d, out;
    p = LINE_KP * err;
    line_pid_integral += err;
    i = LINE_KI * line_pid_integral;
    d = LINE_KD * (err - line_pid_last_err);
    out = p + i + d;
    line_pid_last_err = err;
    // 积分限幅，防止积分过大
    if(line_pid_integral > 5000) line_pid_integral = 5000;
    if(line_pid_integral < -5000) line_pid_integral = -5000;
    return out;
}
#include "line.h"
#include "jy61p.h"

#include <stdint.h>

/**
 * @brief       灰度巡线补偿值获取
 * @param       无
 * @retval      巡线偏移的值
 */
int32_t line_err(void)
{
    // 灰度寻迹加权平均法，权重按物理位置分布，间隔0.5cm，中心为0
    int sensor_val[8];
    float sensor_pos[8] = {1.75, 1.25, 0.75, 0.25, -0.25, -0.75, -1.25, -1.75}; // 单位cm，中心为0
    float sum = 0, weight = 0;
    // 灰度传感器编号：1~8，左到右
    sensor_val[0] = (HW1==1)?1:0;
    sensor_val[1] = (HW2==1)?1:0;
    sensor_val[2] = (HW3==1)?1:0;
    sensor_val[3] = (HW4==1)?1:0;
    sensor_val[4] = (HW5==1)?1:0;
    sensor_val[5] = (HW6==1)?1:0;
    sensor_val[6] = (HW7==1)?1:0;
    sensor_val[7] = (HW8==1)?1:0;
    for(int i=0; i<8; i++) {
        sum += sensor_val[i] * sensor_pos[i];
        weight += sensor_val[i];
    }
    if(weight == 0) {
        return 0; // 无线，保持直行
    } else {
        return (int32_t)(sum / weight * 1000); // 返回偏移量，单位mm，放大1000便于PID使用
    }
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
    if((g_yaw_jy61>0)&&(g_yaw_jy61<90)) yaw_num = g_yaw_jy61;
    else if((g_yaw_jy61>270)&&(g_yaw_jy61<360)) yaw_num = g_yaw_jy61-360;
    else yaw_num=0;
    return yaw_num;
}
/* 掉头后以180度为基准 */
int32_t yaw_err180(void)
{
    int32_t yaw_num;
    if((g_yaw_jy61>180)&&(g_yaw_jy61<270))  yaw_num = g_yaw_jy61-180;
    else if((g_yaw_jy61>90)&&(g_yaw_jy61<180))  yaw_num = g_yaw_jy61-180;
    else yaw_num=0;
    return yaw_num;
}
int detect_line(void)
{		
		if(HW1||HW2||HW3||HW4||HW5||HW6||HW7||HW8)return 0;
		else{
		car_stop();
		g_Stop_Flag=0;
		return 1;
		}
}



