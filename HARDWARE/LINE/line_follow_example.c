/*
 * 循迹PID控制使用示例
 * 文件名：line_follow_example.c
 *
 * 此文件展示了如何使用新添加的循迹PID控制功能
 */

#include "line.h"
#include "control.h"
#include "motor.h"
#include "pid.h"

/* 全局变量 */
extern uint8_t g_Line_Flag; // 巡线标志位

/**
 * @brief       循迹系统初始化
 * @param       无
 * @retval      无
 * @note        在main函数中调用一次即可
 */
void line_follow_system_init(void)
{
    /* 1. 初始化PID参数 */
    pid_param_init();

    /* 2. 初始化循迹PID参数 */
    line_follow_pid_init();

    /* 3. 设置基础速度（可根据需要调整） */
    set_line_follow_base_speed(80.0f); // 设置基础速度为80

    /* 4. 设置PID参数（需要根据实际调试） */
    set_line_follow_pid_params(2.5f, 0.15f, 0.8f); // Kp=2.5, Ki=0.15, Kd=0.8
}

/**
 * @brief       开始循迹控制
 * @param       无
 * @retval      无
 */
void start_line_following(void)
{
    /* 启动循迹标志位 */
    g_Line_Flag = 1;

    /* 使能电机 */
    set_motor1_enable();
    set_motor2_enable();
}

/**
 * @brief       停止循迹控制
 * @param       无
 * @retval      无
 */
void stop_line_following(void)
{
    /* 停止循迹标志位 */
    g_Line_Flag = 0;

    /* 停止电机 */
    car_stop();
}

/**
 * @brief       循迹控制任务（在定时器中断中调用）
 * @param       无
 * @retval      无
 * @note        建议在20ms定时器中断中调用此函数
 */
void line_follow_task(void)
{
    if (g_Line_Flag == 1) // 只有在循迹标志位为1时才执行
    {
        /* 执行循迹PID控制 */
        line_follow_control();
    }
}

/* ================================== */
/*        在定时器中断中的集成示例      */
/* ================================== */

/**
 * @brief       定时器中断回调函数示例
 * @param       htim: 定时器句柄
 * @retval      无
 * @note        这是修改后的定时器中断函数，集成了新的循迹控制
 */
void HAL_TIM_PeriodElapsedCallback_Example(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3) // 20ms定时器中断进行PID计算
    {
        /* 读取编码器脉冲 */
        g_unittime_motor1pluse = read_pluse(&htim2);
        g_unittime_motor2pluse = read_pluse(&htim4);

        /* 更新累计脉冲 */
        g_sigma_motor1pluse += g_unittime_motor1pluse;
        g_sigma_motor2pluse += g_unittime_motor2pluse;

        /* 计算行驶距离 */
        g_motor1_journey_cm = (g_sigma_motor1pluse / (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION)) * (WHEEL_D * 3.1416);
        g_motor2_journey_cm = (g_sigma_motor2pluse / (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION)) * (WHEEL_D * 3.1416);

        /* ===== 新的循迹PID控制 ===== */
        if (g_Line_Flag == 1)
        {
            /* 执行循迹PID控制任务 */
            line_follow_task();

            /* 获取速度环输出并装载到电机 */
            if (g_is_motor1_en == 1 && g_is_motor2_en == 1)
            {
                /* 执行速度环PID控制 */
                g_speed1_outval = speed1_pid_control();
                g_speed2_outval = speed2_pid_control();

                /* 限幅并装载PWM */
                g_motor1_pwm = (int)g_speed1_outval;
                g_motor2_pwm = (int)g_speed2_outval;
                limit_motor_pwm(&g_motor1_pwm, &g_motor2_pwm);
                load_motor_pwm(g_motor1_pwm, g_motor2_pwm);
            }
        }

        /* 其他控制任务... */
        /* 这里可以保留原有的其他控制逻辑 */
    }
}

/* ================================== */
/*          主程序使用示例            */
/* ================================== */

/**
 * @brief       主程序示例
 * @param       无
 * @retval      无
 */
void main_example(void)
{
    /* 系统初始化 */
    // HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();
    // MX_TIM2_Init();
    // MX_TIM3_Init();
    // MX_TIM4_Init();

    /* 循迹系统初始化 */
    line_follow_system_init();

    /* 启动定时器 */
    // HAL_TIM_Base_Start_IT(&htim3);  // 启动20ms定时器中断
    // HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 启动编码器1
    // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 启动编码器2

    /* 开始循迹 */
    start_line_following();

    while (1)
    {
        /* 主循环中可以处理其他任务 */

        /* 示例：根据某个条件停止循迹 */
        // if(某个停止条件)
        // {
        //     stop_line_following();
        // }

        /* 示例：动态调整PID参数 */
        // if(需要调整PID参数)
        // {
        //     set_line_follow_pid_params(new_kp, new_ki, new_kd);
        // }

        /* 示例：动态调整基础速度 */
        // if(需要调整速度)
        // {
        //     set_line_follow_base_speed(new_speed);
        // }

        HAL_Delay(10);
    }
}
