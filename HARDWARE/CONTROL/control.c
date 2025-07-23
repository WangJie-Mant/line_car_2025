#include "control.h"
#include "tim.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "protocol.h"
#include "jy61p.h"
#include "line.h"
#include <stdint.h>
/* 定义全局变量 */
long g_sigma_motor1pluse = 0;       //电机1累计脉冲
long g_sigma_motor2pluse = 0;       //电机2累计脉冲
short g_unittime_motor1pluse = 0;   //电机1单位时间脉冲
short g_unittime_motor2pluse = 0;   //电机2单位时间脉冲

float g_motor1_journey_cm = 0;    //电机1走过的路程
float g_motor2_journey_cm = 0;    //电机2走过的路程

float g_ftarget_journey = 0;  //存放小车左右轮走的路程,单位cm，需要在下一阶段任务中设置(car_go),用于判断车子到达目标位置

uint8_t g_stop_count = 0;   //停止计时，用于判断是否停车
uint8_t g_spin_count = 0;   //旋转计时，用于判断是否完成旋转

uint8_t g_location_control_count = 0;  //执行频率不需要那么高的用这个事件计数，用在中断中，控制串级pid内外环执行频率
uint8_t g_turn_angle_control_count = 0;//执行频率不需要那么高的用这个事件计数，用在中断中，控制串级pid内外环执行频率
uint8_t g_line_speed_control_count = 0;//执行频率不需要那么高的用这个事件计数，用在中断中，控制串级pid内外环执行频率

float g_speed1_outval = 0;      //电机1速度环输出值
float g_location1_outval = 0;   //电机1位置环输出值
float g_speed2_outval = 0;      //电机2速度环输出值
float g_location2_outval = 0;   //电机2位置环输出值
float g_turn_outval = 0;        //电机差速输出值，用于角度环和巡线环
float g_speed3_outval = 0;      //电机1转向速度环输出值
float g_speed4_outval = 0;      //电机2转向速度环输出值

int g_motor1_pwm = 0;     //电机1装载pwm值
int g_motor2_pwm = 0;     //电机2装载pwm值

uint8_t g_Line_Flag = 0;            //巡线标志位,0不巡线,1巡线
uint8_t g_Stop_Flag = 0;            //停止标志位,0行驶,1停止
uint8_t g_Spin_Start_Flag = 0;      //转向开始标志位
uint8_t g_Spin_Succeed_Flag = 0;    //转向结束标志位
uint8_t g_Turn_Flag = 0;            //转向标志位,好像没用到
uint8_t g_Angle_Flag = 0;           //角度环调试标志位

int32_t g_line_num = 0;//灰度巡线偏移量
float g_line_outval = 0;//巡线差值量
int32_t g_yaw_err = 0;//角度偏移量

extern uint8_t g_mode;//运行哪个功能

float spin90_cm=0; 

/* 定时器回调函数,pid控制在这里进行,周期20ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)//20ms定时器中断进行PID计算
    {
        /* ***** */
        /* 读取脉冲 */
        g_unittime_motor1pluse = read_pluse(&htim2); //根据实际情况选择正负,保证两个电机正方向相同
        g_unittime_motor2pluse = -read_pluse(&htim4);//根据实际情况选择正负,保证两个电机正方向相同
        /* 更新累计脉冲 */
        g_sigma_motor1pluse += g_unittime_motor1pluse; //位置外环使用的脉冲累积  //完成一次位置控制之后才清除。
        g_sigma_motor2pluse += g_unittime_motor2pluse; //位置外环使用的脉冲累积  //记录了之后也要及时清零呀！！！
				
        //有了巡线环之后，小车直线距离可能需要补偿，根据寻直线的效果来定。  3.14即可补偿？巡线效果不好（不直）的话可以继续减小这个pai的值
        g_motor1_journey_cm = (g_sigma_motor1pluse / (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION)) * (WHEEL_D*3.1416);
        g_motor2_journey_cm = (g_sigma_motor2pluse / (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION)) * (WHEEL_D*3.1416);
        
        /* ***** */
        
//        /*****上位机调试速度环PID时用,实际运行时注释掉*****/
//          /* pid控制 */
//        if(g_is_motor1_en == 1)
//        {
//            g_motor1_pwm = speed1_pid_control();
//        }
//        if(g_is_motor2_en == 1) 
//        {
//            g_motor2_pwm = speed2_pid_control();
//        }
//        /* 限幅 */
//        limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
//        /* 装载 */
//        load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
    
          /******/
          
//        if(g_mode == 2 || g_mode == 3)
//        {
//            /******上位机调试位置速度串级PID时用，实际使用时注释掉******/
//            if(g_is_motor1_en == 1 && g_is_motor2_en == 1)
//            {
//                /* pid控制 */
//                location_speed_control();
//                g_motor1_pwm = g_speed1_outval;  
//                g_motor2_pwm = g_speed2_outval;
//                
//                /* 限幅 */
//                limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
//                /* 装载 */
//                load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
//            }
//        }
//        
//        /* ***** */
//        
//        if(g_mode == 4)
//        {
//            /****** 上位机调试角度速度串级PID时用，实际使用时注释掉 ******/
//            if(g_Angle_Flag == 1)
//            {
//                if(g_is_motor1_en == 1 && g_is_motor2_en == 1)
//                {
//                    /* pid控制 */
//                    turn_angle_speed_control();
//                    g_motor1_pwm = g_speed3_outval;  
//                    g_motor2_pwm = g_speed4_outval;
//                    
//                    /* 限幅 */
//                    limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
//                    /* 装载 */
//                    load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
//                }
//            }
//        }
//        
//        /* ***** */
//        
//        if(g_mode == 1)
//        {
//            /****** 上位机调试巡线速度串级PID时用，实际使用时注释掉 ******/
//            if(g_Line_Flag == 1)
//            {
//                if(g_is_motor1_en == 1 && g_is_motor2_en == 1)
//                {
//                    /* pid控制 */
//                    line_speed_control();
//                    
//                    g_motor1_pwm = g_speed3_outval;  
//                    g_motor2_pwm = g_speed4_outval;
//                    /* 限幅 */
//                    limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
//                    /* 装载 */
//                    load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
//                }
//            }
//        }
        /* ***** */
    		
		       				
			 if(g_Line_Flag == 1) 
        {
            //来个判断，直行到达位置后停止PID控制，防止识别图片时或者等待装卸药时电机耗能，直走巡线结束的阶段才用这个， 
            //其实这个要是用好一点的驱动模块的话，短时间内运行也不需要.....不用怕电机驱动过热...一直PID控制也没啥
            //    调试位置速度串级PID时注释掉这个让电机失能的函数
            if(g_motor1_journey_cm == g_ftarget_journey)  //这里偏差这么多真的好吗？并不是真的偏差
					
						{
                    g_Line_Flag = 0;
                    g_Stop_Flag = 1; //这个标志位可以用来判断是否执行下一阶段任务
                    g_stop_count = 0;
        
                    set_motor1_disable();
                    set_motor2_disable();
            }
            else
            {
                g_Stop_Flag = 0;  
                g_stop_count = 0;  
            }
            if(g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
            {
                location_speed_control();
                g_line_num = line_pid_control();               //位置环速度环串级PID的输出是速度环输出的PWM值
                if(g_line_num == 0)  //每次回到线上需要补偿的时候，都将两个电机的累计脉冲数取平均值，这个也会有在转向后帮助回到线上的效果
                {
                    long pulse;
                    pulse = (g_sigma_motor1pluse + g_sigma_motor2pluse) / 2;
                    g_sigma_motor1pluse = pulse;  //可能有时候这里加上个补偿会更好
                    g_sigma_motor2pluse = pulse;
                }
                //这个是灰度传感器的巡线补偿
                g_line_outval =  g_line_num;     //g_line_num得在PWM的重装载值一半左右才会有明显的效果
                g_motor1_pwm = g_speed1_outval + g_line_outval;  
                g_motor2_pwm = g_speed2_outval - g_line_outval;
                
                limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
                load_motor_pwm(g_motor1_pwm,g_motor2_pwm); 
            }
        }

        /* ****** */
        
				/*转弯*/
				  if( g_Spin_Start_Flag == 1)  
        {
            if (g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
            {
                location_speed_control(); //位置环速度环串级PID的输出是速度环输出的PWM值
                g_motor1_pwm = g_speed1_outval;
                g_motor2_pwm = g_speed2_outval;
                limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
                load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
							if((spin90_cm-0.1<g_motor1_journey_cm)||(g_motor1_journey_cm>spin90_cm))
							{
										set_motor1_disable();
                    set_motor2_disable();
										g_Spin_Start_Flag=0;
							}
            }
        }
        /* ***** */
        
				
				 if( g_Angle_Flag == 1)  
        {
            set_motor1_enable();   //使能电机控制PWM输出    
            set_motor2_enable();   //使能电机2控制PWM输出
            if (g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
            {
                turn_angle_speed_control(); //位置环速度环串级PID的输出是速度环输出的PWM值
                g_motor1_pwm = g_speed3_outval;
                g_motor2_pwm = g_speed4_outval;
                limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);
                load_motor_pwm(g_motor1_pwm,g_motor2_pwm);
                int count=0;
                if(g_pid_turn_angle.target_val+2>g_yaw_jy61&&g_yaw_jy61>g_pid_turn_angle.target_val-2) //这里的角度误差可以调节
                {  
                    count++;
                    if(count>25){
                    set_motor1_disable();
                    set_motor2_disable();
                    g_Angle_Flag=0; 
                    } //转向完成后将标志位清零
                }
            }
        }
        /* ***** */
    }
}
   
					



/********小车控制函数********/
/********停止函数**********/
void car_stop(void)
{
 uint8_t g_Line_Flag = 0;            //巡线标志位,0不巡线,1巡线
uint8_t g_Stop_Flag = 0;            //停止标志位,0行驶,1停止
uint8_t g_Spin_Start_Flag = 0;      //转向开始标志位
uint8_t g_Spin_Succeed_Flag = 0;    //转向结束标志位
uint8_t g_Turn_Flag = 0;            //转向标志位,好像没用到
uint8_t g_Angle_Flag = 0;           //角度环调试标志位
    set_motor1_disable();
    set_motor2_disable();
}

/**
 * @brief       小车直行一定距离函数
 * @param       要走的路程
 * @retval      无
 */
/****走完后自动将 g_Stop_Flag置1****/
/*这里加将位置环的输入直接编程距离(cm)的函数 */   //所有普通直行都用这个
void car_go(int32_t distance_cm)   //直走函数     //连续两次的直行好像会让巡线补偿失效
{
    float target_pluse;//目标距离对应的脉冲值
    
    /* 设置对应标志位 */
    g_Line_Flag = 1;   
    g_Stop_Flag = 0;
    
    g_Spin_Start_Flag = 0;
    g_Spin_Succeed_Flag = 0;  
    
//  if(location_cm < 0)
//  {
//      LineNumToggleFlag = 1;
//  }
//  else LineNumToggleFlag = 0;
    
    /* 清除上次走的路程记录 */
    g_motor1_journey_cm = 0;    //之前没清除所以用不了
    g_motor2_journey_cm = 0;
    g_sigma_motor1pluse = 0;
    g_sigma_motor2pluse = 0; 
    
    g_ftarget_journey = distance_cm;   //防止长时间PID控制用
    
    /* 将目标距离转换为对应的脉冲值，设为位置环期望 */
    //如果巡线效果不好就将3.142加大
    target_pluse = (distance_cm / (WHEEL_D * 3.1416) ) * (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION);  //将distance_cm 转换为对应的脉冲数脉冲数 
    //小车位置直接用一个电机的脉冲数累积就好，转向时不计数，开始一个位置前直接清零。   
    //打滑导致一个轮比另一个轮转位置不一样咋办,用“巡线环“弥补就好，转向就用“转向环”
    /* 设为位置环期望 */
    set_pid_target(&g_pid_location1,target_pluse);   
    set_pid_target(&g_pid_location2,target_pluse);
    
    /* 使能电机 */
    set_motor1_enable();   //使能电机控制PWM输出    
    set_motor2_enable();   //使能电机2控制PWM输出
}


/**
 * @brief       小车旋转控制函数
 * @param       left_90左转90度,right_90右转90度,back_180掉头
 * @retval      无
 */
/******转向完成时自动将  g_Spin_Start_Flag == 0 、 g_Spin_Succeed_Flag  ********/
/*转角有左转90，右转90，和转180三种情况。*/
void car_spin(spin_dir direction) //原地转向可以直接 调用这个  //其实转向环并不需要很精准，转弯后直接用直走时的巡线函数回正车身就好
{
         //自旋90度的距离
    float car_turn_pluse;  //小车旋转对应的脉冲数
    
    /* 设置对应标志位 */
    g_Line_Flag = 0;  //不进行巡线的补偿了
    g_Stop_Flag = 0;   //执行转弯时，将直走完成的标志位清零. 即如果上一次是直行，这次是转弯，则不用在业务代码里手动置位
    
    g_Spin_Start_Flag = 1;   
    g_Spin_Succeed_Flag = 0; 
    
    /* 清除上次路程 */
    g_motor1_journey_cm = 0;
    g_motor2_journey_cm = 0;
    g_sigma_motor1pluse = 0;   
    g_sigma_motor2pluse = 0;  //之前没清除所以用不了
    
    /* 自旋90度的距离 */
    spin90_cm = 0.25*3.1416*LUN_JU; //以车中心为圆心，小车两轮距离为直径，小车旋转90度即为圆的1/4
    
    /****转弯后不循线可以调转向系数**********/
    if(direction == left_90)  //需要往左边走
    {
        car_turn_pluse = (spin90_cm / (WHEEL_D*3.142)) * (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION); 
        car_turn_pluse = car_turn_pluse;   //90*0.94 = 84.6   //惯性影响，导致转弯比理论设定的多。直接设90度接下来的巡线就寻不回来了
    }
    else if(direction == right_90)  //需要往右边走
    {
        car_turn_pluse = -(spin90_cm / (WHEEL_D*3.142)) * (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION); 
        car_turn_pluse = car_turn_pluse;  //90*0.96 = 86.4    //惯性影响，导致转弯比理论设定的多。接下来的巡线可能就寻不回来了
    }
    else if(direction == back_180)
    {
        car_turn_pluse = -(spin90_cm / (WHEEL_D*3.142)) * (REDUCTION_RATIO*ENCODER_TOTAL_RESOLUTION)*2;
       
    }
    
    set_pid_target(&g_pid_location1, -car_turn_pluse);   
    set_pid_target(&g_pid_location2, car_turn_pluse);
    
    set_motor1_enable();   //使能电机控制PWM输出    
    set_motor2_enable();   //使能电机2控制PWM输出
}




/*********************各PID**********************/

/**
 * @brief        速度环位置环串级PID控制
 * @param       无
 * @retval      无
 */
void location_speed_control(void)          
{
    if (g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
    {
        /* 位置环控制 */
        g_location_control_count++;
        if(g_location_control_count >= 2)  //内环执行频率大于外环
        { 
            g_location_control_count = 0; 
            g_location1_outval = location1_pid_control();
            g_location2_outval = location2_pid_control();
        }

        /* 位置环输出作为速度环输入 */
        set_pid_target(&g_pid_speed1,g_location1_outval); //每次都必须有位置环的值     
        set_pid_target(&g_pid_speed2,g_location2_outval); //每次都必须有位置环的值       
        
        /* 速度环控制 */
        g_speed1_outval = speed1_pid_control();    //要是电机转向不符合预期，就在这两句里取反数值
        g_speed2_outval = speed2_pid_control();  
    }
}


/**
 * @brief       电机1位置环控制
 * @param       无
 * @retval      位置环的输出值
 */
float location1_pid_control(void)  
{
    float cont_val = 0.0; 
    int32_t actual_location;
    
    actual_location = g_sigma_motor1pluse;   //1圈 = 1768个脉冲 = 34*13*4  //这里位置用圈数代替。

    cont_val = location_pid_realize(&g_pid_location1,actual_location);   
    
    //还没加串级PID之前，位置环的cont_val对应PWM。 改成串级PID后，位置换的cont_val对应目标速度
       
    /* 目标速度上限处理 */
    if (cont_val > TARGET_SPEED_MAX)
    {
        cont_val = TARGET_SPEED_MAX;
    }
    else if (cont_val < -TARGET_SPEED_MAX)
    {
        cont_val = -TARGET_SPEED_MAX;
    }
    
    /* 上位机调试代码 */
//    #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location, 1);    // 给通道 1 发送实际值
//  #endif

    return cont_val;
}

/**
 * @brief       电机1速度环控制
 * @param       无
 * @retval      速度环输出值
 */
float speed1_pid_control(void)  
{
    float cont_val = 0.0;           //当前控制值
    int32_t actual_speed;           //车轮实际转速度,单位rpm(转/min)
    
    /* 计算公式 */
    /* 
    编码器倍频后的总分辨率 = 编码器线数*倍频数------倍频数由编码器计数方式决定
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比)-----一个控制周期转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)-----每ms转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000-----每s转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000*60-----每min转的圈数
    */
    /* 转速=单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000*60 */
    actual_speed = ((float)g_unittime_motor1pluse*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);

    cont_val = speed_pid_realize(&g_pid_speed1,actual_speed);    // 进行 PID 计算
  
    /* 上位机调试代码 */
//  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_speed, 1);                // 给通道 1 发送实际值
//  #endif
    
    return cont_val;
}

/**
 * @brief       电机2位置环控制
 * @param       无
 * @retval      位置环的输出值
 */
float location2_pid_control(void)  
{
    float cont_val = 0.0; 
    int32_t actual_location;
    
    actual_location = g_sigma_motor2pluse;   //1圈 = 1768个脉冲 = 34*13*4  //这里位置用圈数代替。

    cont_val = location_pid_realize(&g_pid_location2,actual_location);   
    
    //还没加串级PID之前，位置环的cont_val对应PWM。 改成串级PID后，位置换的cont_val对应目标速度
       
    /* 目标速度上限处理 */
    if (cont_val > TARGET_SPEED_MAX)
    {
        cont_val = TARGET_SPEED_MAX;
    }
    else if (cont_val < -TARGET_SPEED_MAX)
    {
        cont_val = -TARGET_SPEED_MAX;
    }
    
    /* 上位机调试代码 */
//    #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location, 1);                // 给通道 1 发送实际值
//  #endif

    return cont_val;
}

/**
 * @brief       电机2速度环控制
 * @param       无
 * @retval      速度环输出值
 */
float speed2_pid_control(void)  
{
    float cont_val = 0.0;           //当前控制值
    int32_t actual_speed;           //车轮实际转速度,单位rpm(转/min)
    
    /* 计算公式 */
    /* 
    编码器倍频后的总分辨率 = 编码器线数*倍频数------倍频数由编码器计数方式决定
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比)-----一个控制周期转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)-----每ms转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000-----每s转的圈数
    单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000*60-----每min转的圈数
    */
    /* 转速=单位时间内脉冲/(编码器倍频后的总分辨率*电机减速比*PID控制周期)*1000*60 */
    actual_speed = ((float)g_unittime_motor2pluse*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);

    cont_val = speed_pid_realize(&g_pid_speed2,actual_speed);    // 进行 PID 计算
  
    /* 上位机调试代码 */
//  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_speed, 1);                // 给通道 1 发送实际值
//  #endif
    
    return cont_val;
}

/**
 * @brief       角度速度串级PID控制，角度外环的输出给到速度内环
 * @param       无
 * @retval      无
 */
void turn_angle_speed_control(void)
{
    if (g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
    {
        g_turn_angle_control_count++;
        if(g_turn_angle_control_count >= 2)     //内环执行频率大于外环
        {
            g_turn_angle_control_count = 0; 
            /* 角度外环输出 输出一个值就可以给速度内环一正一负就可以实现转向 */
            g_turn_outval = turn_angle_pid_control();
        }
        /* 角度外环输出给到速度内环输入 */  
        set_pid_target(&g_pid_speed1,-g_turn_outval); //如果极性错误将左右轮正负号取反
        set_pid_target(&g_pid_speed2,g_turn_outval); 
        g_speed3_outval = speed1_pid_control();   
        g_speed4_outval = speed2_pid_control();  
    }

}


/**
 * @brief       角度环控制
 * @param       无
 * @retval      角度环的输出值
 */
float turn_angle_pid_control(void)
{
    float cont_val = 0.0; 
    int32_t actual_angle;
    
    actual_angle = g_yaw_jy61;  

    cont_val = turn_angle_pid_realize(&g_pid_turn_angle,actual_angle);   
    

    /* 目标速度上限处理 */
    if (cont_val > TARGET_SPEED_MAX)
    {
        cont_val = TARGET_SPEED_MAX;
    }
    else if (cont_val < -TARGET_SPEED_MAX)
    {
        cont_val = -TARGET_SPEED_MAX;
    }

//    #if defined(PID_ASSISTANT_EN)
//        set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_angle, 1);    // 给通道 1 发送实际值
//    
//    #endif
    return cont_val;
}

/**
 * @brief       巡线速度串级PID实现，通过巡线实现差速转弯
 * @param       无
 * @retval      无
 */
void line_speed_control(void)
{
    if (g_is_motor1_en == 1 || g_is_motor2_en == 1)     // 电机在使能状态下才进行控制处理
    {
        g_line_speed_control_count++;         //这个是巡线串级pid外环
        if(g_line_speed_control_count >= 2)
        {
            g_line_speed_control_count = 0; 
            g_turn_outval = line_pid_control();
        }
        
        
        //这个的100是基础的速度加减g_turn_outval实现差速
        set_pid_target(&g_pid_speed1, 100-g_turn_outval); 
        set_pid_target(&g_pid_speed2, 100+g_turn_outval); 
        g_speed3_outval = speed1_pid_control();   
        g_speed4_outval = speed2_pid_control();  
    }
}

/**
 * @brief       巡线环控制
 * @param       无
 * @retval      巡线环的输出值，实际上是两个轮子的差速值
 */
float line_pid_control(void)  
{
   
    float cont_val = 0.0;                       // 当前控制值
    int32_t actual_speed;    
    
    // 使用灰度传感器巡线偏差作为PID输入，提升拐弯能力
    int32_t err = line_err();
    cont_val = line_pid_realize(err * 2); // 放大补偿量，乘以2（可根据实际调试调整倍数）
    return cont_val;
}







