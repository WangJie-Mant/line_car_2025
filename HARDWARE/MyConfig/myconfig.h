

/*
                            NVIC_config         
                             中断配置           
分组：HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);          
                
   Interrupt               PreemptPriority      SubPriority     Comment      
    (中断)                  (抢占优先级)       (响应优先级)     (说明)  
sysTick---------------------------0-----------------0---------    
TIM3_IRQHandler-------------------1-----------------0---------
TIM2_IRQHandler-------------------3-----------------0---------
TIM4_IRQHandler-------------------3-----------------0---------
USART1----------------------------2-----------------0---------
USART2----------------------------2-----------------0---------
USART3----------------------------2-----------------0---------




*/

/*
                          HARDWARE_config       
                             硬件连接       
Module          Pin             STM32_Pin          Peripheral          Comment        
(模块)         (引脚)          (单片机引脚)          (外设)            (说明)        
KEY0------------GND----------------PB0---------------GPIO_IN-----------下降沿触发
KEY1------------GND----------------PB1---------------GPIO_-------------下降沿触发

OLED------------SCL----------------PB8---------------GPIO_OUT----------软件iic
    ------------SDA----------------PB9---------------GPIO_OUT----------软件iic
      
TB6612----------AIN1---------------PB13--------------GPIO_OUT----------方向控制
     -----------AIN2---------------PB12--------------GPIO_OUT----------方向控制
     -----------PWMA---------------PA11--------------TIM1_CH4----------PWM
     -----------BIN1---------------PB14--------------GPIO_OUT----------方向控制
     -----------BIN2---------------PB15--------------GPIO_OUT----------方向控制
     -----------PWMB---------------PA8---------------TIM1_CH1----------PWM
     
MOTOR1----------AO1----------------------------------TB6612------------左电机
      ----------EncoderB-----------PA1---------------TIM2_CH2----------编码器模式
      ----------EncoderA-----------PA0---------------TIM2_CH1----------编码器模式
      ----------AO2----------------------------------TB6612------------左电机
      
MOTOR2----------BO1----------------------------------TB6612------------右电机
      ----------EncoderB-----------PB7---------------TIM4_CH1----------编码器模式
      ----------EncoderA-----------PB6---------------TIM4_CH2----------编码器模式
      ----------BO2----------------------------------TB6612------------右电机
     
USART1----------TX-----------------PA10--------------USART1_RX---------
       ---------RX-----------------PA9---------------USART1_TX---------

JY901S----------TX-----------------PA3---------------USART2_RX---------陀螺仪
       ---------RX-----------------PA2---------------USART2_TX---------陀螺仪

USART3----------TX-----------------PB11--------------USART3_RX---------
       ---------RX-----------------PB10--------------USART3_TX---------

MPU6050---------SCL----------------PB4---------------GPIO_OUT---------软件iic
       ---------SDA----------------PB3---------------GPIO_OUT---------软件iic
       
HW1-------------DO-----------------PA4---------------GPIO_IN---------红外对管1
HW2-------------DO-----------------PA5---------------GPIO_IN---------红外对管2
HW3-------------DO-----------------PA6---------------GPIO_IN---------红外对管3
HW4-------------DO-----------------PC15--------------GPIO_IN---------红外对管4

*/