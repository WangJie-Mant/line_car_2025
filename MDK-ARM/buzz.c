#include "buzz.h"
#include "tim.h"

void Buzz_Init(void)
{
    // 确保蜂鸣器初始状态为关闭（假设低电平关闭）
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
}

void Buzz(int duration)
{
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);
    while (duration > 0)
    {
        duration--;
    }
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
}
