#include "buzz.h"
#include "tim.h"

void Buzz(int duration)
{
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);
    while (duration > 0)
    {
        duration--;
    }
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
}
