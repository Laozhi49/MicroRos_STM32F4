#include "motor_control.h"

void Motor_initial()
{
    // 启动d电机pwm
    //HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);
}

void Motor_Forward()
{
    //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1450);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1550);
}

void Motor_Backward()
{
    //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1550);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1450);
}

void Motor_RightTurn()
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1550);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1550);
}

void Motor_LeftTurn()
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1450);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1450);
}

void Motor_Stop()
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);
}