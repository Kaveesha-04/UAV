#include "motors.h"
#include "main.h" // Needed for htim3 access

extern TIM_HandleTypeDef htim3;

// Motor control function implementation
void Set_Motor_PWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, m4);
}
