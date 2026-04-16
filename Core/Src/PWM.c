#include "main.h"
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
#define TIMER_CLK 120000000UL  
#define TIM_MAX    65535UL
void PWM_Set(float duty, float freq)
{
    // --- b?o v? input ---
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    if (freq <= 0.0f) return;

    uint32_t psc, arr, ccr;

    // --- t�nh PSC sao cho ARR <= 65535 ---
    psc = (uint32_t)(TIMER_CLK / (freq * (TIM_MAX + 1.0f)));

    if (psc > TIM_MAX) psc = TIM_MAX;

    // --- t�nh ARR ---
    arr = (uint32_t)(TIMER_CLK / (freq * (psc + 1))) - 1;

    if (arr > TIM_MAX) arr = TIM_MAX;

    // --- t�nh CCR t? duty ---
    ccr = (uint32_t)(duty * (arr + 1));

    // --- c?p nh?t timer (an to�n) ---
    __HAL_TIM_DISABLE(&htim4);

    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ccr);

    __HAL_TIM_ENABLE(&htim4);
}
void PWM_Set_Braking(float duty, float freq)
{
    // --- b?o v? input ---
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    if (freq <= 0.0f) return;

    uint32_t psc, arr, ccr;

    // --- t�nh PSC sao cho ARR <= 65535 ---
    psc = (uint32_t)(TIMER_CLK / (freq * (TIM_MAX + 1.0f)));

    if (psc > TIM_MAX) psc = TIM_MAX;

    // --- t�nh ARR ---
    arr = (uint32_t)(TIMER_CLK / (freq * (psc + 1))) - 1;

    if (arr > TIM_MAX) arr = TIM_MAX;

    // --- t�nh CCR t? duty ---
    ccr = (uint32_t)(duty * (arr + 1));

    // --- c?p nh?t timer (an to�n) ---
    __HAL_TIM_DISABLE(&htim8);

    __HAL_TIM_SET_PRESCALER(&htim8, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim8, arr);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, ccr);

    __HAL_TIM_ENABLE(&htim8);
}
void delay_us(uint16_t us)
{
    // --- t�nh s? chu k? c?n d?m ---
    uint32_t cycles = (TIMER_CLK / 1000000) * us;

    // --- reset timer ---
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // --- d?i d?n khi d? chu k? tr�i qua ---
    while (__HAL_TIM_GET_COUNTER(&htim4) < cycles);
}
