#pragma once

#include "stm32f1xx_hal.h"

void ws2812b_init(void);

void ws2812b_set_color(uint32_t led, uint8_t red, uint8_t green, uint8_t blue);

void ws2812b_update(void);

void ws2812b_wait(void);


#define LED_N			8


TIM_HandleTypeDef htim3;

const uint8_t gamma8[];
