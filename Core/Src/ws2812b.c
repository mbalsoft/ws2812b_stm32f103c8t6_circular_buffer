#include "ws2812b.h"

#define BIT_0_TIME		32
#define BIT_1_TIME		64

#define CIRCULAR_LEN    48


const uint8_t gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };


static uint8_t circular_buffer[ CIRCULAR_LEN ];
static uint8_t ws2812b_array[ 3 * LED_N ];
uint16_t led_counter = 0;


// ============================================================================================

void copy24bit_to_24byte( uint8_t *buf3byte, uint8_t *buf24byte ) {
	uint8_t value;
	for( uint8_t loop3byte = 0; loop3byte < 3; loop3byte++ ) {
		value = buf3byte[ loop3byte ];
		for( uint8_t loop24byte = 0; loop24byte < 8; loop24byte++ ) {
			if (value & 0x80) {
				buf24byte[ (loop3byte * 8) + loop24byte ] = BIT_1_TIME;
			}
			else {
				buf24byte[ (loop3byte * 8) + loop24byte ] = BIT_0_TIME;
			}
			value <<= 1;
		}
	}
}

void ws2812b_set_color( uint32_t led, uint8_t red, uint8_t green, uint8_t blue )
{
	if( led < LED_N )
	{
		ws2812b_array[ 3 * led + 0 ] = green;
		ws2812b_array[ 3 * led + 1 ] = red;
		ws2812b_array[ 3 * led + 2 ] = blue;
	}
}

void ws2812b_update(void)
{
	busy_indicator = 1;
	led_counter = 0;
	for( uint8_t loop = 0; loop < CIRCULAR_LEN; loop++ ) {
		circular_buffer[ loop ] = 0;
	}
	HAL_TIM_PWM_Start_DMA( &htim3, TIM_CHANNEL_1, (uint32_t*) circular_buffer, sizeof( circular_buffer ));
}

void ws2812b_dma_interupt(void) {
	if( led_counter > LED_N ) {
		HAL_TIM_PWM_Stop_DMA( &htim3, TIM_CHANNEL_1 );
		busy_indicator = 0;
	}
	else if( led_counter == LED_N ) {
		for( uint8_t loop = 0; loop < CIRCULAR_LEN; loop++ ) {
			circular_buffer[ loop ] = 90;
		}
	}
	else {
		if( (led_counter & 0x01) == 0 ) {
			copy24bit_to_24byte( ws2812b_array + (led_counter * 3), circular_buffer + 0 );
		}
		else {
			copy24bit_to_24byte( ws2812b_array + (led_counter * 3), circular_buffer + 24 );
		}
	}
	led_counter++;
}

void ws2812b_init(void)
{
  for( uint16_t loop = 0; loop < 3 * LED_N; loop++ )
	  ws2812b_array[ loop ] = 0;

  HAL_TIM_Base_Start( &htim3 );
  ws2812b_update();
}

void clear_led_data(void) {
	for( uint16_t loop = 0; loop < 3 * LED_N; loop++ )
		  ws2812b_array[ loop ] = 0;
}
