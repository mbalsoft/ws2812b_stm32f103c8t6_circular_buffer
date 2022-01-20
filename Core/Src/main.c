/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */

#include "ws2812b.h"

char input_usb_buffer[ USB_BUFFER_LENGTH ] = "";
uint16_t in_usb_buf_pos = 0;
char *output_usb_buffer[ USB_INPUT_QUEUE_LEN ];

char prompt[] = "\r\n> ";
char info[]   = "WS2812B driver.\r\n";
char help[]   = " led <LED number> <color> [ENTER]\r\n \
 - LED number from 1 to N\r\n \
 - color #RRGGBB using hex notation\r\n \
example: led 3 #FF0000\r\n \
 - for red light led number 3 with maximum volume\r\n \
example: led 1 off\r\n \
 - for turn off led 1\r\n\r\n";

uint8_t  effect_on_off = 0;
uint8_t  welcome_cnt = 0;
uint16_t counter = 0;
uint16_t led_loop = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void usb_transmit_fs( uint8_t *txBuf, uint32_t buf_len );
void welcome(void);
void get_command(void);
void send_queue_via_usb(void);
void write_to_future_send_via_usb( char *text_to_send );
void send_prompt(void);
void print_to_usb( uint8_t x );
void print_str_to_usb( char *str );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  for( uint8_t loop = 0; loop < USB_INPUT_QUEUE_LEN; loop++ ) {
	  output_usb_buffer[ loop ] = NULL;
  }

  ws2812b_init();

  srand( (unsigned) time( NULL ));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint16_t volume = 128; // max 256
  uint8_t r = gamma8[ rand() % volume ];
  uint8_t g = gamma8[ rand() % volume ];
  uint8_t b = gamma8[ rand() % volume ];

  while (1)
  {

	send_queue_via_usb();
	HAL_Delay( 1 );

	if( effect_on_off ) counter++;
	if( counter >= 100 && effect_on_off ) {
		counter = 0;
		if( led_loop >= LED_N ) {
			led_loop = 0;
			r = gamma8[ rand() % volume ];
			g = gamma8[ rand() % volume ];
			b = gamma8[ rand() % volume ];
		}
		ws2812b_set_color( led_loop, r, g, b );
		ws2812b_update();
		led_loop++;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 89;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void usb_transmit_fs( uint8_t *txBuf, uint32_t buf_len ) {
	wait_for_CDC_transmit_ready();
	CDC_Transmit_FS( txBuf, buf_len );
//	while( CDC_Transmit_FS( txBuf, buf_len ) == USBD_BUSY ) { // USBD_OK
//		HAL_Delay( 1 );
//	}
}

void welcome(void) {
	if( welcome_cnt & 0x01 == 1 ) {
		welcome_cnt = 0;
		return;
	}
	welcome_cnt++;
	for( uint8_t loop = 0; loop < USB_INPUT_QUEUE_LEN; loop++ ) {
		if( output_usb_buffer[ loop ] != NULL ) return;
	}
	char *tmp_buf;
	tmp_buf = malloc( (strlen( info ) + strlen( prompt )) * sizeof( char ));
	strcpy( tmp_buf, info );
	strcat( tmp_buf, prompt );
	write_to_future_send_via_usb( tmp_buf );
}

void get_command(void) {
	char *out_str;
	char *result[ 5 ];
	input_usb_buffer[ in_usb_buf_pos ] = 0;
	strcat( input_usb_buffer, " " );
	char *token = strtok( input_usb_buffer, " " );
    uint8_t loop = 0;
	if( token != NULL ) {
		while( token != NULL && loop < 5 ) {
			result[ loop ] = malloc( strlen( token ) * sizeof( char ));
			strcpy( result[ loop ], token );
			loop++;
		    token = strtok( NULL, " " );
		}
	}
	else {
		result[ 0 ] = malloc( strlen( input_usb_buffer ) * sizeof( char ));
		strcpy( result[ 0 ], input_usb_buffer );
		loop++;
	}

    if( strcmp( result[ 0 ], "help" ) == 0 || strcmp( result[ 0 ], "?" ) == 0 ) {
    	out_str = malloc( strlen( help ) * sizeof( char ));
    	strcpy( out_str, help );
    	write_to_future_send_via_usb( out_str );
    }
    else if( strcmp( result[ 0 ], "on" ) == 0 ) {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
    }
    else if( strcmp( result[ 0 ], "off" ) == 0 ) {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET );
    }
    else if( strcmp( result[ 0 ], "toggle" ) == 0 ) {
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13 );
    }
    else if( strcmp( result[ 0 ], "led" ) == 0 ) {
    	if( loop >= 3 ) {
    		uint16_t led_no = atoi( result[ 1 ]) - 1;
    		uint32_t r = 0;
    		uint32_t g = 0;
    		uint32_t b = 0;
    		if( strcmp( result[ 2 ], "off" ) == 0 ) {
    			ws2812b_set_color( led_no, r, g, b );
				ws2812b_update();
				out_str = malloc( 50 * sizeof( char ));
//				spritnf( out_str, "LED %d\nR = %d\nG = %d\nB = %d\n", led_no, r, g, b );
				strcpy( out_str, "OK " );
				write_to_future_send_via_usb( out_str );
    		}
    		else if( result[ 2 ][ 0 ] == '#' && strlen( result[ 2 ]) >= 7 ) {
        		char r_str[] = "0x00";
    			r_str[ 2 ] = result[ 2 ][ 1 ];
    			r_str[ 3 ] = result[ 2 ][ 2 ];
    			sscanf( r_str, "%x", &r );
        		char g_str[] = "0x00";
    			g_str[ 2 ] = result[ 2 ][ 3 ];
    			g_str[ 3 ] = result[ 2 ][ 4 ];
    			sscanf( g_str, "%x", &g );
        		char b_str[] = "0x00";
    			b_str[ 2 ] = result[ 2 ][ 5 ];
    			b_str[ 3 ] = result[ 2 ][ 6 ];
    			sscanf( b_str, "%x", &b );
    			ws2812b_set_color( led_no, r, g, b );
			    ws2812b_update();
			    out_str = malloc( 50 * sizeof( char ));
//				spritnf( out_str, "LED %d\nR = %d\nG = %d\nB = %d\n", led_no, r, g, b );
				strcpy( out_str, "OK" );
				write_to_future_send_via_usb( out_str );
    		}
    		else if( loop >= 5 ) {
    			r = atoi( result[ 2 ]);
    			g = atoi( result[ 3 ]);
    			b = atoi( result[ 4 ]);
        		ws2812b_set_color( led_no, r, g, b );
    			ws2812b_update();
    			out_str = malloc( 50 * sizeof( char ));
//				spritnf( out_str, "LED %d\nR = %d\nG = %d\nB = %d\n", led_no, r, g, b );
				strcpy( out_str, "OK" );
				write_to_future_send_via_usb( out_str );
    		}
    	}
    }
    else if( strcmp( result[ 0 ], "effect" ) == 0 ) {
    	effect_on_off = ! effect_on_off;
    	if( ! effect_on_off ) {
    		counter  = 0;
    		led_loop = 0;
    	}
    }
    else if( strcmp( result[ 0 ], "turn" ) == 0 && strcmp( result[ 1 ], "off" ) == 0 ) {
    	clear_led_data();
    	ws2812b_update();
    	out_str = malloc( 50 * sizeof( char ));
		strcpy( out_str, "all LED's turn OFF" );
		write_to_future_send_via_usb( out_str );
    }

    while( loop > 0 ) {
    	loop--;
    	free( result[ loop ]);
    }
	in_usb_buf_pos = 0;
	send_prompt();
}

void send_queue_via_usb(void) {
	for( uint8_t loop = 0; loop < USB_INPUT_QUEUE_LEN; loop++ ) {
		if( output_usb_buffer[ loop ] != NULL && strlen( output_usb_buffer[ loop ]) > 0 ) {
			usb_transmit_fs( output_usb_buffer[ loop ], strlen( output_usb_buffer[ loop ]));
			free( output_usb_buffer[ loop ] );
			output_usb_buffer[ loop ] = NULL;
		}
	}
}

void write_to_future_send_via_usb( char *text_to_send ) {
	uint8_t loop = 0;

	while( loop < USB_INPUT_QUEUE_LEN && output_usb_buffer[ loop ] != NULL ) {
		loop++;
	}
	if( loop < USB_INPUT_QUEUE_LEN ) {
		output_usb_buffer[ loop ] = text_to_send;
	}
}

void send_prompt(void) {
	char *tmp_buf;
	tmp_buf = malloc( strlen( prompt ) * sizeof( char ));
	strcpy( tmp_buf, prompt );
	write_to_future_send_via_usb( tmp_buf );
}


void print_to_usb( uint8_t x ) {
	char *tmp_buf;
	tmp_buf = malloc( strlen( prompt ) * sizeof( char ));
	itoa( x, tmp_buf, 10 );
	strcat( tmp_buf, "\r\n" );
	write_to_future_send_via_usb( tmp_buf );
}

void print_str_to_usb( char *str ) {
	char *tmp_buf;
	tmp_buf = malloc( strlen( prompt ) * sizeof( char ));
	strcpy( tmp_buf, str );
	strcat( tmp_buf, "\r\n" );
	write_to_future_send_via_usb( tmp_buf );
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
