/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "tm_stm32_hd44780.h"
#include "menu.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TRUE 1
#define FALSE 0
#define RETRY_COUNT 100

short int last_option;
volatile int round_time_ms;
static short int toggled_menu = MAIN_MENU;
static short int round_finished = FALSE;

/*USB Communication related*/
uint8_t data_to_send[USB_COMM_BUF_SIZE];
uint8_t message_length = 0;

uint8_t received_data_flag = FALSE;
uint8_t received_data[40] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*
 * Callback for handling contactron and menu button interrupt vectors
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MENU_BUTTON_Pin) {
		if (toggled_menu < MENU_SIZE - 1) {
			toggled_menu++;
		} else {
			toggled_menu = MAIN_MENU;
		}
	} else if (GPIO_Pin == CONTACTRON_Pin) {
		round_finished = TRUE;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10){
		round_time_ms++;
	}
}
void my_delay_ms(int value){
	round_time_ms = 0;
	HAL_TIM_Base_Start_IT(&htim10);
	while(round_time_ms <value)
	{
//		goto lab1;
	}
	return;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern char * eval_velocity();
int draw_state_lcd(menu_state *ms);
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	 menu_state menu[] = {
			 {.patterns = {{0,FIRST_ROW,"V:"},{7,FIRST_ROW,"km/h"}, {12,FIRST_ROW,"A:"},{15,FIRST_ROW,"G"},{0,SECOND_ROW,"T:"}, {4, SECOND_ROW,"h"}, {7, SECOND_ROW,"m"}, {10, SECOND_ROW,"s"}}, .state = MAIN_MENU},
			 {.patterns = {{1,SECOND_ROW,"SPEED:"}, {12,SECOND_ROW, "km/h"}, {0,FIRST_ROW,"<AVG> ACCEL:"},{15,FIRST_ROW,"G"}}, .state = STAT_MENU},
			 {.patterns = {{2,FIRST_ROW,"<TOTAL> DIST:"}, {14,SECOND_ROW,"km"}}, .state = STAT_MENU2},
			 {.patterns = {{0,FIRST_ROW,"<USB CONF MODE>"}}, .state = USB_CONF_MENU, .substate = USB_SUBSTATE_INIT}
			 };
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

	 //Screen Initialization
	 TM_HD44780_Init(16, 2);

	 //Some basic test output on LCD
//	 TM_HD44780_Puts(0, 4, "0 km/h");
		TM_HD44780_Puts(0,FIRST_ROW,"Counter v.0.1");
		TM_HD44780_Puts(0,SECOND_ROW, "Aut:Tomek Ferens");
		my_delay_ms(1000);
		TM_HD44780_Clear();
		short int state = menu[USB_CONF_MENU].substate;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if(round_finished)
		{
eval_velocity();
			round_finished = 0;
			round_time_ms = 0;
		}
		if (last_option != toggled_menu) {
			switch (toggled_menu) {
			case MAIN_MENU:
				//TODO: evalue velocity and other variables
				draw_state_lcd(&menu[MAIN_MENU]);
				break;
			case STAT_MENU:
				//TODO: evalue average and total variables
				draw_state_lcd(&menu[STAT_MENU]);
				break;
			case STAT_MENU2:
				//TODO: evalue average and total variables
				draw_state_lcd(&menu[STAT_MENU2]);
				break;
			case USB_CONF_MENU:
				draw_state_lcd(&menu[USB_CONF_MENU]);
				//TODO: implementation of usb handshake
				message_length = sprintf(data_to_send,"cou\n\r");
				CDC_Transmit_FS(data_to_send, message_length);

				short int retry_count = 0;

				while (state != USB_SUBSTATE_FINISHED  && received_data_flag == TRUE) {


//					if (retry_count > RETRY_COUNT) {
//						TM_HD44780_Puts(0, SECOND_ROW,
//								"Error in communication!!!");
//											Delayms(1500);
//											TM_HD44780_Clear();
//					//	break;
//						//TODO: transmit back info to host. clear substate to initial
//					}

					char buf22[2];
					itoa(retry_count,buf22,10);
					TM_HD44780_Puts(0, SECOND_ROW, buf22);

					char buf[USB_COMM_BUF_SIZE] = { 0 };

					for (int i = 0; i < USB_COMM_BUF_SIZE; i++)
						buf[i] = (char) received_data[i];

					if (state == USB_SUBSTATE_INIT) {
						if (strstr(buf, "host_ack")) {
							state = USB_SUBSTATE_ACK;
							retry_count = 0;
							received_data_flag = FALSE;
						} else {
							retry_count++;
							Delayms(1000);
						}
					}

					if (state == USB_SUBSTATE_ACK) {
						message_length = sprintf(data_to_send,
								"provide_set_param\r\n");

						CDC_Transmit_FS(data_to_send, message_length);
						TM_HD44780_Puts(0, SECOND_ROW, received_data);
						state = USB_SUBSTATE_CONF;
						received_data_flag = FALSE;
//						continue;
					}

					if (state == USB_SUBSTATE_CONF) {
						if (1) {
							message_length = sprintf(data_to_send,
									"set_param_confirmed\n\r");

							CDC_Transmit_FS(data_to_send, USB_COMM_BUF_SIZE);
//							TM_HD44780_Puts(0, SECOND_ROW, received_data);
							TM_HD44780_Puts(0, SECOND_ROW,
									"Setting <parameter> ...");
							Delayms(1000);
							state = USB_SUBSTATE_FINISHED;
							received_data_flag = FALSE;
//							break;
						} else {
							retry_count++;
							Delayms(1000);
						}
					}

				}
				break;
			}
			last_option = toggled_menu;
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 7;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CONTACTRON_Pin */
  GPIO_InitStruct.Pin = CONTACTRON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONTACTRON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MENU_BUTTON_Pin */
  GPIO_InitStruct.Pin = MENU_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MENU_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
