/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "bno055_stm32.h"
#include "pid_controller.h"
#include "dshot150.h"

#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO_I2C_HANDLE &hi2c1
#define DEBUG_UART_HANDLE &huart2

#define SAMPLE_TIME 0.01f
#define PID_KP 15.0f
#define PID_KI 30.0f
#define PID_KD 70.0f
#define PID_TAU 0.06f
#define REF_ANGLE 0.0f

//#define PHIL_S_LAB
//#define REVERSE_SPIN_DIRECTION

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t emergency_stop_flag = 0;
volatile bno055_vector_t bno_vector;
volatile float copter_angle;
volatile uint16_t speed_ref;

//PID
PID_t pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM15_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_Init();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 0);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(12, 12);
	ssd1306_WriteString("1-DOF copter demo", Font_6x8, White);
	ssd1306_SetCursor(30, 24);
	ssd1306_WriteString("3S AM32 ESC", Font_6x8, White);
	ssd1306_SetCursor(30, 36);
	ssd1306_WriteString("BNO055 AHRS", Font_6x8, White);
	ssd1306_SetCursor(20, 48);
	ssd1306_WriteString("PID controller", Font_6x8, White);

	ssd1306_UpdateScreen();

	bno055_assignI2C(BNO_I2C_HANDLE);
	bno055_setup();
	bno055_setOperationModeNDOF();

	printf("BNO055 config finished.\r\n");

#ifdef PHIL_S_LAB
	PID_Init_Phil_s_Lab(&pid, PID_KP, PID_KI, PID_KD, PID_TAU, 100.0f, 1800.0f,
			100.0f, 1800.0f, SAMPLE_TIME);
#else
	PID_Init_Bartek_s_Lab(&pid, PID_KP, PID_KI, PID_KD, PID_TAU, 100.0f,
			1800.0f, SAMPLE_TIME);
#endif

	printf("PID config finished.\r\n");

	HAL_Delay(2000); // let the motor stop after uC RST

	dshot_arm_esc();
	printf("ESC armed.\r\n");
#ifdef REVERSE_SPIN_DIRECTION
  	dshot_led_on(DSHOT_BLUE_LED); // does not work for AM32
  	dshot_set_spin_direction(DSHOT_SPIN_DIRECTION_REVERSE);
  #else
	dshot_led_on(DSHOT_GREEN_LED); // does not work for AM32
	dshot_set_spin_direction(DSHOT_SPIN_DIRECTION_NORMAL);
#endif
	printf("Direction set.\r\n");
	HAL_TIM_Base_Start_IT(&htim15); // control loop interrupt
	printf("Control loop TIM started.\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == YELLOW_BUTTON_Pin)
	{
		emergency_stop_flag = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM15)
	{
		bno_vector = bno055_getVectorEuler();
		copter_angle = -bno_vector.y;

#ifdef PHIL_S_LAB
		speed_ref = (uint16_t) PID_Controller_Phil_s_Lab(&pid, REF_ANGLE,
				copter_angle);
#else
		speed_ref = (uint16_t) PID_Controller_Bartek_s_Lab(&pid, REF_ANGLE,
				copter_angle);
#endif

		// Double-check :))
		if (speed_ref < 48)
		{
			speed_ref = 48;
		}
		else if (speed_ref > 2047)
		{
			speed_ref = 2047;
		}
		else
		{
			__NOP();
		}

		// Tripple-check :)))
		if (0 == check_dshot_ref_speed(speed_ref) && (0 == emergency_stop_flag))
		{
			dshot_send_ref_speed(speed_ref);
		}

	}
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
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
