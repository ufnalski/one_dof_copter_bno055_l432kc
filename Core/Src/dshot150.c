/*
 * dshot150.c
 *
 *  Created on: Sep 1, 2023
 *      Author: user
 */

#include "dshot150.h"
#include "tim.h"

uint8_t dshot_dmabuffer_ccr[17];

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			HAL_GPIO_TogglePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin);
			HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
		}
	}
}

// https://github.com/mokhwasomssi/stm32_hal_dshot
uint16_t dshot_prepare_packet(uint16_t _command)
{
	uint16_t packet;

	packet = (_command << 1) | (DSHOT_TELEMETRY ? 1 : 0);

	// compute checksum
	uint8_t csum = 0;
	uint16_t csum_data = packet;

	for (uint8_t i = 0; i < 3; i++)
	{
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;

	packet = (packet << 4) | csum;

	return packet; //^0x0110; // this is not CRC!
}

void dshot_prepare_dmabuffer(uint8_t *_dshot_dmabuffer_ccr, uint16_t _value)
{
	uint16_t packet;
	packet = dshot_prepare_packet(_value);

	for (int i = 0; i < 16; i++)
	{
		_dshot_dmabuffer_ccr[i] =
				(packet & 0x8000) ? DSHOT150_BIT_1 : DSHOT150_BIT_0;
		packet <<= 1;
	}
	// https://electronics.stackexchange.com/questions/377604/stm32-pwm-generates-excessive-pulses
	_dshot_dmabuffer_ccr[16] = 0;
}

void dshot_arm_esc(void)
{
	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 0);

	for (int i = 0; i < 1000; i++)
	{
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
				(uint32_t*) dshot_dmabuffer_ccr, 17);
		HAL_Delay(1);
	}
}

//void dshot_disarm_esc(void)
//{
//	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 0);
//
//	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
//			(uint32_t*) dshot_dmabuffer_ccr, 17);
//	HAL_Delay(1);
//}

void dshot_led_on(DShot_LedColorTypeDef led_color)
{
	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 22 + led_color);

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
			(uint32_t*) dshot_dmabuffer_ccr, 17);
	HAL_Delay(1);
}

void dshot_led_off(DShot_LedColorTypeDef led_color)
{
	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 26 + led_color);

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
			(uint32_t*) dshot_dmabuffer_ccr, 17);
	HAL_Delay(1);
}

void dshot_set_spin_direction(DShot_SpinDirectionTypeDef spin_dir)
{
	if (spin_dir == DSHOT_SPIN_DIRECTION_NORMAL)
	{
		dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 20);
	}
	else if (spin_dir == DSHOT_SPIN_DIRECTION_REVERSE)
	{
		dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 21);
	}
	else
	{
		dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 0);
	}

	for (int i = 0; i < 10; i++)
	{
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
				(uint32_t*) dshot_dmabuffer_ccr, 17);
		HAL_Delay(1);
	}
}

//void dshot_save_settings(void)
//{
//	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 12);
//
//	for (int i = 0; i < 6; i++)
//	{
//		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
//				(uint32_t*) dshot_dmabuffer_ccr, 17);
//		HAL_Delay(1);
////		HAL_Delay(50);
//	}
//	HAL_Delay(50);
//}

void dshot_send_ref_speed(uint16_t _motor_speed)
{
	dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, _motor_speed);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
			(uint32_t*) dshot_dmabuffer_ccr, 17);

/* Source of time base is configured  to generate interrupts at regular
 * time intervals. Care must be taken if HAL_Delay() is called from a
 * peripheral ISR process, the Tick interrupt line must have higher priority
 * (numerically lower) than the peripheral interrupt. Otherwise the caller
 * ISR process will be blocked.
 * [stm32l4xx_hal.c]
 */

//	HAL_Delay(1);
}

uint8_t check_dshot_ref_speed(uint16_t speed)
{
	if ((speed < SPEED_MIN) || (speed > SPEED_MAX))
		return 1;
	else
		return 0;
}
