/*
 * 초음파 센서(US)를 이용한 전방 충돌 방지
 * trig(출력): 최소 10us로 출력하면 10kHz이상의 초음파를 방출
 * echo(입력): 초음파를 방출하는 즉시 echo High로 올림, 초음파가 돌아오는 즉시 0으로 바꿔 High였던 시간을 거리로 계산함.
 */
#include "main.h"
#include <stdbool.h>

volatile bool obstacle_flag = false;
float distance = 0;
static uint32_t ic_val1 = 0, ic_val2 = 0, diff = 0;
static bool is_first_captured = false;

extern TIM_HandleTypeDef htim1;
extern uint8_t rx;

void ultrasound_trigger(void) {
	HAL_GPIO_WritePin(GPIOC, US_trig_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 100; i++) __NOP();
	HAL_GPIO_WritePin(GPIOC, US_trig_Pin, GPIO_PIN_RESET);
}


/*
 * 초음파 거리 계산 및 장애물 로직 처리
 * 거리 = 속도 * 시간
 * 음속은 340m/s 인데 왕복이기 떄문에 2로 나눠야 함.
 * d = (IC_Val2 - IC_Val1) * 0.017 / 2
 */
void ultrasound_callback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (is_first_captured == false) {
			ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			is_first_captured = true;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		} else {
			ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			diff = ic_val2 - ic_val1;
			distance = diff * 0.017f;
			is_first_captured = false;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

			if (distance > 0 && distance <= 15.0f) {
				obstacle_flag = true;
				HAL_GPIO_WritePin(GPIOC, LED_Output_Pin, GPIO_PIN_SET);
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

				if (rx == 'F' || rx == 'G' || rx == 'H')
				{
					cmd_stop();
					rx = 'S';
				}
			}
			else if (distance > 20.0f) {
				obstacle_flag = false;
				HAL_GPIO_WritePin(GPIOC, LED_Output_Pin, GPIO_PIN_RESET);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
			}
		}
	}
}
