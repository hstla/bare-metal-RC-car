#include "01_motor_controller.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*
 * DIR_SER, CLK, LATCH, EN
 * SER(Serial Data): 전송할 실제 비트 값
 * CLK(System Clock): 이 핀이 깜빡거릴 때마다 SER이 한 칸씩 안으로 이동
 * LATCH(Latch/Store): 8비트가 다 들어왔을 때, 이 핀을 치면 한번에 모터 드라이버로 한번에 전달.
 * EN(Output Enable): 칩 전체의 출력을 끄고 킨다.(0에서 켜짐)
 */
static void shift_out_byte(uint8_t data)
{
	for (int i = 7; i >= 0; i--)
	{
		HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_RESET);

		GPIO_PinState bit =
				(data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
		HAL_GPIO_WritePin(DIR_SER_GPIO_Port, DIR_SER_Pin, bit);

		HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_SET);
	}
}

static void latch_output(void)
{
	HAL_GPIO_WritePin(DIR_LATCH_GPIO_Port, DIR_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIR_LATCH_GPIO_Port, DIR_LATCH_Pin, GPIO_PIN_SET);
}

static uint8_t forward(void)
{
	uint8_t bits = 0;

  // 오른쪽 바퀴
	bits |= (1 << MOTOR1_A);
	bits &= ~(1 << MOTOR1_B);
	bits |= (1 << MOTOR2_A);
	bits &= ~(1 << MOTOR2_B);

	// 왼쪽 바퀴
	bits |= (1 << MOTOR3_A);
	bits &= ~(1 << MOTOR3_B);
	bits |= (1 << MOTOR4_A);
	bits &= ~(1 << MOTOR4_B);

	return bits;
}

static uint8_t back(void)
{
	uint8_t bits = 0;

	// 오른쪽 바퀴
	bits &= ~(1 << MOTOR1_A);
	bits |= (1 << MOTOR1_B);
	bits &= ~(1 << MOTOR2_A);
	bits |= (1 << MOTOR2_B);

	// 왼쪽 바퀴
	bits &= ~(1 << MOTOR3_A);
	bits |= (1 << MOTOR3_B);
	bits &= ~(1 << MOTOR4_A);
	bits |= (1 << MOTOR4_B);

	return bits;
}

static void apply_bits(uint8_t bits)
{
	shift_out_byte(bits);
	latch_output();

	// 출력 enable (OE active low)
	HAL_GPIO_WritePin(DIR_EN_GPIO_Port, DIR_EN_Pin, GPIO_PIN_RESET);
}

static void set_speed_percent(uint8_t left_pct, uint8_t right_pct)
{
	uint16_t left_ccr = ((uint16_t)(left_pct * 49) / 100);
	uint16_t right_ccr =((uint16_t)(right_pct * 49) / 100);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, left_ccr);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_ccr);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_ccr);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_ccr);
}


void cmd_forward(void)
{
	uint8_t bits = forward();
	apply_bits(bits);

	set_speed_percent(95, 95);
}

void cmd_back(void)
{
	uint8_t bits = back();
	apply_bits(bits);

	set_speed_percent(95, 95);
}

void cmd_left(void)
{
	uint8_t bits = forward();
	apply_bits(bits);

	set_speed_percent(0, 95);
}

void cmd_right(void)
{
	uint8_t bits = forward();
	apply_bits(bits);

	set_speed_percent(95, 0);
}

void cmd_forward_left(void)
{
	uint8_t bits = forward();
	apply_bits(bits);

	set_speed_percent(85, 95);
}

void cmd_forward_right(void)
{
	uint8_t bits = forward();
	apply_bits(bits);

	set_speed_percent(95, 85);
}

void cmd_back_left(void)
{
	uint8_t bits = back();
	apply_bits(bits);

	set_speed_percent(85, 95);
}

void cmd_back_right(void)
{
	uint8_t bits = back();
	apply_bits(bits);

	set_speed_percent(95, 85);
}


void cmd_stop(void)
{
	set_speed_percent(0, 0);
}
