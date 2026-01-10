// 초음파 센서를 이용한 전방 충돌 방지
extern volatile bool obstacle_flag;
extern float distance;

void ultrasound_trigger(void);
void ultrasound_callback(TIM_HandleTypeDef *htim);
