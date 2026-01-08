
// 아두이노 라이브러리에서 가져온 비트 위치
// M1, M2 오른쪽 바퀴
// M3, M4 왼쪽 바퀴
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

void cmd_forward(void);
void cmd_back(void);
void cmd_left(void);
void cmd_right(void);
void cmd_forward_left(void);
void cmd_forward_right(void);
void cmd_back_left(void);
void cmd_back_right(void);
void cmd_stop(void);
