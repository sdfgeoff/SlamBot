#ifndef chassis_h
#define chassis_h



const int PIN_LEFT_1 = 32;
const int PIN_LEFT_2 = 33;
const int PIN_RIGHT_1 = 25;
const int PIN_RIGHT_2 = 26;

const int PWM_CHANNEL_LEFT_1 = 3;
const int PWM_CHANNEL_LEFT_2 = 4;
const int PWM_CHANNEL_RIGHT_1 = 5;
const int PWM_CHANNEL_RIGHT_2 = 6;
const int PWM_MOTOR_FREQUENCY = 500;
const int PWM_MOTOR_RESOLUTION = 16;


void initChassis();
void setLeftMotor(int16_t speed);
void setRightMotor(int16_t speed);
void fatal_error(char* message);


#endif
