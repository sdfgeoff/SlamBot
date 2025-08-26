#include "events.h"
#include <Adafruit_MPU6050.h>


int latest_lidar_left = 0;
int latest_lidar_right = 0;

sensors_event_t latest_accelerometer;
sensors_event_t latest_gyro;

uint32_t latest_time = 0;


extern uint32_t latest_control_time = 0;
extern int latest_control_speed = 0;
extern int latest_control_direction = 0;