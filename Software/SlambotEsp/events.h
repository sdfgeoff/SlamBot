#ifndef events_h
#define events_h

#include <Adafruit_MPU6050.h>


extern int latest_lidar_left;
extern int latest_lidar_right;

extern sensors_event_t latest_accelerometer;
extern sensors_event_t latest_gyro;

extern uint32_t latest_time;


extern uint32_t latest_control_time;
extern int latest_control_speed;
extern int latest_control_direction;

#endif