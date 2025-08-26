#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h" // You may need to install this library
#include "chassis.h"
#include "webserver.h"
#include "events.h"
#include <WiFi.h>

const uint8_t PIN_I2C_SDA = 17;
const uint8_t PIN_I2C_SCL = 16;
const uint8_t PIN_TOF_LEFT_XSHUT = 23;
const uint8_t PIN_TOF_RIGHT_XSHUT = 18;


const uint8_t ADDRESS_MP6050 = 0x68;
const uint8_t ADDRESS_DEFAULT_VL53L0X = 0x29;

const uint8_t ADDRESS_TOF_LEFT = 0x01;
const uint8_t ADDRESS_TOF_RIGHT = 0x02;

const char *ssid = "NotForAnything";
const char *password = "TotallyTrue";


Adafruit_VL53L0X tof_sensor_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_sensor_right = Adafruit_VL53L0X();
Adafruit_MPU6050 imu = Adafruit_MPU6050();

void setup() {

  Serial.begin(115200);

  Serial.println("Connecting to Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Init TOF Sensors");
  pinMode(PIN_TOF_LEFT_XSHUT, OUTPUT);
  pinMode(PIN_TOF_RIGHT_XSHUT, OUTPUT);
  // Turn off both TOF sensors
  digitalWrite(PIN_TOF_LEFT_XSHUT, LOW);
  digitalWrite(PIN_TOF_RIGHT_XSHUT, LOW);
  delay(10);

  Serial.println("Init left TOF Sensor");
  digitalWrite(PIN_TOF_LEFT_XSHUT, HIGH);
  delay(2);  // 1.2ms boot time
  if (!tof_sensor_left.begin(ADDRESS_TOF_LEFT)) {
    fatal_error("Failed to boot Left TOF Sensor");
  }
  //tof_sensor_left.startRangeContinuous();

  Serial.println("Init right TOF Sensor");
  digitalWrite(PIN_TOF_RIGHT_XSHUT, HIGH);
  delay(2);  // 1.2ms boot time
  if (!tof_sensor_right.begin(ADDRESS_TOF_RIGHT)) {
    fatal_error("Failed to boot Right TOF Sensor");
  }
  //tof_sensor_right.startRangeContinuous();

  Serial.println("Init IMU");
  if (!imu.begin()) {
    fatal_error("Failed to find MPU6050 chip");
  }
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Init Chassis");
  initChassis();
  setLeftMotor(0);
  setRightMotor(0);

  Serial.println("Init Webserver");
  init_webserver();

  Serial.println("Boot Complete");
}

uint32_t last_loop_time = 0;

void loop() {
  uint32_t this_loop_time = millis();
  uint32_t duration = this_loop_time - last_loop_time;
  // Serial.println(duration);
  last_loop_time = this_loop_time;

  tof_sensor_left.waitRangeComplete();
  tof_sensor_right.waitRangeComplete();
  uint16_t left_distance = tof_sensor_left.readRangeResult();
  uint16_t right_distance = tof_sensor_right.readRangeResult();
  tof_sensor_left.startRange();
  tof_sensor_right.startRange();

  sensors_event_t acceleration_averaged, gyro_averaged;

  uint8_t averaging_samples = 30;

  acceleration_averaged.acceleration.x = 0.0;
  acceleration_averaged.acceleration.y = 0.0;
  acceleration_averaged.acceleration.z = 0.0;
  gyro_averaged.gyro.x = 0.0;
  gyro_averaged.gyro.y = 0.0;
  gyro_averaged.gyro.z = 0.0;

  sensors_event_t acceleration_event, gyro_event, temp_event;
  for (uint8_t i=0; i<averaging_samples; i++) {
    imu.getEvent(&acceleration_event, &gyro_event, &temp_event);
    acceleration_averaged.acceleration.x += acceleration_event.acceleration.x;
    acceleration_averaged.acceleration.y += acceleration_event.acceleration.y;
    acceleration_averaged.acceleration.z += acceleration_event.acceleration.z;
    gyro_averaged.gyro.x += gyro_event.gyro.x;
    gyro_averaged.gyro.y += gyro_event.gyro.y;
    gyro_averaged.gyro.z += gyro_event.gyro.z;
  }

  acceleration_averaged.acceleration.x = acceleration_averaged.acceleration.x / float(averaging_samples);
  acceleration_averaged.acceleration.y = acceleration_averaged.acceleration.y / float(averaging_samples);
  acceleration_averaged.acceleration.z = acceleration_averaged.acceleration.z / float(averaging_samples);
  gyro_averaged.gyro.x = gyro_averaged.gyro.x / float(averaging_samples);
  gyro_averaged.gyro.y = gyro_averaged.gyro.y / float(averaging_samples);
  gyro_averaged.gyro.z = gyro_averaged.gyro.z / float(averaging_samples);

  latest_accelerometer = acceleration_averaged;
  latest_gyro = gyro_averaged;
  latest_lidar_left = left_distance;
  latest_lidar_right = right_distance;
  latest_time = millis();

  Serial.print("{");
  Serial.print("\"t\":");
  Serial.print(millis());
  Serial.print(",\"rx\":");
  Serial.print(gyro_averaged.gyro.x, 6);
  Serial.print(",\"ry\":");
  Serial.print(gyro_averaged.gyro.y, 6);
  Serial.print(",\"rz\":");
  Serial.print(gyro_averaged.gyro.z, 6);
  Serial.print(",\"ax\":");
  Serial.print(acceleration_averaged.acceleration.x, 6);
  Serial.print(",\"ay\":");
  Serial.print(acceleration_averaged.acceleration.y, 6);
  Serial.print(",\"az\":");
  Serial.print(acceleration_averaged.acceleration.z, 6);
  Serial.print(",\"dl\":");
  Serial.print(left_distance);
  Serial.print(",\"dr\":");
  Serial.print(right_distance);
  Serial.println("}");

  poll_webserver();

  if (millis() - latest_control_time < 1000) {
    setLeftMotor(latest_control_speed - latest_control_direction);
    setRightMotor(latest_control_speed + latest_control_direction);
  } else {
    setLeftMotor(0);
    setRightMotor(0);
  }

  delay(1);
  

}
