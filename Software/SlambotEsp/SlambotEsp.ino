#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h" // You may need to install this library


const uint8_t LED = 22;

const uint8_t PIN_I2C_SDA = 17;
const uint8_t PIN_I2C_SCL = 16;
const uint8_t PIN_TOF_LEFT_XSHUT = 23;
const uint8_t PIN_TOF_RIGHT_XSHUT = 18;


const uint8_t ADDRESS_MP6050 = 0x68;
const uint8_t ADDRESS_DEFAULT_VL53L0X = 0x29;

const uint8_t ADDRESS_TOF_LEFT = 0x01;
const uint8_t ADDRESS_TOF_RIGHT = 0x02;



void fatal_error(char* message) {
  while(1) {
    Serial.println(message);
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
}


void setup_tof_sensor(uint8_t from_address, uint8_t to_address, uint8_t xshut_pin) {
  Serial.print("Setting up TOF sensor:");
  Serial.print(from_address);
  Serial.print(" to ");
  Serial.print(to_address);

  

  

  delay(100);

}


Adafruit_VL53L0X tof_sensor_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_sensor_right = Adafruit_VL53L0X();
Adafruit_MPU6050 imu = Adafruit_MPU6050();


 
void setup() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

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
  tof_sensor_left.startRangeContinuous();

  Serial.println("Init right TOF Sensor");
  digitalWrite(PIN_TOF_RIGHT_XSHUT, HIGH);
  delay(2);  // 1.2ms boot time
  if (!tof_sensor_right.begin(ADDRESS_TOF_RIGHT)) {
    fatal_error("Failed to boot Right TOF Sensor");
  }
  tof_sensor_right.startRangeContinuous();

  Serial.println("Init IMU");
  if (!imu.begin()) {
    fatal_error("Failed to find MPU6050 chip");
  }
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Boot Complete");
}

float zaccum = 0.0;

void loop() {
  uint32_t start_time = millis();
  uint16_t left_distance = tof_sensor_left.readRange();
  uint16_t right_distance = tof_sensor_right.readRange();

  uint32_t end_time = millis();
  uint32_t duration = end_time - start_time;

  Serial.print("TOFs: "); Serial.print(left_distance); Serial.print(" "); Serial.print(right_distance); Serial.print(" T:"); Serial.println(duration);

  sensors_event_t acceleration_event, gyro_event, temp_event;
  start_time = millis();
  imu.getEvent(&acceleration_event, &gyro_event, &temp_event);
  end_time = millis();
  duration = end_time - start_time;
  Serial.print("Acceleration X: ");
  Serial.print(acceleration_event.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(acceleration_event.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(acceleration_event.acceleration.z);
  Serial.print(" ");

  Serial.print("Rotation X: ");
  Serial.print(gyro_event.gyro.x);
  Serial.print(", Y: ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(", Z: ");
  Serial.print(gyro_event.gyro.z);
  Serial.print(" ");

  Serial.print(" T: ");
  Serial.println(duration);

  float zrot = gyro_event.gyro.x * acceleration_event.acceleration.x + acceleration_event.acceleration.y * gyro_event.gyro.y + acceleration_event.acceleration.z * gyro_event.gyro.z;
  if (zrot < 0.5 && zrot > -0.5) {
    zrot = 0.0;
  }
  zaccum += zrot;
  Serial.print("ZROT: ");
  Serial.println(zrot);
  
  Serial.print("ZACCM: ");
  Serial.println(zaccum);
  delay(1);  
}
