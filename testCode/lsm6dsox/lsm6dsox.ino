// Basic demo for accelerometer & gyro readings from Adafruit
// LSM6DSOX sensor

#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>
#include "filters.h"

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

float roll, pitch, yaw;
float ax, ay, az;

// Parameters for filter
const float cutoff_freq   = 100.0;  //Cutoff frequency in Hz
const float sampling_time = 0.2; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
    
// Creating a low-pass filter class
Filter f_ax(cutoff_freq, sampling_time, order);
Filter f_ay(cutoff_freq, sampling_time, order);
Filter f_az(cutoff_freq, sampling_time, order);

// Creating a high-pass filter class
Madgwick filter;

Adafruit_LSM6DSOX sox;
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  
  //printSetup();

  filter.begin(cutoff_freq);
}

void loop() {

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  //printData();

  // Update filter with sensor data
  filter.updateIMU(gyro.gyro.x/PI*180.0, gyro.gyro.y/PI*180.0, gyro.gyro.z/PI*180.0,
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  //roll = alpha * gyro_roll + (1 - alpha) * roll;
  //pitch = alpha * gyro_pitch + (1 - alpha) * pitch;
  roll = filter.getRoll()/180.0*PI;
  pitch = filter.getPitch()/180.0*PI; 
  yaw = (filter.getYaw() - 180.0)/180.0*PI;
  // Get orientation quaternion from filter
  //roll = atan2(accel.acceleration.y, accel.acceleration.z);
  //pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z));
  //float gyro_roll = roll + gyro.gyro.x * 0.1; // 0.1 is the time interval in seconds
  //float gyro_pitch = pitch + gyro.gyro.y * 0.1;

  

  // Calculate linear acceleration without gravity
  // The additional low pass filter may be redundent
  float accelWithoutGravityX = (accel.acceleration.x - (-sin(pitch))*(9.81));
  float accelWithoutGravityY = (accel.acceleration.y - (sin(roll) * cos(pitch))*(9.81));
  float accelWithoutGravityZ = (accel.acceleration.z - (cos(roll) * cos(pitch))*(9.81));

  /* Display the results (rotation is measured in rad/s) */
  //Serial.print("\t\tGyroWithougG X: ");
  //Serial.print(-20); // To freeze the lower limit
  //Serial.print(" ");
  //Serial.print(20); // To freeze the upper limit
  //Serial.print(" ");
  Serial.print(accelWithoutGravityX);
  Serial.print(" ");
  Serial.print(accelWithoutGravityY);
  Serial.print(" ");
  Serial.print(accelWithoutGravityZ);
  //Serial.println(" radians/s ");
  Serial.println();

  delay(10);

  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
}

void printSetup() {
  Serial.println("LSM6DSOX Found!");

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
}

void printData() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
}