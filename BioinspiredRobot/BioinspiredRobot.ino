# include <Stepper.h>
# include "filters.h"
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_MLX90393.h"
#include <MadgwickAHRS.h>

// Parameters for filter
//const float cutoff_freq   = 100.0;  //Cutoff frequency in Hz
//const float sampling_time = 0.02; //Sampling time in seconds.
//IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

//float pitch=0.0, roll=0.0;
//float alpha = 0.95;
    

Adafruit_LSM6DSOX imu_sensor;
Adafruit_MLX90393 mag_sensor = Adafruit_MLX90393();
Madgwick madg_filter;

//#define LSM6DSOX_ADDRESS  // LSM6DSOX I2C address
//#define MLX90393_ADDRESS 0x19 // MLX90393 I2C address

// Define functions
void imu_sensor_setup();
float* get_imu_data();
float* get_mag_data();
float* calc_rpy(float* imu_data, float* mag_data);
float* calc_accel_without_g(float* imu_data, float* rpy);
void print_imu_mag(float* imu_data, float* mag_data);
void print_odometry(float* accel_without_g, float* rpy);
float average (float* array, int len);


unsigned long startTime = 0;
unsigned long duration = 5000; // 5 seconds in milliseconds
float imu_mag_readings[50]; // for delay(100) and 5 seconds
int iter = 0;
//int i = 0;
float imu_mag_mean = 0.0;


void setup() {
  // Start serial communication
  Serial.begin(19200);
  //Serial.println("starting setup");
  imu_sensor_setup();
  //Serial.println("imu ready");
  mag_sensor_setup();
  //Serial.println("starting timer");
  

  startTime = millis(); // Record the start time
  
  //madg_filter.begin(cutoff_freq);
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time

  float* imu_data = get_imu_data();
  float* mag_data = get_mag_data();

  float* rpy = calc_rpy(imu_data, mag_data);
  float* accel_without_g = calc_accel_without_g(imu_data, rpy);

  print_imu_mag(imu_data, mag_data);
  
  //print_odometry(accel_without_g, rpy);

  //print_imu_mag(imu_data, mag_data);
  // Calculate pitch and roll angles using complementary filter
  //float pitch_acc = atan2(imu_data[0], sqrt(imu_data[1] * imu_data[1] + imu_data[2] * imu_data[2])) * 180.0 / PI;
  //float roll_acc = atan2(-imu_data[4], imu_data[5]) * 180.0 / PI;

  //float dt = 0.01;  // Sample time (s)
  //float pitch_gyro = pitch + imu_data[4] * dt;  // Integrate gyro data for pitch
  //float roll_gyro = roll + imu_data[3] * dt;    // Integrate gyro data for roll

  // Combine accelerometer and gyroscope data using complementary filter
  //pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
  //float roll = alpha * roll_gyro + (1 - alpha) * roll_acc;

  //float magx, magy;
  //magx = mag_data[0];
  //magy = mag_data[1];
  //float heading_in_degrees = 180.0*atan2(magy,magx)/PI;
  //Serial.print(pitch);
  //Serial.print(" ");
  //Serial.print(roll);
  //Serial.print(" ");
  //Serial.print(heading_in_degrees);
  //Serial.println();


  //print_accel_mag(imu_data, mag_data);
  iter = iter + 1;
  delay(100); // Loop time will approx. match the sampling time.
}




/////////////////////////////////////////////////////////////////////////////
/////////////                helper functions               /////////////////
/////////////////////////////////////////////////////////////////////////////

void mag_sensor_setup() {
  if (! mag_sensor.begin_I2C(0x18)) {          // hardware I2C mode, can pass in address & alt Wire
    //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No magnetometer found ... check your wiring?");
    while (1) { delay(10); }
  }
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  mag_sensor.setResolution(MLX90393_X, MLX90393_RES_17);
  mag_sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
  mag_sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
  // Set oversampling
  mag_sensor.setOversampling(MLX90393_OSR_3);
  // Set digital filtering
  mag_sensor.setFilter(MLX90393_FILTER_5);
}

void imu_sensor_setup() {
  if (!imu_sensor.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) { delay(10); }
  }
}

float* get_imu_data() {
  sensors_event_t accel, gyro, temp;
  imu_sensor.getEvent(&accel, &gyro, &temp);
  static float data[6];
  data[0] = accel.acceleration.x;
  data[1] = accel.acceleration.y;
  data[2] = accel.acceleration.z;
  data[3] = gyro.gyro.x;
  data[4] = gyro.gyro.y;
  data[5] = gyro.gyro.z;
  return data;
}

float* get_mag_data() {
  sensors_event_t event;
  mag_sensor.getEvent(&event);
  static float data[3];
  //float x, y, z;
  
  data[0] = event.magnetic.x;
  data[1] = event.magnetic.y;
  data[2] = event.magnetic.z;
  return data;
}


float* calc_rpy(float* imu_data, float* mag_data) {
  static float rpy[3];

  float roll = atan2(-imu_data[0] ,( sqrt((imu_data[1] * imu_data[1]) + (imu_data[2] * imu_data[2]))));
  float pitch = atan2 (imu_data[1] ,( sqrt ((imu_data[0] * imu_data[0]) + (imu_data[2] * imu_data[2]))));
  
  float Xh = (mag_data[0] * cos(pitch))+(mag_data[1] * sin(roll)*sin(pitch)) + (mag_data[2] * cos(roll) * sin(pitch));
  float Yh = (mag_data[1] * cos(roll)) - (mag_data[2] * sin(roll));

  float yaw =  atan2(Yh, Xh);

  rpy[0] = roll;
  rpy[1] = pitch;
  rpy[2] = yaw;
  return rpy;
}

float* calc_accel_without_g(float* imu_data, float* rpy) {
  static float accel_without_g[3];
   // Calculate the rotation matrix

  // Remove gravity from the accelerometer readings
  accel_without_g[0] = imu_data[0] - cos(rpy[0])*sin(rpy[1])*cos(rpy[2]) + sin(rpy[0])*sin(rpy[2]) * 9.81;
  accel_without_g[1] = imu_data[1] - cos(rpy[0])*sin(rpy[1])*sin(rpy[2]) - sin(rpy[0])*cos(rpy[2]) * 9.81;
  accel_without_g[2] = imu_data[2] - cos(rpy[0])*cos(rpy[1]) * 9.81;
  return accel_without_g;
}

void print_imu_mag(float* imu_data, float* mag_data) {
  Serial.print(imu_data[0] - imu_mag_mean);
  Serial.print(" ");
  Serial.print(imu_data[1]);
  Serial.print(" ");
  Serial.print(imu_data[2]);
  Serial.print(" ");
  Serial.print(imu_data[3]);
  Serial.print(" ");
  Serial.print(imu_data[4]);
  Serial.print(" ");
  Serial.print(imu_data[5]);
  Serial.print(" ");
  Serial.print(mag_data[0]);
  Serial.print(" ");
  Serial.print(mag_data[1]);
  Serial.print(" ");
  Serial.print(mag_data[2]);
  Serial.println();
}

void print_odometry(float* accel_without_g, float* rpy) {
  Serial.print(accel_without_g[0]);
  Serial.print(" ");
  Serial.print(accel_without_g[1]);
  Serial.print(" ");
  Serial.print(accel_without_g[2]);
  Serial.print(" ");
  Serial.print(rpy[0]);
  Serial.print(" ");
  Serial.print(rpy[1]);
  Serial.print(" ");
  Serial.print(rpy[2]);
  Serial.println();
}



