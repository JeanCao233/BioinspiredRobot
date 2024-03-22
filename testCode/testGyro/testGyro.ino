// (c) Michael Schoeffler 2017, http://www.mschoeffler.de

#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "filters.h"
#include <MPU6050.h>
#include <MadgwickAHRS.h>

MPU6050 mpu;

//const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t ax, ay, az, gx, gy, gz;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float prev_roll=0.0, prev_pitch=0.0, prev_yaw=0.0;
float roll, pitch, yaw;
float delta_roll, delta_pitch, delta_yaw;

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// Parameters for filter
const float cutoff_freq   = 10.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
    
// Creating a low-pass filter class
Filter f_ax(cutoff_freq, sampling_time, order);
Filter f_ay(cutoff_freq, sampling_time, order);
Filter f_az(cutoff_freq, sampling_time, order);
Filter f_gx(cutoff_freq, sampling_time, order);
Filter f_gy(cutoff_freq, sampling_time, order);
Filter f_gz(cutoff_freq, sampling_time, order);
Filter f_ax_(cutoff_freq, sampling_time, order);
Filter f_ay_(cutoff_freq, sampling_time, order);
Filter f_az_(cutoff_freq, sampling_time, order);

// Creating a high-pass filter class
Madgwick filter;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  // Set accelerometer range to ±2g
  //mpu.setFullScaleAccelRange(AFS_SEL=0);

  //Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  //Wire.write(0x6B); // PWR_MGMT_1 register
  //Wire.write(0); // set to zero (wakes up the MPU-6050)
  //Wire.endTransmission(true);
  
  // Setting up sensor filters
  pinMode(A0, INPUT);
  // Enable pull-ups if necessary
  digitalWrite(A0, HIGH);

  filter.begin(cutoff_freq);

}

void loop() {

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  accelX = f_ax.filterIn(ax)/16384.0;
  accelY = f_ay.filterIn(ay)/16384.0;
  accelZ = f_az.filterIn(az)/16384.0;
  gyroX = f_gx.filterIn(gx)/131.0;
  gyroY = f_gy.filterIn(gy)/131.0;
  gyroZ = f_gz.filterIn(gz)/131.0;

  
  //Serial.print(" | filtered_gX = "); Serial.print(filtered_gX);
  //Serial.print(" | filtered_gY = "); Serial.print(filtered_gY);
  //Serial.print(" | filtered_gZ = "); Serial.print(filtered_gZ);
  

  // Update filter with sensor data
  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  // Get orientation quaternion from filter
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw() - 180;
  delta_roll = roll - prev_roll;
  delta_pitch = pitch - prev_pitch;
  delta_yaw = yaw - prev_yaw;
  prev_roll = roll;
  prev_pitch = pitch;
  prev_yaw = yaw;

  // Calculate linear acceleration without gravity
  // The additional low pass filter may be redundent
  float accelWithoutGravityX = (accelX - (-sin(pitch/180.0*PI)))*(-9.81);
  float accelWithoutGravityY = (accelY - (sin(roll/180.0*PI) * cos(pitch/180.0*PI)))*(-9.81);
  float accelWithoutGravityZ = (accelZ - (cos(roll/180.0*PI) * cos(pitch/180.0*PI)))*(-9.81);
  
  
  // print filtered data
  //printScaledData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  
  // print accelerations without gravity
  //printAccelerationWithoutG(accelWithoutGravityX, accelWithoutGravityY, accelWithoutGravityZ, roll, pitch, yaw);

  // print output
  printOutput(accelWithoutGravityX, accelWithoutGravityY, accelWithoutGravityZ);
  
  // delay
  delay(100);
}

void printScaledData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
  //Serial.print("Acceleration: ");
  // acceleration in g
  Serial.print(accelX);
  Serial.print(" ");
  Serial.print(accelY);
  Serial.print(" ");
  Serial.print(accelZ);
  Serial.print(" ");
  // angle in degrees
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.println(gyroZ);
}

void printAccelerationWithoutG(float accelWithoutGravityX, float accelWithoutGravityY, float accelWithoutGravityZ, float roll, float pitch, float yaw) {
  Serial.print("Accel X: "); Serial.print(accelWithoutGravityX); Serial.print("\t");
  Serial.print("Accel Y: "); Serial.print(accelWithoutGravityY); Serial.print("\t");
  Serial.print("Accel Z: "); Serial.print(accelWithoutGravityZ); Serial.print("\t");
  Serial.print("roll: "); Serial.print(roll); Serial.print("\t");
  Serial.print("pitch: "); Serial.print(pitch); Serial.print("\t");
  Serial.print("yaw: "); Serial.println(yaw);
}

void printOutput(float accelWithoutGravityX, float accelWithoutGravityY, float accelWithoutGravityZ) {
  Serial.print(accelWithoutGravityX);
  Serial.print(" ");
  Serial.print(accelWithoutGravityY);
  Serial.print(" ");
  Serial.println(accelWithoutGravityZ);
}