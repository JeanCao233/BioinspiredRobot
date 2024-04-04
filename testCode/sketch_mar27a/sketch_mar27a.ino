/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported magnetometer and output
  uTesla data as well as the hard iron calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Wire.h>

#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

void setup(void) {
   Serial.begin(115200);
  Serial.println("Magnetometer Compass"); Serial.println("");
  

  if (sensor.begin_I2C())
  {
    Serial.println("Found a MLX90393 sensor");
  }
  else
  {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
   float x, y, z;

    if(sensor.readData(&x, &y, &z)) {
  min_x = max_x = x;
  min_y = max_y = y;
  min_z = max_z = z;
  delay(10);
    }
}



void loop() {
   float x, y, z;

    if(sensor.readData(&x, &y, &z)) {

  
  Serial.print("Mag: (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.print(")");

  min_x = min(min_x, x);
  min_y = min(min_y, y);
  min_z = min(min_z, z);

  max_x = max(max_x, x);
  max_y = max(max_y, y);
  max_z = max(max_z, z);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;
  mid_z = (max_z + min_z) / 2;
  Serial.print(" Hard offset: (");
  Serial.print(mid_x); Serial.print(", ");
  Serial.print(mid_y); Serial.print(", ");
  Serial.print(mid_z); Serial.print(")");  

  Serial.print(" Field: (");
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(", ");
  Serial.print((max_z - min_z)/2); Serial.println(")");    
  delay(10); 
    }
}