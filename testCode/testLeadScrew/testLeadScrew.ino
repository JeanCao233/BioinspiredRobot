/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-controls-stepper-motor-using-l298n-driver
 */

#include <AccelStepper.h>
#include <Servo.h>

//setup servo
Servo myservo1;
Servo myservo2;
int pos = 0; 

//setup step motor
AccelStepper stepper(AccelStepper::FULL4WIRE, 7, 6, 5, 4);


void setup() {
  Serial.begin(9600);
  myservo1.attach(9);
  myservo2.attach(10);
  stepper.setMaxSpeed(100);
  stepper.setCurrentPosition(0);  // set position to 0
}


void loop() {
  for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);
    myservo2.write(abs(180 - pos));             // tell servo to go to position in variable 'pos'
    delay(15);                       
  }
  delay(500);
  while(stepper.currentPosition() != 800)
  {
    stepper.setSpeed(60);
    stepper.runSpeed();
  }
 
  delay(1000);
  Serial.println("out of the if loop");
  for (pos = 100; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos);
    myservo2.write(abs(180 - pos));               // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  delay(500);
  while(stepper.currentPosition() != -800) 
  {
    stepper.setSpeed(-60);
    stepper.runSpeed();
  }
 
  delay(1000);
}
