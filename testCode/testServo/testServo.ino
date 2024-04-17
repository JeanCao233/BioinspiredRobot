#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo
Servo myservo4;  // create servo object to control a servo
// twelve servo objects can be created on most boards

float pos = 0.0;    // variable to store the servo position
float pos1 = 0.0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo1.attach(2);  // attaches the servo on pin 3 to the servo object
  myservo2.attach(3);  // attaches the servo on pin 2 to the servo object
  myservo3.attach(4);
  myservo4.attach(5);
  Serial.println("Starting");
  myservo1.write(0);
  myservo2.write(80);
  myservo3.write(0);
  myservo4.write(180);
  delay(1500);

}

void loop() {
  
  //delay(1500);
  for (pos = 0; pos <= 80.0; pos += 1.0) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //Serial.println(pos);
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(abs(80 - pos));              // tell servo to go to position in variable 'pos'
    //Serial.println(abs(180 - pos));
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos1 = 0; pos1 <= 60.0; pos1 += 1.0) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //Serial.println(pos);
    myservo3.write(pos1);              // tell servo to go to position in variable 'pos'
    myservo4.write(abs(60 - pos1)+120);              // tell servo to go to position in variable 'pos'
  }

  for (pos = 80.0; pos >= 0; pos -= 1.0) { // goes from 180 degrees to 0 degrees
    Serial.println(pos);
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(abs(80.0 - pos));              // tell servo to go to position in variable 'pos'
    //Serial.println(abs(180 - pos))
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos1 = 60; pos1 >= 0.0; pos1 -= 1.0) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //Serial.println(pos);
    myservo3.write(pos1);              // tell servo to go to position in variable 'pos'
    myservo4.write(abs(60 - pos1)+120);              // tell servo to go to position in variable 'pos'
    //Serial.println(abs(180 - pos));
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(150);
  //Serial.println("testing3");
}