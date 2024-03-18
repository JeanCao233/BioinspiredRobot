#include <Stepper.h>

// Define the number of steps per revolution and the motor pins
const int stepsPerRevolution = 200;
const int stepPin = 2;
const int dirPin = 3;

// Create a new instance of the Stepper class
Stepper stepper(stepsPerRevolution, stepPin, dirPin);

void setup() {
  // Set the speed of the motor in RPM (revolutions per minute)
  stepper.setSpeed(60); // Set speed to 60 RPM
}

void loop() {
  // Run the motor for 200 steps (1 revolution) at the set speed
  for (int i = 0; i < stepsPerRevolution; i++) {
    stepper.step(1); // Step one step
    delay(20); // Delay between steps (adjust as needed for your motor)
  }
  delay(1000); // Delay before running the motor again (adjust as needed)
}
