# include <Stepper.h>
# include "filters.h"

// Simulated temperature sensor value
int temperature = 25;
char incomingByte = "not specified";
int val = 0;

void setup() {
  // Start serial communication
  Serial.begin(19200);
}

void loop() {
  // Simulate temperature reading
  temperature = random(20, 30); // Random temperature between 20 and 30

  // Print temperature reading to Serial Monitor
  //Serial.print("Temperature: ");
  //Serial.print(temperature);
  //Serial.println(" C");
  //incomingByte = Serial.read();
  //val = atoi(&incomingByte);
  //Serial.println(val);
  

  // Delay for demonstration purposes
  delay(1000);
}
