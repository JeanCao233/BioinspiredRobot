# include <Stepper.h>
# include "filters.h"

// Parameters for filter
const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
    
// Creating a low-pass filter class
Filter f(cutoff_freq, sampling_time, order);

void setup() {
  // Start serial communication
  Serial.begin(19200);

  // Setting up sensor filters
  pinMode(A0, INPUT);
  // Enable pull-ups if necessary
  digitalWrite(A0, HIGH);

}

void loop() {
  int value = analogRead(0);

  // Filtering input signal
  float filteredval = f.filterIn(value);
  
  //View with Serial Plotter
  Serial.print(value, DEC);
  Serial.print(",");
  Serial.println(filteredval, 4);
  delay(5); // Loop time will approx. match the sampling time.
}
