#include "ECE3.h"

void setup() {
  /* Set up serial communication for use at 9600bps */
  Serial.begin(9600);

  /* The ECE3_Init() Function is needed to set up the library for use */
  ECE3_Init();

  /* The following 2 lines will configure pushbutton 2 (left side) to
     trigger the "button" function via interrupt. This serves as an example
     of using the buttons to aid in development and troubleshooting.
  */
  pinMode(PUSH2, INPUT_PULLUP);
  attachInterrupt(PUSH2, button, FALLING);
}

/* Main program loop will get the encoder count and print to serial port */
void loop() {
  Serial.println(getEncoderCount_left());
  Serial.println(getEncoderCount_right());
  Serial.println();
  delay(300);
}

/* Interrupt hander for the button that was configured in setup. 
   The function will reset the encoder count via the ECE3 library
   functions.
*/
void button() {
  resetEncoderCount_left();
  resetEncoderCount_right();
}