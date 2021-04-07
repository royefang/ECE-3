#include "ECE3.h"
QTRSensors IR;
#define P5_0 45
#define P5_2 61

void ECE3_Init(){
/* Pinmodes */

  pinMode(P5_2, INPUT);
  pinMode(P5_0, INPUT);


/* Setup Interrupts */

  attachInterrupt(P5_2, ISR_LEFT, FALLING);
  attachInterrupt(P5_0, ISR_RIGHT, FALLING);

  IR.setSensorPins((const uint8_t[]) {65, 48, 64, 47, 52, 68, 53, 69}, 8);
  IR.setEmitterPins(45, 61);
  IR.setTimeout(2500);
	
}

void ECE3_read_IR(uint16_t * sensorValues){
	return IR.read(sensorValues);
}
