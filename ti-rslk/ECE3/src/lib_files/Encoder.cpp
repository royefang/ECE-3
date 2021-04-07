#include <stdlib.h>

#include "Encoder.h"

volatile uint32_t left_count = 0;
volatile uint32_t right_count = 0;

uint32_t getEncoderCount_left(){
	return left_count;
}

uint32_t getEncoderCount_right(){
	return right_count;
}

void resetEncoderCount_left(){
	left_count = 0;
}

void resetEncoderCount_right(){
	right_count = 0;
}

void ISR_LEFT() {
  left_count++;
}
void ISR_RIGHT() {
  right_count++;
}