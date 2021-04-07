#include <stdint.h>
#include <Arduino.h>
#ifndef Encoder_h
#define Encoder_h

uint32_t getEncoderCount_left();
uint32_t getEncoderCount_right();

void resetEncoderCount_left();
void resetEncoderCount_right();

void ISR_LEFT();
void ISR_RIGHT();
#endif
