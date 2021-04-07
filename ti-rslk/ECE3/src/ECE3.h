#ifndef ECE3_3
#define ECE3_3

#include <stdint.h>
#include "msp.h"
#include "lib_files/Encoder.h"
#include "lib_files/QTRSensors.h"

void ECE3_Init();
void ECE3_read_IR(uint16_t *);

#endif
