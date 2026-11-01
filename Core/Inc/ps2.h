#ifndef PS2_H
#define PS2_H

#include <stdbool.h>
#include <stdint.h>

#include "drive_control.h"

bool PS2_ReadFrame(GamepadFrame *frame);
bool PS2_IsConnected(void);
uint8_t PS2_RedLight(void);
void PS2_SetInit(void);
void PS2_Vibration(uint8_t motor1, uint8_t motor2);
void PS2_print(void);
void PS2_Vofa_print(void);

#endif
