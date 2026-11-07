#ifndef RM_C_PROJECT1_GM6020_H
#define RM_C_PROJECT1_GM6020_H

#include <stdint.h>

extern volatile uint16_t motor_encoder;
extern volatile int16_t motor_speed;
extern volatile int16_t motor_current;
extern volatile uint8_t motor_temperature;

void GM6020_Decode(const uint8_t data[8]);
uint8_t GM6020_SendCurrent(int16_t current);
uint8_t GM6020_SendVoltage(int16_t voltage);
uint8_t GM6020_IsOnline(void);
#endif