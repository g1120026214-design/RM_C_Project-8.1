#ifndef RM_C_PROJECT1_GM6020_H
#define RM_C_PROJECT1_GM6020_H

#include <stdint.h>

extern volatile uint16_t motor_encoder;
extern volatile int16_t motor_speed;
extern volatile int16_t motor_current;
extern volatile uint8_t motor_temperature;

void GM6020_Decode(const uint8_t data[8]);
uint8_t GM6020_SendCurrent(int16_t current); //发送电流指令（存疑 这个函数我在实验室的电机上都跑不了）
uint8_t GM6020_SendVoltage(int16_t voltage); //发送电压指令（可以成功让电机转起来）
uint8_t GM6020_IsOnline(void); //看门狗 电机在线返回1 否则0
#endif