//
// Created by QQZZ on 2026/11/1.
//
#include "gm6020.h"
#include "can.h"
volatile uint16_t motor_encoder = 0;
volatile int16_t motor_speed = 0; //速度单位为rpm
volatile int16_t motor_current = 0;
volatile uint8_t motor_temperature = 0;
static volatile uint32_t last_rx_tick = 0;
static volatile uint8_t received = 0;

void GM6020_Decode(const uint8_t data[8]){ //实时反馈数据解码
    motor_encoder = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    motor_speed = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    motor_current = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor_temperature = data[6];
    last_rx_tick = HAL_GetTick();
    received = 1;
}
// 控制ID为1的GM6020；同组ID 2～4的电流指令置零
uint8_t GM6020_SendCurrent(int16_t current){
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t mailbox;

    if (current > 16384) current = 16384;
    if (current < -16384) current = -16384;

    header.StdId = 0x1FE;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    data[0] = (uint8_t)((uint16_t)current >> 8);
    data[1] = (uint8_t)current;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        return 0;

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK;
}
// ID 1的电压指令；同组ID 2～4置零
uint8_t GM6020_SendVoltage(int16_t voltage){
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t mailbox;

    if (voltage > 25000) voltage = 25000;
    if (voltage < -25000) voltage = -25000;

    header.StdId = 0x1FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    data[0] = (uint8_t)((uint16_t)voltage >> 8);
    data[1] = (uint8_t)voltage;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        return 0;

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK;
}
uint8_t GM6020_IsOnline(void){//看门狗
    if (!received)
        return 0;

    uint32_t last = last_rx_tick;
    return (uint32_t)(HAL_GetTick() - last) <= 100U;
}