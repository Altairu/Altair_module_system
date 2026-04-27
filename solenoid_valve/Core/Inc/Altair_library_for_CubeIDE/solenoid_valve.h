#ifndef SOLENOID_VALVE_H
#define SOLENOID_VALVE_H

#include "stm32f4xx_hal.h"
#include "can_lib.h"

// ペイロードバイトでのビット位置
#define VALVE_1_BIT  (1 << 0)
#define VALVE_2_BIT  (1 << 1)
#define VALVE_3_BIT  (1 << 2)
#define VALVE_4_BIT  (1 << 3)
#define VALVE_5_BIT  (1 << 4)
#define VALVE_6_BIT  (1 << 5)
#define VALVE_7_BIT  (1 << 6)
#define VALVE_8_BIT  (1 << 7)
#define VALVE_9_BIT  (1 << 0)
#define VALVE_10_BIT (1 << 1)
#define VALVE_11_BIT (1 << 2)
#define VALVE_12_BIT (1 << 3)

#define SOLENOID_CAN_ID 0x300

// 無受信タイムアウト
#define SOLENOID_TIMEOUT_MS 200

void Solenoid_Init(CAN_HandleTypeDef *hcan);
void Solenoid_Update(void);
void Solenoid_SetValveState(uint8_t byte0, uint8_t byte1);

#endif // SOLENOID_VALVE_H
