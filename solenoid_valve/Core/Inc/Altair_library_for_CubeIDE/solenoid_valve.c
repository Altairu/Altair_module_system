#include "solenoid_valve.h"

// グローバル変数としてCANハンドルへのポインタを保持
static CAN_HandleTypeDef *g_hcan = NULL;

// タイムアウト管理用
static uint32_t last_can_rx_time = 0;
static uint8_t can_connected = 0;

// Valve ピン定義
// Valve 1 : PA0
// Valve 2 : PA1
// Valve 3 : PA6
// Valve 4 : PA7
// Valve 5 : PA8
// Valve 6 : PA9
// Valve 7 : PA15
// Valve 8 : PB3
// Valve 9 : PB6
// Valve 10: PB7
// Valve 11: PB8
// Valve 12: PB9

void Solenoid_Init(CAN_HandleTypeDef *hcan) {
    g_hcan = hcan;

    // CAN通信の初期化
    CanInitConfig config = Can_DefaultInitConfig(hcan);
    Can_Init(hcan, &config);

    // ピンの初期化状態を設定 (全てOFF)
    Solenoid_SetValveState(0x00, 0x00);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED_Pin (PA5)

    last_can_rx_time = HAL_GetTick();
    can_connected = 0;
}

void Solenoid_Update(void) {
    // CANメッセージの受信チェック
    if (g_can1_rx_data.new_data_flag) {
        if (g_can1_rx_data.std_id == SOLENOID_CAN_ID && g_can1_rx_data.dlc >= 2) {
            Solenoid_SetValveState(g_can1_rx_data.data[0], g_can1_rx_data.data[1]);
            last_can_rx_time = HAL_GetTick();
            can_connected = 1;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON
        }
        g_can1_rx_data.new_data_flag = 0;
    }

    // タイムアウト処理 (200ms以上無受信の場合)
    if (can_connected && (HAL_GetTick() - last_can_rx_time > SOLENOID_TIMEOUT_MS)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
        can_connected = 0;
    }
}

void Solenoid_SetValveState(uint8_t byte0, uint8_t byte1) {
    // Byte 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (byte0 & VALVE_1_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (byte0 & VALVE_2_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (byte0 & VALVE_3_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (byte0 & VALVE_4_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (byte0 & VALVE_5_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (byte0 & VALVE_6_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (byte0 & VALVE_7_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (byte0 & VALVE_8_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Byte 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (byte1 & VALVE_9_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (byte1 & VALVE_10_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (byte1 & VALVE_11_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (byte1 & VALVE_12_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
