#include "mdd.h"
#include "Altair_library_for_CubeIDE/altair.h"
#include <stdint.h>

typedef enum {
    MDD_MODE_PARAM_SET = 0,
    MDD_MODE_CONTROL = 1
} MDD_Mode_t;

typedef enum {
    MDD_CTRL_SPEED = 0,
    MDD_CTRL_ANGLE = 1
} MDD_CtrlMode_t;

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

#define MOTOR_COUNT             4

#define ENCODER_PPR             8192
#define ENCODER_PERIOD_MS       1
#define CONTROL_PERIOD_MS       10

#define STATUS_PERIOD_MS       10
#define INIT_WAIT_TIMEOUT_MS   2000
#define CAN_RX_TIMEOUT_MS      1000

#define CAN_ID_PARAM_BASE      0x200U
#define CAN_ID_MODE            0x210U
#define CAN_ID_TARGET          0x220U
#define CAN_ID_STATUS          0x230U

#define ERR_NORMAL         0x00U
#define ERR_INIT_TIMEOUT   0x01U
#define ERR_CAN_RX_TIMEOUT 0x02U
#define ERR_CAN_TX_FAIL    0x04U

#define LIMIT_SW_COUNT           4
#define LIMIT_SW_DEBOUNCE_COUNT  3U

static MDD_Mode_t g_mode = MDD_MODE_PARAM_SET;
static MDD_CtrlMode_t g_ctrl_mode[MOTOR_COUNT] = {
    MDD_CTRL_SPEED, MDD_CTRL_SPEED, MDD_CTRL_SPEED, MDD_CTRL_SPEED
};

static MotorDriver g_motor[MOTOR_COUNT];
static Encoder g_encoder[MOTOR_COUNT];
static EncoderData g_encoder_data[MOTOR_COUNT];
static Pid g_pid[MOTOR_COUNT];

static double g_target_val[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
static int g_output_dir[MOTOR_COUNT] = {1, 1, 1, 1};
static uint8_t g_param_received[MOTOR_COUNT] = {0U, 0U, 0U, 0U};

static uint8_t g_error_code = ERR_NORMAL;

static uint32_t boot_tick = 0U;
static uint32_t last_can_rx_tick = 0U;
static uint32_t last_control_tick = 0U;
static uint32_t last_status_tick = 0;

static uint8_t g_limit_sw_state[LIMIT_SW_COUNT] = {0U, 0U, 0U, 0U};
static uint8_t g_limit_sw_diff_count[LIMIT_SW_COUNT] = {0U, 0U, 0U, 0U};

static int MDD_ClampPwm(double in)
{
    int v = (int)in;
    if (v > 100) {
        v = 99;
    }
    if (v < -100) {
        v = -99;
    }
    return v;
}

static uint8_t MDD_AllParamsReceived(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_param_received[i] == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t MDD_ReadLimitSw(uint8_t index) {
    uint16_t pin = GPIO_PIN_0;
    switch (index) {
        case 0: pin = GPIO_PIN_0; break;
        case 1: pin = GPIO_PIN_1; break;
        case 2: pin = GPIO_PIN_2; break;
        default: pin = GPIO_PIN_3; break;
    }

    return (HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static int16_t MDD_ReadI16Le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void MDD_UpdateLimitSwDebounce(void) {
    for (uint8_t i = 0U; i < LIMIT_SW_COUNT; i++) {
        uint8_t raw = MDD_ReadLimitSw(i);

        if (raw == g_limit_sw_state[i]) {
            g_limit_sw_diff_count[i] = 0U;
        } else {
            if (g_limit_sw_diff_count[i] < LIMIT_SW_DEBOUNCE_COUNT) {
                g_limit_sw_diff_count[i]++;
            }
            if (g_limit_sw_diff_count[i] >= LIMIT_SW_DEBOUNCE_COUNT) {
                g_limit_sw_state[i] = raw;
                g_limit_sw_diff_count[i] = 0U;
            }
        }
    }
}

static void MDD_ProcessParamFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((id < CAN_ID_PARAM_BASE) || (id > (CAN_ID_PARAM_BASE + 3U)) || (dlc != 8U)) {
        return;
    }

    int idx = (int)(id - CAN_ID_PARAM_BASE);
    int16_t p_raw = MDD_ReadI16Le(&data[0]);
    int16_t i_raw = MDD_ReadI16Le(&data[2]);
    int16_t d_raw = MDD_ReadI16Le(&data[4]);
    int16_t dia_dir_raw = MDD_ReadI16Le(&data[6]);

    double kp = (double)p_raw / 1000.0;
    double ki = (double)i_raw / 1000.0;
    double kd = (double)d_raw / 1000.0;

    Pid_Init(&g_pid[idx]);
    Pid_setGainWithLimit(&g_pid[idx], kp, ki, kd, 0.01, 100.0);

    if (dia_dir_raw >= 0) {
        g_output_dir[idx] = 1;
        g_encoder[idx].diameter = (double)dia_dir_raw;
    } else {
        g_output_dir[idx] = -1;
        g_encoder[idx].diameter = (double)(-dia_dir_raw);
    }

    g_param_received[idx] = 1U;
}

static void MDD_ProcessModeFrame(const uint8_t *data, uint8_t dlc)
{
    if (dlc != 4U) {
        return;
    }

    if (MDD_AllParamsReceived() == 0U) {
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_ctrl_mode[i] = (data[i] == 0U) ? MDD_CTRL_SPEED : MDD_CTRL_ANGLE;
    }

    g_mode = MDD_MODE_CONTROL;
#ifdef LED_Pin
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
#endif
}

static void MDD_ProcessTargetFrame(const uint8_t *data, uint8_t dlc)
{
    if (dlc != 8U) {
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        int16_t raw = MDD_ReadI16Le(&data[i * 2]);
        g_target_val[i] = (double)raw / 10.0;
    }
}

static void MDD_ProcessCan(void)
{
    if (g_can1_rx_data.new_data_flag == 0U) {
        return;
    }

    g_can1_rx_data.new_data_flag = 0U;
    last_can_rx_tick = HAL_GetTick();

    uint32_t id = g_can1_rx_data.std_id;
    uint8_t dlc = g_can1_rx_data.dlc;
    uint8_t *data = g_can1_rx_data.data;

    if (g_mode == MDD_MODE_PARAM_SET) {
        if ((id >= CAN_ID_PARAM_BASE) && (id <= (CAN_ID_PARAM_BASE + 3U))) {
            MDD_ProcessParamFrame(id, data, dlc);
        } else if (id == CAN_ID_MODE) {
            MDD_ProcessModeFrame(data, dlc);
        }
    } else {
        if ((id == CAN_ID_TARGET) && (dlc == 8U)) {
            MDD_ProcessTargetFrame(data, dlc);
        }
    }
}

static void MDD_UpdateErrorFlags(uint32_t now)
{
    g_error_code &= (uint8_t)~ERR_INIT_TIMEOUT;
    g_error_code &= (uint8_t)~ERR_CAN_RX_TIMEOUT;

    if ((g_mode == MDD_MODE_PARAM_SET) && (MDD_AllParamsReceived() == 0U) && ((now - boot_tick) >= INIT_WAIT_TIMEOUT_MS)) {
        g_error_code |= ERR_INIT_TIMEOUT;
    }

    if ((now - last_can_rx_tick) >= CAN_RX_TIMEOUT_MS) {
        g_error_code |= ERR_CAN_RX_TIMEOUT;
    }
}

static void MDD_RunControlStep(void)
{
    if (g_mode != MDD_MODE_CONTROL) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            MotorDriver_setSpeed(&g_motor[i], 0);
        }
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        double feedback = 0.0;

        if (g_ctrl_mode[i] == MDD_CTRL_SPEED) {
            feedback = g_encoder_data[i].rps;
        } else {
            feedback = g_encoder_data[i].deg;
        }

        double out = Pid_control(&g_pid[i], g_target_val[i], feedback, CONTROL_PERIOD_MS);
        out *= (double)g_output_dir[i];
        MotorDriver_setSpeed(&g_motor[i], MDD_ClampPwm(out));
    }
}

static void MDD_SendStatus(uint32_t now) {
    if ((now - last_status_tick) < STATUS_PERIOD_MS) {
        return;
    }

    last_status_tick = now;

    uint8_t tx_data[8] = {0};
    MDD_UpdateLimitSwDebounce();

    tx_data[0] = g_limit_sw_state[0];
    tx_data[1] = g_limit_sw_state[1];
    tx_data[2] = g_limit_sw_state[2];
    tx_data[3] = g_limit_sw_state[3];
    tx_data[4] = g_error_code;
    tx_data[5] = (uint8_t)g_mode;

    if (Can_Transmit(&hcan1, CAN_ID_STATUS, tx_data, 8U) != HAL_OK) {
        g_error_code |= ERR_CAN_TX_FAIL;
    } else {
        g_error_code &= (uint8_t)~ERR_CAN_TX_FAIL;
    }
}

void MDD_Init(void) {
    g_mode = MDD_MODE_PARAM_SET;
    g_error_code = ERR_NORMAL;

#ifdef LED_Pin
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#endif

    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_ctrl_mode[i] = MDD_CTRL_SPEED;
        g_target_val[i] = 0.0;
        g_output_dir[i] = 1;
        g_param_received[i] = 0U;
        g_limit_sw_state[i] = 0U;
        g_limit_sw_diff_count[i] = 0U;

        Pid_Init(&g_pid[i]);
        Pid_setGainWithLimit(&g_pid[i], 0.0, 0.0, 0.0, 0.01, 100.0);
    }

    MotorDriver_Init(&g_motor[0], &htim12, TIM_CHANNEL_1, &htim12, TIM_CHANNEL_2);
    MotorDriver_Init(&g_motor[1], &htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2);
    MotorDriver_Init(&g_motor[2], &htim14, TIM_CHANNEL_1, &htim13, TIM_CHANNEL_1);
    MotorDriver_Init(&g_motor[3], &htim11, TIM_CHANNEL_1, &htim10, TIM_CHANNEL_1);

    Encoder_Init(&g_encoder[0], &htim5, 100.0, ENCODER_PPR, ENCODER_PERIOD_MS);
    Encoder_Init(&g_encoder[1], &htim4, 100.0, ENCODER_PPR, ENCODER_PERIOD_MS);
    Encoder_Init(&g_encoder[2], &htim3, 100.0, ENCODER_PPR, ENCODER_PERIOD_MS);
    Encoder_Init(&g_encoder[3], &htim2, 100.0, ENCODER_PPR, ENCODER_PERIOD_MS);

    uint32_t now = HAL_GetTick();
    boot_tick = now;
    last_can_rx_tick = now;
    last_control_tick = now;
    last_status_tick = now;
}

void MDD_Update(void) {
    uint32_t now = HAL_GetTick();
    MDD_ProcessCan();
    MDD_UpdateErrorFlags(now);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        Encoder_Interrupt(&g_encoder[i], &g_encoder_data[i]);
    }

    if ((now - last_control_tick) >= CONTROL_PERIOD_MS) {
        last_control_tick = now;
        MDD_RunControlStep();
    }

    MDD_SendStatus(now);
}
