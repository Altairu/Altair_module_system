#include "mdd.h"
#include "Altair_library_for_CubeIDE/altair.h"
#include <stdint.h>

typedef enum { MDD_MODE_PARAM_SET = 0, MDD_MODE_CONTROL = 1 } MDD_Mode_t;

typedef enum {
  MDD_CTRL_SPEED = 0,
  MDD_CTRL_ANGLE = 1,
  MDD_CTRL_POSITION = 2
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

#define MOTOR_COUNT 4

#define ENCODER_PPR 8192
#define ENCODER_PERIOD_MS 1
#define CONTROL_PERIOD_MS 10

#define STATUS_PERIOD_MS 10
#define ENCODER_TX_PERIOD_MS 10  /* エンコーダー送信周期 (モード問わず常時) */
#define INIT_WAIT_TIMEOUT_MS 2000
#define CAN_RX_TIMEOUT_MS 1000

#define CAN_ID_PARAM_BASE 0x200U
#define CAN_ID_MODE 0x210U
#define CAN_ID_TARGET 0x220U
#define CAN_ID_STATUS 0x230U
#define CAN_ID_DEG    0x240U  /* エンコーダ角度送信 (baseId + 0x40) */
#define CAN_ID_RPS    0x250U  /* エンコーダ回転速度送信 (baseId + 0x50) */

/*　CANのID規則
MDD1
#define CAN_ID_PARAM_BASE      0x200U
#define CAN_ID_MODE            0x210U
#define CAN_ID_TARGET          0x220U
#define CAN_ID_STATUS          0x230U
#define CAN_ID_DEG             0x240U
#define CAN_ID_RPS             0x250U

MDD2
#define CAN_ID_PARAM_BASE      0x204U
#define CAN_ID_MODE            0x214U
#define CAN_ID_TARGET          0x224U
#define CAN_ID_STATUS          0x234U
#define CAN_ID_DEG             0x244U
#define CAN_ID_RPS             0x254U

MDD3
#define CAN_ID_PARAM_BASE      0x208U
#define CAN_ID_MODE            0x218U
#define CAN_ID_TARGET          0x228U
#define CAN_ID_STATUS          0x238U
#define CAN_ID_DEG             0x248U
#define CAN_ID_RPS             0x258U

MDD4
#define CAN_ID_PARAM_BASE      0x20CU
#define CAN_ID_MODE            0x21CU
#define CAN_ID_TARGET          0x22CU
#define CAN_ID_STATUS          0x23CU
#define CAN_ID_DEG             0x24CU
#define CAN_ID_RPS             0x25CU

(オフセット規則: DEG = baseId + 0x40, RPS = baseId + 0x50)
*/

#define ERR_NORMAL 0x00U
#define ERR_INIT_TIMEOUT 0x01U
#define ERR_CAN_RX_TIMEOUT 0x02U
#define ERR_CAN_TX_FAIL 0x04U

#define LIMIT_SW_COUNT 4
#define LIMIT_SW_DEBOUNCE_COUNT 3U

static MDD_Mode_t g_mode = MDD_MODE_PARAM_SET;
static MDD_CtrlMode_t g_ctrl_mode[MOTOR_COUNT] = {
    MDD_CTRL_SPEED, MDD_CTRL_SPEED, MDD_CTRL_SPEED, MDD_CTRL_SPEED};

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
static uint32_t last_status_tick = 0U;
static uint32_t last_encoder_tx_tick = 0U;  /* エンコーダー送信タイムスタンプ */

static uint8_t g_limit_sw_state[LIMIT_SW_COUNT] = {0U, 0U, 0U, 0U};
static uint8_t g_limit_sw_diff_count[LIMIT_SW_COUNT] = {0U, 0U, 0U, 0U};

static int MDD_ClampPwm(double in) {
  int v = (int)in;
  if (v > 100) {
    v = 99;
  }
  if (v < -100) {
    v = -99;
  }
  return v;
}

static uint8_t MDD_AllParamsReceived(void) {
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
  case 0:
    pin = GPIO_PIN_0;
    break;
  case 1:
    pin = GPIO_PIN_1;
    break;
  case 2:
    pin = GPIO_PIN_2;
    break;
  default:
    pin = GPIO_PIN_3;
    break;
  }

  return (HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static int16_t MDD_ReadI16Le(const uint8_t *data) {
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

static void MDD_ProcessParamFrame(uint32_t id, const uint8_t *data,
                                  uint8_t dlc) {
  if ((id < CAN_ID_PARAM_BASE) || (id > (CAN_ID_PARAM_BASE + 3U)) ||
      (dlc != 8U)) {
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

static void MDD_ProcessModeFrame(const uint8_t *data, uint8_t dlc) {
  /* USB-CAN(DLC=4)とEthernet-CAN(固定DLC=8)の双方を受け付ける */
  if (dlc < 4U) {
    return;
  }

  if (MDD_AllParamsReceived() == 0U) {
    return;
  }

  uint8_t mode_changed = (g_mode == MDD_MODE_PARAM_SET);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (data[i] == 0U) {
      g_ctrl_mode[i] = MDD_CTRL_SPEED;
    } else if (data[i] == 1U) {
      g_ctrl_mode[i] = MDD_CTRL_ANGLE;
    } else if (data[i] == 2U) {
      g_ctrl_mode[i] = MDD_CTRL_POSITION;
    } else {
      g_ctrl_mode[i] = MDD_CTRL_SPEED;
    }
  }

  if (mode_changed) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      /* エンコーダのハードウェアタイマー値をリセット */
      Encoder_Reset(&g_encoder[i]);
      
      /* エンコーダの蓄積データと計算用変数をリセット */
      g_encoder[i].limit = 0;
      g_encoder[i].before_rot = 0.0;
      g_encoder[i].before_deg = 0.0;
      
      g_encoder_data[i].count = 0;
      g_encoder_data[i].rot = 0.0;
      g_encoder_data[i].deg = 0.0;
      g_encoder_data[i].distance = 0.0;
      g_encoder_data[i].velocity = 0.0;
      g_encoder_data[i].rps = 0.0;

      /* PIDの内部状態(積分項や過去の偏差)をリセット */
      Pid_reset(&g_pid[i]);

      /* 安全のため目標値も0にリセットし、切り替え直後の急発進を防ぐ */
      g_target_val[i] = 0.0;
    }
  }

  g_mode = MDD_MODE_CONTROL;
#ifdef LED_Pin
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
#endif
}

static void MDD_ProcessTargetFrame(const uint8_t *data, uint8_t dlc) {
  if (dlc != 8U) {
    return;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int16_t raw = MDD_ReadI16Le(&data[i * 2]);
    g_target_val[i] = (double)raw / 10.0;
  }
}

static void MDD_ProcessCan(void) {
  CanRxData rx;

  /* 受信キューに溜まった全フレームを順番に処理する */
  while (Can_Receive(&g_can1_rx_queue, &rx) != 0U) {
    last_can_rx_tick = HAL_GetTick();

    uint32_t id = rx.std_id;
    uint8_t dlc = rx.dlc;
    uint8_t *data = rx.data;

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
}

static void MDD_UpdateErrorFlags(uint32_t now) {
  g_error_code &= (uint8_t)~ERR_INIT_TIMEOUT;
  g_error_code &= (uint8_t)~ERR_CAN_RX_TIMEOUT;

  if ((g_mode == MDD_MODE_PARAM_SET) && (MDD_AllParamsReceived() == 0U) &&
      ((now - boot_tick) >= INIT_WAIT_TIMEOUT_MS)) {
    g_error_code |= ERR_INIT_TIMEOUT;
  }

  if ((now - last_can_rx_tick) >= CAN_RX_TIMEOUT_MS) {
    g_error_code |= ERR_CAN_RX_TIMEOUT;
  }
}

static void MDD_RunControlStep(void) {
  if (g_mode != MDD_MODE_CONTROL) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      MotorDriver_setSpeed(&g_motor[i], 0);
      /* PARAM MODE中はPIDの内部状態(I項など)をリセットして、不要な出力を防ぐ */
      Pid_reset(&g_pid[i]);
    }
    return;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    double feedback = 0.0;

    if (g_ctrl_mode[i] == MDD_CTRL_SPEED) {
      feedback = g_encoder_data[i].rps;
    } else if (g_ctrl_mode[i] == MDD_CTRL_ANGLE) {
      feedback = g_encoder_data[i].deg;
    } else if (g_ctrl_mode[i] == MDD_CTRL_POSITION) {
      feedback = g_encoder_data[i].distance;
    }

    double out =
        Pid_control(&g_pid[i], g_target_val[i], feedback, CONTROL_PERIOD_MS);
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

/**
 * @brief エンコーダー角度・回転速度をCANで送信する（モード問わず常時）
 *        DEG: 0x240 - int16_t × 4 (単位: 0.1度、例: 1234 = 123.4度)
 *        RPS: 0x250 - int16_t × 4 (単位: 0.01rps、例: 123 = 1.23rps)
 */
static void MDD_SendEncoderData(uint32_t now) {
  if ((now - last_encoder_tx_tick) < ENCODER_TX_PERIOD_MS) {
    return;
  }

  last_encoder_tx_tick = now;

  uint8_t deg_data[8] = {0};
  uint8_t rps_data[8] = {0};

  for (int i = 0; i < MOTOR_COUNT; i++) {
    /* deg: 0.1度単位に変換してint16_tにクランプ */
    int32_t deg_raw = (int32_t)(g_encoder_data[i].deg * 10.0);
    if (deg_raw > 32767)  deg_raw = 32767;
    if (deg_raw < -32768) deg_raw = -32768;
    int16_t deg_val = (int16_t)deg_raw;
    deg_data[i * 2]     = (uint8_t)((uint16_t)deg_val & 0xFFU);
    deg_data[i * 2 + 1] = (uint8_t)(((uint16_t)deg_val >> 8U) & 0xFFU);

    /* rps: 0.01rps単位に変換してint16_tにクランプ */
    int32_t rps_raw = (int32_t)(g_encoder_data[i].rps * 100.0);
    if (rps_raw > 32767)  rps_raw = 32767;
    if (rps_raw < -32768) rps_raw = -32768;
    int16_t rps_val = (int16_t)rps_raw;
    rps_data[i * 2]     = (uint8_t)((uint16_t)rps_val & 0xFFU);
    rps_data[i * 2 + 1] = (uint8_t)(((uint16_t)rps_val >> 8U) & 0xFFU);
  }

  /* エンコーダ角度を送信 */
  Can_Transmit(&hcan1, CAN_ID_DEG, deg_data, 8U);

  /* エンコーダ回転速度を送信 */
  Can_Transmit(&hcan1, CAN_ID_RPS, rps_data, 8U);
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
  last_encoder_tx_tick = now;  /* エンコーダー送信タイマーを初期化 */
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

  /* エンコーダー値はモードに関わらず常時送信する */
  MDD_SendEncoderData(now);
}
