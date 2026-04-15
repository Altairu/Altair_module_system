#include "main.h"
#include "Altair_library_for_CubeIDE/altair.h"

/* ===== アプリケーション基本設定 ===== */
/* モータ台数（Altair_MDD_V3は4モータ構成） */
#define MOTOR_COUNT 4
/* 制御演算周期（PID実行周期） */
#define CONTROL_PERIOD_MS 1U
/* ステータスCAN送信周期 */
#define STATUS_PERIOD_MS 10U
/* 起動時にパラメータ受信を待つ最大時間 */
#define INIT_WAIT_TIMEOUT_MS 2000U
/* CAN受信タイムアウト判定時間 */
#define CAN_RX_TIMEOUT_MS 1000U

/* ===== ROS2 -> MDD 受信ID ===== */
#define CAN_ID_PARAM_M1 0x200U
#define CAN_ID_PARAM_M2 0x201U
#define CAN_ID_PARAM_M3 0x203U
#define CAN_ID_PARAM_M4 0x204U
#define CAN_ID_TARGET 0x210U
#define CAN_ID_MODE 0x211U

/* ===== MDD -> ROS2 送信ID ===== */
#define CAN_ID_STATUS_12 0x120U
#define CAN_ID_STATUS_34 0x121U

/* ===== エラーコード（ビットフラグ） ===== */
#define ERR_NONE 0x00U
#define ERR_INIT_TIMEOUT 0x01U
#define ERR_CAN_RX_TIMEOUT 0x02U
#define ERR_CAN_TX_FAIL 0x04U

/* ===== デフォルト制御パラメータ ===== */
#define DEFAULT_P_GAIN 0.80
#define DEFAULT_I_GAIN 0.00
#define DEFAULT_D_GAIN 0.02
/* encoderライブラリでは直径をmmで扱う */
#define DEFAULT_WHEEL_DIAMETER_MM 65.0
#define DEFAULT_ENCODER_PPR 2048

/* システム全体の状態遷移 */
typedef enum {
  APP_MODE_PARAMETER = 0,
  APP_MODE_CONTROL = 1
} AppMode;

/* モータごとの制御モード */
typedef enum {
  CONTROL_SPEED = 0,
  CONTROL_ANGLE = 1
} ControlMode;

/*
 * 1モータ分の制御データをまとめた構造体
 * - motor: PWM出力操作
 * - encoder/encoder_data: 現在値推定
 * - pid: 制御器状態
 * - target_*: 目標値（モードで使い分け）
 */
typedef struct {
  MotorDriver motor;
  Encoder encoder;
  EncoderData encoder_data;
  Pid pid;
  int32_t total_count;
  int16_t target_raw;
  double target_speed_rps;
  double target_angle_deg;
  double wheel_diameter_mm;
  ControlMode mode;
  uint8_t param_received;
} MotorControl;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

USART_HandleTypeDef husart2;
USART_HandleTypeDef husart3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);

/*
 * グローバル状態
 * - g_app_mode: パラメータ待機中か制御実行中か
 * - g_error_code: 異常状態（ビットフラグ）
 * - g_last_*: 周期処理の時刻管理
 */
static AppMode g_app_mode = APP_MODE_PARAMETER;
static uint8_t g_error_code = ERR_NONE;
static uint32_t g_boot_tick = 0U;
static uint32_t g_last_control_tick = 0U;
static uint32_t g_last_status_tick = 0U;
static uint32_t g_last_can_rx_tick = 0U;

static MotorControl g_motors[MOTOR_COUNT];

static int16_t read_i16_le(const uint8_t *buf);
static int clamp_int(int value, int min_value, int max_value);
static void update_led_state(void);
static uint8_t get_limit_state(uint8_t index);
static void app_init_defaults(void);
static void app_init_modules(void);
static void process_can_message(void);
static void handle_param_frame(uint8_t motor_index, const uint8_t *data, uint8_t dlc);
static void handle_target_frame(const uint8_t *data, uint8_t dlc);
static void handle_mode_frame(const uint8_t *data, uint8_t dlc);
static uint8_t are_all_params_received(void);
static void app_control_step(void);
static void app_send_feedback(void);

/* little-endianの2byteを符号付き16bitとして読む */
static int16_t read_i16_le(const uint8_t *buf)
{
  return (int16_t)(((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8));
}

/* 値を[min, max]に丸める（モータ指令の安全制限に利用） */
static int clamp_int(int value, int min_value, int max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void update_led_state(void)
{
  /* エラーなしで点灯、エラーありで消灯 */
  GPIO_PinState led_state = (g_error_code == ERR_NONE) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state);
}

static uint8_t get_limit_state(uint8_t index)
{
  uint16_t pin;

  /* index 0..3 を PC0..PC3 に対応付け */
  switch (index)
  {
    case 0: pin = GPIO_PIN_0; break;
    case 1: pin = GPIO_PIN_1; break;
    case 2: pin = GPIO_PIN_2; break;
    default: pin = GPIO_PIN_3; break;
  }

  return (HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void app_init_defaults(void)
{
  uint8_t i;

  /* 全モータの制御状態を既定値で初期化 */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    g_motors[i].total_count = 0;
    g_motors[i].target_raw = 0;
    g_motors[i].target_speed_rps = 0.0;
    g_motors[i].target_angle_deg = 0.0;
    g_motors[i].wheel_diameter_mm = DEFAULT_WHEEL_DIAMETER_MM;
    g_motors[i].mode = CONTROL_SPEED;
    g_motors[i].param_received = 0U;

    /* フィードバック値（算出値）の初期化 */
    g_motors[i].encoder_data.count = 0;
    g_motors[i].encoder_data.rot = 0.0;
    g_motors[i].encoder_data.deg = 0.0;
    g_motors[i].encoder_data.distance = 0.0;
    g_motors[i].encoder_data.velocity = 0.0;
    g_motors[i].encoder_data.rps = 0.0;

    /* PIDはゼロクリアしてから既定ゲインを設定 */
    Pid_Init(&g_motors[i].pid);
    Pid_setGainWithLimit(&g_motors[i].pid, DEFAULT_P_GAIN, DEFAULT_I_GAIN, DEFAULT_D_GAIN, 0.0, 100.0);
    Pid_setInvert(&g_motors[i].pid, 1);
  }
}

static void app_init_modules(void)
{
  /* まずソフトウェア状態を初期化 */
  app_init_defaults();

  /* モータドライバ接続（回路配線に対応） */
  MotorDriver_Init(&g_motors[0].motor, &htim12, TIM_CHANNEL_1, &htim12, TIM_CHANNEL_2);
  MotorDriver_Init(&g_motors[1].motor, &htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2);
  MotorDriver_Init(&g_motors[2].motor, &htim13, TIM_CHANNEL_1, &htim14, TIM_CHANNEL_1);
  MotorDriver_Init(&g_motors[3].motor, &htim10, TIM_CHANNEL_1, &htim11, TIM_CHANNEL_1);

  /* エンコーダ接続（タイマ対応） */
  Encoder_Init(&g_motors[0].encoder, &htim5, g_motors[0].wheel_diameter_mm, DEFAULT_ENCODER_PPR, CONTROL_PERIOD_MS);
  Encoder_Init(&g_motors[1].encoder, &htim4, g_motors[1].wheel_diameter_mm, DEFAULT_ENCODER_PPR, CONTROL_PERIOD_MS);
  Encoder_Init(&g_motors[2].encoder, &htim3, g_motors[2].wheel_diameter_mm, DEFAULT_ENCODER_PPR, CONTROL_PERIOD_MS);
  Encoder_Init(&g_motors[3].encoder, &htim2, g_motors[3].wheel_diameter_mm, DEFAULT_ENCODER_PPR, CONTROL_PERIOD_MS);

  /* CAN1開始（受信割り込み有効化はCan_Init内部で実施） */
  if (Can_Init(&hcan1) != HAL_OK)
  {
    /* 初期化失敗時は受信系エラーとして扱う */
    g_error_code |= ERR_CAN_RX_TIMEOUT;
  }

  /* 周期処理の基準時刻をそろえる */
  g_boot_tick = HAL_GetTick();
  g_last_control_tick = g_boot_tick;
  g_last_status_tick = g_boot_tick;
  g_last_can_rx_tick = g_boot_tick;
}

static void handle_param_frame(uint8_t motor_index, const uint8_t *data, uint8_t dlc)
{
  int16_t p_raw;
  int16_t i_raw;
  int16_t d_raw;
  int16_t wheel_dir_raw;
  double wheel_diameter;

  if (motor_index >= MOTOR_COUNT || dlc < 8U)
  {
    /* 不正フレームは破棄 */
    return;
  }

  /* 8byteを [P,I,D,車輪径/方向] として展開 */
  p_raw = read_i16_le(&data[0]);
  i_raw = read_i16_le(&data[2]);
  d_raw = read_i16_le(&data[4]);
  wheel_dir_raw = read_i16_le(&data[6]);

  /* ゲインはx100スケールで受信して実数へ変換 */
  Pid_setGainWithLimit(&g_motors[motor_index].pid,
                       (double)p_raw / 100.0,
                       (double)i_raw / 100.0,
                       (double)d_raw / 100.0,
                       0.0,
                       200.0);

  /*
   * wheel_dir_rawの符号で出力方向を指定
   * 絶対値は車輪径[mm]
   */
  if (wheel_dir_raw < 0)
  {
    Pid_setInvert(&g_motors[motor_index].pid, -1);
    wheel_diameter = (double)(-wheel_dir_raw);
  }
  else
  {
    Pid_setInvert(&g_motors[motor_index].pid, 1);
    wheel_diameter = (double)wheel_dir_raw;
  }

  if (wheel_diameter > 0.0)
  {
    /* 実行時に車輪径を更新（速度/距離換算に反映） */
    g_motors[motor_index].wheel_diameter_mm = wheel_diameter;
    g_motors[motor_index].encoder.diameter = wheel_diameter;
  }

  /* このモータの初期パラメータ受信完了 */
  g_motors[motor_index].param_received = 1U;
}

static void handle_target_frame(const uint8_t *data, uint8_t dlc)
{
  uint8_t i;
  int16_t raw_target;

  if (dlc < 8U)
  {
    return;
  }

  /* 各モータ2byteずつの目標値（x10）を読み込む */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    raw_target = read_i16_le(&data[i * 2U]);
    g_motors[i].target_raw = raw_target;
    g_motors[i].target_speed_rps = (double)raw_target / 10.0;
    g_motors[i].target_angle_deg = (double)raw_target / 10.0;
  }
}

static void handle_mode_frame(const uint8_t *data, uint8_t dlc)
{
  uint8_t i;
  int16_t raw_mode;

  if (dlc < 8U)
  {
    return;
  }

  /* 0:速度制御, 1:角度制御（それ以外は速度扱い） */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    raw_mode = read_i16_le(&data[i * 2U]);
    if (raw_mode == 1)
    {
      g_motors[i].mode = CONTROL_ANGLE;
    }
    else
    {
      g_motors[i].mode = CONTROL_SPEED;
    }
  }
}

static void process_can_message(void)
{
  CanRxData rx_data;

  /* 新規受信がなければ何もしない */
  if (g_can1_rx_data.new_data_flag == 0U)
  {
    return;
  }

  /*
   * 割り込み側で更新される共有データを安全にコピー
   * new_data_flagをここでクリアして次フレームを受ける
   */
  __disable_irq();
  rx_data = g_can1_rx_data;
  g_can1_rx_data.new_data_flag = 0U;
  __enable_irq();

  /* 受信成功として監視時刻更新、RXタイムアウトフラグ解除 */
  g_last_can_rx_tick = HAL_GetTick();
  g_error_code &= (uint8_t)(~ERR_CAN_RX_TIMEOUT);

  /* IDごとにハンドラを振り分け */
  switch (rx_data.std_id)
  {
    case CAN_ID_PARAM_M1:
      handle_param_frame(0U, rx_data.data, rx_data.dlc);
      break;
    case CAN_ID_PARAM_M2:
      handle_param_frame(1U, rx_data.data, rx_data.dlc);
      break;
    case CAN_ID_PARAM_M3:
      handle_param_frame(2U, rx_data.data, rx_data.dlc);
      break;
    case CAN_ID_PARAM_M4:
      handle_param_frame(3U, rx_data.data, rx_data.dlc);
      break;
    case CAN_ID_TARGET:
      handle_target_frame(rx_data.data, rx_data.dlc);
      break;
    case CAN_ID_MODE:
      handle_mode_frame(rx_data.data, rx_data.dlc);
      break;
    default:
      break;
  }
}

static uint8_t are_all_params_received(void)
{
  uint8_t i;

  /* 起動時に4モータ分パラメータがそろったか判定 */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    if (g_motors[i].param_received == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

static void app_control_step(void)
{
  uint8_t i;
  int16_t pulse_count;
  double now_value;
  double target_value;
  double control_out;
  int speed_cmd;

  /* 4モータ分を同じ周期で更新する */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    /* この1周期で増減したパルス数を取得 */
    pulse_count = (int16_t)Encoder_Read(&g_motors[i].encoder);
    Encoder_Reset(&g_motors[i].encoder);

    /*
     * エンコーダの生パルスから物理量を計算
     * - rot/deg: 累積位置
     * - rps: この周期の速度
     */
    g_motors[i].total_count += pulse_count;
    g_motors[i].encoder_data.count = g_motors[i].total_count;
    g_motors[i].encoder_data.rot = (double)g_motors[i].total_count / (double)g_motors[i].encoder.ppr;
    g_motors[i].encoder_data.deg = g_motors[i].encoder_data.rot * 360.0;
    g_motors[i].encoder_data.distance = g_motors[i].encoder_data.rot * (PI * g_motors[i].wheel_diameter_mm);
    g_motors[i].encoder_data.rps = ((double)pulse_count * 1000.0) / ((double)g_motors[i].encoder.ppr * CONTROL_PERIOD_MS);
    g_motors[i].encoder_data.velocity = g_motors[i].encoder_data.rps * PI * g_motors[i].wheel_diameter_mm;

    /* モードに応じてPIDの入力を切り替える */
    if (g_motors[i].mode == CONTROL_ANGLE)
    {
      now_value = g_motors[i].encoder_data.deg;
      target_value = g_motors[i].target_angle_deg;
    }
    else
    {
      now_value = g_motors[i].encoder_data.rps;
      target_value = g_motors[i].target_speed_rps;
    }

    /* PID出力をモータ指令値へ変換し、過大指令を制限 */
    control_out = Pid_control(&g_motors[i].pid, target_value, now_value, CONTROL_PERIOD_MS);
    speed_cmd = clamp_int((int)control_out, -99, 99);
    MotorDriver_setSpeed(&g_motors[i].motor, speed_cmd);
  }
}

static void app_send_feedback(void)
{
  uint8_t tx_data[8];
  HAL_StatusTypeDef st;

  /* 0x120: リミットスイッチ状態（4byte） */
  tx_data[0] = get_limit_state(0U);
  tx_data[1] = get_limit_state(1U);
  tx_data[2] = get_limit_state(2U);
  tx_data[3] = get_limit_state(3U);
  st = Can_Transmit(&hcan1, CAN_ID_STATUS_12, tx_data, 4U);

  /* 0x121: モード4byte + エラー1byte + システム状態1byte */
  tx_data[0] = (uint8_t)g_motors[0].mode;
  tx_data[1] = (uint8_t)g_motors[1].mode;
  tx_data[2] = (uint8_t)g_motors[2].mode;
  tx_data[3] = (uint8_t)g_motors[3].mode;
  tx_data[4] = g_error_code;
  tx_data[5] = (uint8_t)g_app_mode;
  if (Can_Transmit(&hcan1, CAN_ID_STATUS_34, tx_data, 6U) != HAL_OK)
  {
    st = HAL_ERROR;
  }

  /* 送信成否をエラーコードへ反映 */
  if (st == HAL_OK)
  {
    g_error_code &= (uint8_t)(~ERR_CAN_TX_FAIL);
  }
  else
  {
    g_error_code |= ERR_CAN_TX_FAIL;
  }
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_Init();
  MX_USART3_Init();

  /* 手書きアプリ層の初期化 */
  app_init_modules();

  while (1)
  {
    uint32_t now;

    /* 受信済みCANフレームを処理 */
    process_can_message();

    now = HAL_GetTick();

    /* 一定時間受信がない場合は通信異常とする */
    if ((uint32_t)(now - g_last_can_rx_tick) > CAN_RX_TIMEOUT_MS)
    {
      g_error_code |= ERR_CAN_RX_TIMEOUT;
    }

    /*
     * 起動直後はパラメータ待機モード
     * - 4モータ分そろえば制御開始
     * - タイムアウトならデフォルト値で制御開始
     */
    if (g_app_mode == APP_MODE_PARAMETER)
    {
      if (are_all_params_received() != 0U)
      {
        g_app_mode = APP_MODE_CONTROL;
        g_error_code &= (uint8_t)(~ERR_INIT_TIMEOUT);
      }
      else if ((uint32_t)(now - g_boot_tick) > INIT_WAIT_TIMEOUT_MS)
      {
        g_app_mode = APP_MODE_CONTROL;
        g_error_code |= ERR_INIT_TIMEOUT;
      }
    }

    /* 制御実行モード時のみ1ms周期でPIDを回す */
    if ((g_app_mode == APP_MODE_CONTROL) && ((uint32_t)(now - g_last_control_tick) >= CONTROL_PERIOD_MS))
    {
      g_last_control_tick += CONTROL_PERIOD_MS;
      app_control_step();
    }

    /* 10ms周期でROS2へ状態通知 */
    if ((uint32_t)(now - g_last_status_tick) >= STATUS_PERIOD_MS)
    {
      g_last_status_tick += STATUS_PERIOD_MS;
      app_send_feedback();
    }

    /* LEDは常に最新エラー状態を反映 */
    update_led_state();

  }
  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
