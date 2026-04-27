/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Altair_library_for_CubeIDE/can_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SV_COUNT 12U
#define CAN_SV_CMD_STD_ID 0x300U
#define CAN_RX_TIMEOUT_MS 200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

USART_HandleTypeDef husart2;
USART_HandleTypeDef husart3;

/* USER CODE BEGIN PV */
static volatile uint8_t g_sv_target_state[SV_COUNT] = {0};
static volatile uint32_t g_last_can_rx_tick = 0U;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} SV_PinMappping_t;

static const SV_PinMappping_t SV_PINS[SV_COUNT] = {
    {GPIOA, GPIO_PIN_0},  /* A0 */
    {GPIOA, GPIO_PIN_1},  /* A1 */
    {GPIOA, GPIO_PIN_6},  /* A6 */
    {GPIOA, GPIO_PIN_7},  /* A7 */
    {GPIOA, GPIO_PIN_8},  /* A8 */
    {GPIOA, GPIO_PIN_9},  /* A9 */
    {GPIOA, GPIO_PIN_15}, /* A15 */
    {GPIOB, GPIO_PIN_3},  /* B3 */
    {GPIOB, GPIO_PIN_6},  /* B6 */
    {GPIOB, GPIO_PIN_7},  /* B7 */
    {GPIOB, GPIO_PIN_8},  /* B8 */
    {GPIOB, GPIO_PIN_9}   /* B9 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);
/* USER CODE BEGIN PFP */
static void SV_ApplyTargets(void);
static void SV_ProcessCanCommand(void);
static uint8_t SV_IsAcceptedCanId(uint32_t std_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_Init();
  MX_USART3_Init();
  /* USER CODE BEGIN 2 */
  /* AltairライブラリのCAN初期化（フィルタ設定 + 受信割り込み開始） */
  CanInitConfig can1_config = Can_DefaultInitConfig(&hcan1);
  if (Can_Init(&hcan1, &can1_config) != HAL_OK) {
    Error_Handler();
  }

  /* CAN2も初期化（どちらのポートに接続されてもメッセージを受信できるようにする）
   */
  CanInitConfig can2_config = Can_DefaultInitConfig(&hcan2);
  if (Can_Init(&hcan2, &can2_config) != HAL_OK) {
    Error_Handler();
  }

  SV_ApplyTargets();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    SV_ProcessCanCommand();

    if ((HAL_GetTick() - g_last_can_rx_tick) > CAN_RX_TIMEOUT_MS) {
      HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

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
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
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
static void MX_CAN2_Init(void) {

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
  if (HAL_CAN_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_Init(void) {

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
  if (HAL_USART_Init(&husart2) != HAL_OK) {
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
static void MX_USART3_Init(void) {

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
  if (HAL_USART_Init(&husart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_0 | GPIO_PIN_1 | LED_Pin | GPIO_PIN_6 |
                        GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOB, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
      GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LED_Pin PA6
                           PA7 PA8 PA9 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | LED_Pin | GPIO_PIN_6 |
                        GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SV_ProcessCanCommand(void) {
  if (g_can1_rx_data.new_data_flag) {
    g_can1_rx_data.new_data_flag = 0U;
    
    // 受信割り込みが来たこと自体を確認するためのデバッグトグル
    HAL_GPIO_TogglePin(GPIOA, LED_Pin);

    if (g_can1_rx_data.dlc >= 2 && SV_IsAcceptedCanId(g_can1_rx_data.std_id)) {
      g_last_can_rx_tick = HAL_GetTick();

      uint16_t state16 = g_can1_rx_data.data[0] | (g_can1_rx_data.data[1] << 8);

      for (uint8_t i = 0; i < SV_COUNT; i++) {
        g_sv_target_state[i] = (state16 & (1 << i)) ? 1 : 0;
      }

      SV_ApplyTargets();
    }
  }

  if (g_can2_rx_data.new_data_flag) {
    g_can2_rx_data.new_data_flag = 0U;

    // 受信割り込みが来たこと自体を確認するためのデバッグトグル
    HAL_GPIO_TogglePin(GPIOA, LED_Pin);

    if (g_can2_rx_data.dlc >= 2 && SV_IsAcceptedCanId(g_can2_rx_data.std_id)) {
      g_last_can_rx_tick = HAL_GetTick();

      uint16_t state16 = g_can2_rx_data.data[0] | (g_can2_rx_data.data[1] << 8);

      for (uint8_t i = 0; i < SV_COUNT; i++) {
        g_sv_target_state[i] = (state16 & (1 << i)) ? 1 : 0;
      }

      SV_ApplyTargets();
    }
  }
}

static void SV_ApplyTargets(void) {
  for (uint8_t i = 0; i < SV_COUNT; i++) {
    GPIO_PinState pin_state =
        (g_sv_target_state[i] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(SV_PINS[i].port, SV_PINS[i].pin, pin_state);
  }
}

static uint8_t SV_IsAcceptedCanId(uint32_t std_id) {
  if (std_id == CAN_SV_CMD_STD_ID)
    return 1;
  return 0;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
