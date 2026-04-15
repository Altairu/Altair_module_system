# Altair_module_system

## Servo

回路名: ALTAIR_SERVO_MODULE_V6

サーボモーター用モジュール。ROS2 PC から USB to CAN を介して目標角度を送信し、STM32 が 6ch のサーボ PWM を生成する。

### モジュール仕様

- MCUプロジェクト: Servo
- 対象MCU: STM32F446
- サーボ出力数: 6ch
- CANポート: 2系統搭載 (制御受信は CAN1 を使用)
- 状態LED: PA5

### サーボ出力ピン割り当て

PIN設定
![alt text](images/image.png)

- Servo1: PA6 (TIM3 CH1)
- Servo2: PA7 (TIM3 CH2)
- Servo3: PA8 (TIM1 CH1)
- Servo4: PA9 (TIM1 CH2)
- Servo5: PB8 (TIM2 CH1)
- Servo6: PB9 (TIM2 CH2)

### PWM制御仕様

- 制御周期: 20ms (50Hz)
- パルス幅範囲: 0.5ms から 2.4ms
- 目標角度: 0 から 180 [deg]
- 角度とパルス幅の線形変換: pulse_us = 500 + (1900 * angle_deg / 180)

### CAN受信仕様 (ROS2 -> MCU)

- 使用CAN: CAN1
- デフォルトCAN ID: 100 (標準ID)
- DLC: 6
- ペイロード構造 Byte0: Servo1角度 [0..180]
- ペイロード構造 Byte1: Servo2角度 [0..180]
- ペイロード構造 Byte2: Servo3角度 [0..180]
- ペイロード構造 Byte3: Servo4角度 [0..180]
- ペイロード構造 Byte4: Servo5角度 [0..180]
- ペイロード構造 Byte5: Servo6角度 [0..180]
- Byte値が 180 を超える場合は MCU 側で 180 にクリップ

### 通信時の動作

- CAN受信時: LED(PA5) を ON
- 通信が途切れた場合: 最後に受信した目標角度を保持して出力を継続

### main.c実装メモ (Servo/Core/Src/main.c)

- CAN1 フィルタを CAN ID 100 に設定
- CAN FIFO0 受信割り込みで 6バイトを取り込み
- 受信データを 6系統PWM比較値へ反映
- TIM1/TIM2/TIM3 を 1MHz カウンタ化 (Prescaler=83) し、Period=19999 で 50Hz PWM を生成

