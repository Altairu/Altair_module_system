# Altair_module_system

## MDD

回路名: Altair_MDD_V3

モータドライバードライバモジュール。CAN1でROS2からパラメータ/目標値を受信し、STM32F446でエンコーダフィードバック付きPID制御を実行します。

### 概要

| 項目 | 内容 |
|---|---|
| MCUプロジェクト | MDD |
| 対象MCU | STM32F446 |
| モータ出力数 | 4ch |
| 制御モード | パラメータ設定モード / 制御実行モード |
| 使用CAN | CAN1のみ |
| 状態LED | PA5 |

### ピン割り当て

| 区分 | 信号 | ピン / タイマ |
|---|---|---|
| LED | STATUS LED | PA5 |
| Encoder | Encoder1 | PA0 / PA1 (TIM5) |
| Encoder | Encoder2 | PB6 / PB7 (TIM4) |
| Encoder | Encoder3 | PC6 / PC7 (TIM3) |
| Encoder | Encoder4 | PB3 / PA15 (TIM2) |
| Motor | Motor1 | PB14 (TIM12 CH1), PB15 (TIM12 CH2) |
| Motor | Motor2 | PA8 (TIM1 CH1), PA9 (TIM1 CH2) |
| Motor | Motor3 | PA6 (TIM13 CH1), PA7 (TIM14 CH1) |
| Motor | Motor4 | PB8 (TIM10 CH1), PB9 (TIM11 CH1) |
| Limit SW | SW1 | PC0 |
| Limit SW | SW2 | PC1 |
| Limit SW | SW3 | PC2 |
| Limit SW | SW4 | PC3 |
| Serial | USART2 | TX=PA2, RX=PA3 |
| Serial | USART3 | TX=PB10, RX=PC5 |
| CAN | CAN1 | TX=PA12, RX=PA11 |
| CAN | CAN2 | TX=PB13, RX=PB12 (本仕様では未使用) |

### メインループ処理フロー

1. 初期化
   HAL/CubeMX初期化後、Altair_library_for_CubeIDEを用いてMotorDriver/Encoder/CAN1を初期化し、パラメータ設定モードで起動します。
2. パラメータ設定モード
   CAN ID 0x200, 0x201, 0x203, 0x204 の各パラメータを待機し、4モータ分がそろえば制御実行モードへ遷移します。
3. タイムアウト時の遷移
   起動後2秒以内に全パラメータがそろわない場合は、デフォルトゲインで制御実行モードへ移行し、INIT_TIMEOUTをセットします。
4. 制御実行モード
   1ms周期でエンコーダ差分から速度/角度を更新し、モード(速度/角度)に応じてPID演算してPWMへ反映します。
5. ステータス返信
   10ms周期でリミット状態とモータ系ステータスをCAN1へ送信します。
6. LED制御
   エラーなしでON、エラーありでOFFにします。

### CAN通信仕様 (すべて CAN1, 1Mbps)

#### A. パラメータ設定 (ROS2 -> MDD) [提案3: マルチプレクス化]

CAN ID: **0x200** (マルチプレクス方式)

Payload: 7B
- Byte0: **モータインデックス** (0x00-0x03)
- Byte1: Pゲイン [int8] （100倍スケール）
- Byte2: Iゲイン [int8] （100倍スケール）
- Byte3: Dゲイン [int8] （100倍スケール）
- Byte4-5: 車輪径/出力方向 [int16_t LE, mm単位]
- Byte6: 設定フラグ（将来用、0x00）

**変更点:**
- 旧: 0x200-0x203 の4メッセージ → 新: 0x200 の1メッセージ（モータインデックス付き）
- バス上のトラフィック削減
- 送信側で 0-3ms, 25-28ms, 50-53ms, 75-78ms などでローテーション送信

**例:**
```
M0パラメータ送信: ID=0x200, Data=[0x00, 0x50, 0x00, 0x02, 0x41, 0x00, 0x00] 
  -> M0: P=0.50, I=0.00, D=0.02, wheel=65mm
M1パラメータ送信: ID=0x200, Data=[0x01, 0x50, 0x00, 0x02, 0x41, 0x00, 0x00]
  -> M1: P=0.50, I=0.00, D=0.02, wheel=65mm
```

#### B. 目標値・モード指令 (ROS2 -> MDD) [提案1: 統合化]

CAN ID: **0x210** (統合フレーム)

Payload: 8B (little-endian int16)
- Byte0-1: M1 目標値(15bit) + MSB(**モードフラグ**) [int16_t LE]
- Byte2-3: M2 目標値(15bit) + MSB(**モードフラグ**) [int16_t LE]
- Byte4-5: M3 目標値(15bit) + MSB(**モードフラグ**) [int16_t LE]
- Byte6-7: M4 目標値(15bit) + MSB(**モードフラグ**) [int16_t LE]

**モードフラグ (MSB):**
- MSB = 0: 速度制御 → 目標値 = 目標速度[rps] × 10
- MSB = 1: 角度制御 → 目標値 = 目標角度[deg] × 10

**エンコーディング:**
```
例1) M0=速度100 rps (100×10=1000)
  value_s16 = 1000  →  LSB=0, Payload[0-1] = 0xE8 0x03

例2) M1=角度90 deg (90×10=900, モード=角度)
  value_s16 = -900  →  MSB=1, Payload[2-3] = 0x84 0xFC
```

**変更点:**
- 旧: 0x210 (目標値) + 0x211 (モード) の2メッセージ → 新: 0x210 の1メッセージ
- ID 0x211 削除
- バス上のトラフィック 50% 削減

#### C. ステータス返信 (MDD -> ROS2) [提案2: 統合化]

CAN ID: **0x120** (統合ステータス)

Payload: 8B
- **Byte0:** [Bit 0-3] リミットSW状態(M1-M4) + [Bit 4] システム状態 + [Bit 5-7] エラーコード
  ```
  Bit 0-3: Limit Switch (1bit each, 1=ON)
    Bit0: Limit SW1
    Bit1: Limit SW2
    Bit2: Limit SW3
    Bit3: Limit SW4
  Bit 4: System State (0=パラメータ待機, 1=制御実行中)
  Bit 5-7: Error Code (3bit, ex. 0-7)
  ```
- Byte1: M1 現在モード (0=速度, 1=角度)
- Byte2: M2 現在モード
- Byte3: M3 現在モード
- Byte4: M4 現在モード
- Byte5-7: 予備（将来用、例：電流フィードバックなど）

**変更点:**
- 旧: 0x120 (リミット4byte) + 0x121 (モード4byte + エラー + 状態) の2メッセージ
  → 新: 0x120 の1メッセージ（Byte0で圧縮）
- ID 0x121 削除
- バス上のトラフィック 50% 削減
- リーザーバイト拡張で将来の機能追加に対応

### エラーコード

| 値 | 名称 | 内容 |
|---|---|---|
| 0x00 | NORMAL | 正常 |
| 0x01 | INIT_TIMEOUT | 起動時に必要パラメータ未受信のままタイムアウト |
| 0x02 | CAN_RX_TIMEOUT | 一定時間CAN受信なし |
| 0x04 | CAN_TX_FAIL | フィードバック送信失敗 |

注記:
- エラーコードはビットフラグで、同時に複数立つ場合があります。

## Servo

回路名: ALTAIR_SERVO_MODULE_V6

サーボモーター用モジュール。ROS2 PC から USB to CAN を介して目標角度を送信し、STM32 が 6ch のサーボ PWM を生成する。

### 概要

| 項目 | 内容 |
|---|---|
| MCUプロジェクト | Servo |
| 対象MCU | STM32F446 |
| サーボ出力数 | 6ch |
| 使用CAN | CAN1 (受信) |
| 搭載CANポート | 2系統 |
| 状態LED | PA5 |

### サーボ出力ピン

PIN設定
![alt text](images/image.png)

| サーボ | ピン | タイマ |
|---|---|---|
| Servo1 | PA6 | TIM3 CH1 |
| Servo2 | PA7 | TIM3 CH2 |
| Servo3 | PA8 | TIM1 CH1 |
| Servo4 | PA9 | TIM1 CH2 |
| Servo5 | PB8 | TIM2 CH1 |
| Servo6 | PB9 | TIM2 CH2 |

### PWM制御仕様

| 項目 | 値 |
|---|---|
| 制御周期 | 20ms (50Hz) |
| パルス幅範囲 | 0.5ms から 2.5ms |
| 目標角度 | 0 から 180 [deg] |
| 角度-パルス幅変換 | pulse_us = 500 + (2000 * angle_deg / 180) |

### CAN受信仕様 (ROS2 -> MCU)

| 項目 | 値 |
|---|---|
| 使用CAN | CAN1 |
| CAN ID | 100 (標準ID, dec) または 0x100 (hex) |
| DLC | 6 |

Payload (6B)
- Byte0: Servo1角度 [0..180]
- Byte1: Servo2角度 [0..180]
- Byte2: Servo3角度 [0..180]
- Byte3: Servo4角度 [0..180]
- Byte4: Servo5角度 [0..180]
- Byte5: Servo6角度 [0..180]

注記:
- Byte値が180を超える場合はMCU側で180にクリップします。
- 受信ノイズ対策として、各軸にデッドバンド2degを適用します。
- 同一候補値が3回連続したときのみ目標値へ反映します。

### 通信時の動作

- 任意CANフレーム受信時: LED(PA5) をON
- 200ms 以上CAN無受信時: LED(PA5) をOFF
- 通信が途切れた場合: 最後に反映済みの目標角度を保持して出力を継続
- 起動直後: 初期角度(全軸90deg)でPWMを開始

### 実装メモ (Servo/Core/Src/main.c)

- Altairライブラリ(can_lib.c)でCAN FIFO0受信割り込みを処理し、Can_ReadRxData()で安全に取り出し
- 受信IDが 100(dec) または 0x100(hex)、かつ DLC>=6 のときに角度データとして処理
- デッドバンド/安定化条件を満たした場合のみ6系統PWM比較値を更新
- TIM1/TIM2/TIM3を1MHzカウンタ化 (Prescaler=83)、Period=19999で50Hz PWMを生成
