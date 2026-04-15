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

### CAN通信仕様 (すべて CAN1)

#### A. パラメータ設定 (ROS2 -> MDD)

CAN ID:
- Motor1: 0x200
- Motor2: 0x201
- Motor3: 0x203
- Motor4: 0x204

Payload: 8B (little-endian int16)
- Byte0-1: Pゲイン x100
- Byte2-3: Iゲイン x100
- Byte4-5: Dゲイン x100
- Byte6-7: 車輪径/出力方向

Byte6-7の解釈:
- 絶対値: 車輪径[mm]
- 符号: PID出力方向 (正=通常, 負=反転)

例:
- 0x0064 (100) -> 車輪径 100mm, 通常方向
- 0xFF9C (-100) -> 車輪径 100mm, 反転方向

#### B. 目標値・モード指令 (ROS2 -> MDD)

CAN ID: 0x210 (目標値)

Payload: 8B (little-endian int16)
- Byte0-1: M1目標 x10
- Byte2-3: M2目標 x10
- Byte4-5: M3目標 x10
- Byte6-7: M4目標 x10

スケール:
- 速度モード時: 目標速度[rps] x10
- 角度モード時: 目標角度[deg] x10

CAN ID: 0x211 (モード指令)

Payload: 8B (little-endian int16)
- Byte0-1: M1モード
- Byte2-3: M2モード
- Byte4-5: M3モード
- Byte6-7: M4モード

モード値:
- 0: 速度制御
- 1: 角度制御

備考:
- モードは受信後に保持され、次に0x211を受信するまで継続します。

#### C. ステータス返信 (MDD -> ROS2)

CAN ID: 0x120 (リミットスイッチ)

Payload: 4B
- Byte0: Limit SW1
- Byte1: Limit SW2
- Byte2: Limit SW3
- Byte3: Limit SW4

CAN ID: 0x121 (Motor系ステータス)

Payload: 6B
- Byte0: Motor1モード
- Byte1: Motor2モード
- Byte2: Motor3モード
- Byte3: Motor4モード
- Byte4: エラーコード
- Byte5: システム状態 (0=パラメータ設定, 1=制御実行)

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
| パルス幅範囲 | 0.5ms から 2.4ms |
| 目標角度 | 0 から 180 [deg] |
| 角度-パルス幅変換 | pulse_us = 500 + (1900 * angle_deg / 180) |

### CAN受信仕様 (ROS2 -> MCU)

| 項目 | 値 |
|---|---|
| 使用CAN | CAN1 |
| CAN ID | 100 (標準ID) |
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

### 通信時の動作

- CAN受信時: LED(PA5) をON
- 通信が途切れた場合: 最後に受信した目標角度を保持して出力を継続

### 実装メモ (Servo/Core/Src/main.c)

- CAN1フィルタをCAN ID 100に設定
- CAN FIFO0受信割り込みで6バイトを取り込み
- 受信データを6系統PWM比較値へ反映
- TIM1/TIM2/TIM3を1MHzカウンタ化 (Prescaler=83)、Period=19999で50Hz PWMを生成

## altair_gui.py - MDD制御ツール

Windows PC上で実行するPythonベースのGUIツールです。USB-to-CANアダプタを使用してAltair MDD(モータドライバー)を制御します。

現行版は以下に対応しています。

- CANビットレート 1Mbps 固定
- 4モーター個別 PID パラメータ設定
- 4モーター目標値の同時送信
- 0x120 / 0x121 のライブステータス表示

### インストール

#### 前提条件
- Python 3.9以上
- USB-to-CANアダプタ (例: PEAK PCAN-USB, Kvaser等)
- CANドライバが適切にインストールされていること

#### パッケージ インストール

```bash
pip install python-can
```

### 使用方法

#### 1. スクリプト実行

```bash
python altair_gui.py
```

GUI ウィンドウが起動します。

#### 2. CAN接続設定

**Connection タブ:**
- **Interface**: CAN インターフェース (例: `slcan`, `pcan`, `kvaser`)
  - `slcan`: シリアルCAN (デバイスマネージャーで COM ポート確認)
  - `pcan`: PEAK PCAN-USB等
  - `kvaser`: Kvaser製アダプタ
- **Channel/COM**: デバイスポートまたはチャネル (例: `COM3`, `0`)
- **Bitrate**: 1,000,000 bps 固定
- **Connect ボタン**: クリックして CAN バス接続

接続成功時、ステータス表示が `Connected ...` になります。

#### 3. パラメータ設定

**Per-Motor PID Parameters タブ:**
- **M1-M4 それぞれに個別で** P Gain / I Gain / D Gain / Wheel Dia / Invert を設定可能
- **Send M1 ... Send M4**: モーター単体へ送信
- **Copy M1 to All**: M1の設定をM2-M4へ複製
- **Send All Params**: 4モーターへ一括送信

例: P Gain = 1.5 → CAN メッセージに 150 として送信

#### 4. 制御モード & 目標値

**Control & Targets タブ:**
- **モード選択**:
  - Velocity Mode: 速度制御 (目標回転速度 [rps] ×10)
  - Angle/Pos Mode: 角度制御 (目標角度 [deg] ×10)
- **Send Mode ボタン**: 選択したモードを 4 モーター全て に送信
- **M1/M2/M3/M4 Target**: 4モーター目標値 (×10で内部送信)
- **Send Targets ボタン**: 目標値を送信
- **STOP ボタン**: 全モーター停止 (目標値を 0 に設定)

例:
- Velocity Mode で M1 Target = 5.0 → 実際の目標回転速度 0.5 rps

#### 5. ログ表示

下部の **Log** 領域にCAN送受信メッセージが表示されます。

#### 6. ライブステータス表示

**Live Status パネル**で以下を表示します。
- 0x120: Limit SW1-SW4
- 0x121: M1-M4モード、エラーコード、システム状態

### CAN接続のトラブルシューティング

#### "No module named 'can'" エラー

python-can ライブラリがインストールされていません:
```bash
pip install python-can
```

#### "CAN not connected!" エラー

- USB-to-CAN アダプタが接続されているか確認
- Interface と Channel/COM の設定値が正しいか確認
- デバイスマネージャーで COM ポートが認識されているか確認

#### 接続できても送信が失敗する場合

- MCU 側の CAN ビットレート (1Mbps) とPCの設定が一致しているか確認
- MCU の電源が入っているか確認
- CAN トランシーバーの接続状態を確認

### CAN メッセージフォーマット

本ツールが送信するメッセージ:

| 目的 | CAN ID | Payload |
|---|---|---|
| Motor1 パラメータ | 0x200 | P, I, D, 径 (各 int16 little-endian) |
| Motor2 パラメータ | 0x201 | " |
| Motor3 パラメータ | 0x203 | " |
| Motor4 パラメータ | 0x204 | " |
| モード指令 | 0x211 | M1, M2, M3, M4 (各 int16 little-endian) |
| 目標値 | 0x210 | M1, M2, M3, M4 (各 int16 little-endian) |

詳細は本README の「MDD - CAN通信仕様」セクションを参照。

### サンプル手順

1. GUI を起動: `python altair_gui.py`
2. Interface = `slcan`, Channel/COM = `COM3` に設定
3. Connect ボタンをクリック
4. M1-M4 のP/I/D/径を設定 → Send All Params
5. モードを "Velocity Mode" に選択 → Send Mode
6. M1-M4 Target を設定 → Send Targets
7. ログに "TX ID:0x211" "TX ID:0x200" 等が表示されれば成功

## altair_gui_ubuntu.py - Ubuntu向けMDD制御ツール

Ubuntu向けのPython GUIツールです。基本機能はWindows版と同じで、以下に対応しています。

- 4モーター個別PID設定 (M1-M4)
- 4モーター目標値送信
- モード一括送信 (0x211)
- 0x120 / 0x121 ライブステータス表示

### Ubuntuセットアップ

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-tk can-utils
python3 -m pip install --user python-can
```

### SocketCANを1Mbpsで有効化

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

### 起動方法

```bash
python3 altair_gui_ubuntu.py
```

### 使い方

1. Interface を `socketcan`、Channel を `can0` に設定
2. Connect を押す
3. M1-M4 のPIDと車輪径を設定して `Send All Params`
4. Velocity / Angle を選んで `Send Mode`
5. M1-M4 Target を入力して `Send Targets`
6. Live Status で 0x120 / 0x121 を確認

### 補足

- `socketcan`使用時はビットレート設定をOS側 (`ip link`) で行います。
- `slcan`など別インターフェース使用時は、アダプタに合わせて Interface / Channel を変更してください。

