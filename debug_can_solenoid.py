#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ソレノイドバルブ CAN通信デバッグスクリプト
0x300へ2バイトのデータを送信し、受信も監視する
"""

import time
import sys

try:
    import can
except ImportError:
    print("❌ python-can がインストールされていません")
    print("   pip install python-can")
    sys.exit(1)

def find_available_interface():
    """利用可能なCANインターフェースを検出"""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    print(f"\n[Detected COM ports]:")
    for p in ports:
        print(f"   {p.device}: {p.description} [{p.hwid}]")
    return ports

def test_can_send(interface, channel, bitrate=1000000):
    """CAN送信テスト"""
    print(f"\n[Trying CAN connection]: interface={interface}, channel={channel}, bitrate={bitrate}")
    
    try:
        bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
        print(f"[SUCCESS] CAN connected!")
    except Exception as e:
        print(f"[FAIL] CAN connection failed: {e}")
        return False
    
    print(f"\n--- Test 1: All Valves OFF (0x300) ---")
    try:
        msg = can.Message(arbitration_id=0x300, data=bytes([0x00, 0x00]), is_extended_id=False)
        bus.send(msg)
        print(f"[SUCCESS] Sent: ID=0x{msg.arbitration_id:03X}, Data={list(msg.data)}")
    except Exception as e:
        print(f"[FAIL] Send failed: {e}")
    
    time.sleep(0.5)
    
    print(f"\n--- Test 2: Valve 1 ON ---")
    try:
        msg = can.Message(arbitration_id=0x300, data=bytes([0x01, 0x00]), is_extended_id=False)
        bus.send(msg)
        print(f"[SUCCESS] Sent: ID=0x{msg.arbitration_id:03X}, Data={list(msg.data)}")
    except Exception as e:
        print(f"[FAIL] Send failed: {e}")
    
    time.sleep(0.5)
    
    print(f"\n--- Test 3: Continuous Send (All ON -> OFF) ---")
    for i in range(10):
        try:
            data_on = bytes([0xFF, 0x0F])
            msg = can.Message(arbitration_id=0x300, data=data_on, is_extended_id=False)
            bus.send(msg)
            print(f"  [{i+1}/10] Sent: Data={list(msg.data)} (All ON)")
            time.sleep(0.1)
        except Exception as e:
            print(f"  [{i+1}/10] [FAIL] Send error: {e}")
            break
    
    time.sleep(0.5)
    
    print(f"\n--- Revert to All OFF ---")
    try:
        msg = can.Message(arbitration_id=0x300, data=bytes([0x00, 0x00]), is_extended_id=False)
        bus.send(msg)
        print(f"[SUCCESS] Sent")
    except Exception as e:
        print(f"[FAIL] Send failed: {e}")
    
    print(f"\n--- Checking Receive (1 sec) ---")
    end_time = time.time() + 1.0
    rx_count = 0
    while time.time() < end_time:
        msg = bus.recv(timeout=0.1)
        if msg:
            rx_count += 1
            print(f"  [RX] ID=0x{msg.arbitration_id:03X}, DLC={msg.dlc}, Data={list(msg.data)}")
    
    if rx_count == 0:
        print(f"  (No messages received)")
    else:
        print(f"  Total {rx_count} messages received")
    
    bus.shutdown()
    print(f"\n[DISCONNECTED] CAN closed")
    return True

if __name__ == "__main__":
    print("=" * 60)
    print("  Solenoid Valve CAN Debug")
    print("=" * 60)
    
    ports = find_available_interface()
    
    if not ports:
        print("\n[FAIL] No COM port found")
        sys.exit(1)
    
    interface = sys.argv[1] if len(sys.argv) > 1 else "slcan"
    channel = sys.argv[2] if len(sys.argv) > 2 else "COM3"
    bitrate = int(sys.argv[3]) if len(sys.argv) > 3 else 1000000
    
    test_can_send(interface, channel, bitrate)
    
    print("\n[DONE] Debug complete")
