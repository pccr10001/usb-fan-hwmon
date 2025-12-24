"""
Standalone Test Tool for USB Fan Controller (4-Fan Direct)
Requires: pip install hidapi
"""

import hid
import time
import struct
import sys

# Configuration
VID = 0x1B4F
PID = 0x9206

# Protocol Constants
CMD_FIRMWARE_ID     = 0x01
CMD_FAN_SELECT      = 0x10
CMD_FAN_FIXED_PWM   = 0x13
CMD_FAN_READ_RPM    = 0x16

OP_WRITE_ONE        = 0x06
OP_READ_TWO         = 0x09

REPORT_SIZE = 64

class FanController:
    def __init__(self):
        self.device = None
        self.fan_last_read_time = {} # Store last read timestamp for each fan
        self.seq = 1

    def list_devices(self):
        print("Scanning for supported devices...")
        devices = hid.enumerate(VID, PID)
        for i, d in enumerate(devices):
            print(f"[{i}] {d['product_string']} (Path: {d['path'].decode('utf-8')})")
        return devices

    def open(self, path=None):
        try:
            self.device = hid.device()
            if path:
                self.device.open_path(path)
            else:
                self.device.open(VID, PID)
            print("Device opened successfully.")
            self.device.set_nonblocking(0) # Blocking read
            return True
        except Exception as e:
            print(f"Failed to open device: {e}")
            return False

    def close(self):
        if self.device:
            self.device.close()

    def _next_seq(self):
        s = self.seq
        self.seq += 1
        if self.seq > 31:
            self.seq = 1
        return s

    def _send_command(self, op, cmd, data=None):
        # Packet Structure: [Len, Seq, Op, Cmd, Data...]
        # Note: 'hidapi.write' usually expects Report ID as first byte if device uses it.
        # But for RawHID (generic), it might depend. ATmega32u4 RawHID usually expects raw bytes.
        # Windows hidapi often requires Report ID 0x00 prepended if report descriptors aren't strict?
        # Let's try sending 0x00 + 64 bytes.
        
        if data is None:
            data = []
            
        payload = bytearray(64)
        # Byte 0 is usually Report ID for hidapi on Windows if not configured otherwise.
        # Our Firmware expects: [Len, ...].
        # If we send via hidapi write([0x00, ...]), the 0x00 is consumed as ReportID.
        # So we construct 65 bytes: [0x00, Len, Seq, Op, Cmd, Data...]
        
        # Firmware Logic check:
        # rawhid_rx[0] -> Len. 
        # rawhid_rx[1] -> Seq.
        
        pkt_len = 3 + len(data) # Seq, Op, Cmd = 3 bytes
        
        # Payload content (starts at index 1 in the hid_write buffer)
        payload[0] = pkt_len
        payload[1] = self._next_seq()
        payload[2] = op
        payload[3] = cmd
        for i, b in enumerate(data):
            payload[4+i] = b
            
        # Prepend Report ID 0x00
        report = bytearray([0x00]) + payload
        
        # print(f"Sending: {list(report[:8])}...")
        res = self.device.write(report)
        if res < 0:
            print("Write failed")
            return None
            
        # Read Response
        # Firmware replies with 64 bytes.
        try:
            # timeout_ms optional
            resp = self.device.read(64, timeout_ms=1000)
            if not resp:
                print("No response")
                return None
            return bytes(resp)
        except Exception as e:
            print(f"Read error: {e}")
            return None

    def get_version(self):
        # READ_TWO (0x09) Firmware (0x01)
        res = self._send_command(OP_READ_TWO, CMD_FIRMWARE_ID)
        if res:
            # Res: [Len, Seq, Op, Cmd, Data1, Data2]
            # My FW: Len at 0. Seq at 1. Op at 2. Cmd at 3. Data at 4, 5.
            # hidapi read returns list of bytes.
            # verify header echo
            # Fw echo: [0x00, 0x20] (2.0)
            try:
                major = res[5] >> 4 
                minor = res[5] & 0xF
                patch = res[4]
                return f"{major}.{minor}.{patch}"
            except IndexError:
                return "Unknown"
        return "Error"

    def get_fan_rpm(self, fan_idx):
        if not self.device:
            return 0
        
        # 1. Select Fan
        self._send_command(OP_WRITE_ONE, CMD_FAN_SELECT, [fan_idx])
        
        # 2. Read Tacho Pulses
        res = self._send_command(OP_READ_TWO, CMD_FAN_READ_RPM)
        
        if res and len(res) >= 7:
            # Data starts at index 4 (ID), 5 (Low), 6 (High)
            # fan_id = res[4]
            rpm = res[5] | (res[6] << 8)
            print(f"[DEBUG Fan {fan_idx+1}] RPM: {rpm}")
            return rpm
        
        return 0

    def set_fan_pwm(self, fan_idx, percent):
        # 1. Select Fan
        self._send_command(OP_WRITE_ONE, CMD_FAN_SELECT, [fan_idx])
        
        # 2. Set PWM
        # WRITE_ONE (0x06) FixedPWM (0x13) Duty(0-255)
        duty = int((percent / 100.0) * 255)
        if duty > 255: duty = 255
        if duty < 0: duty = 0
        
        self._send_command(OP_WRITE_ONE, CMD_FAN_FIXED_PWM, [duty])
        print(f"Set Fan {fan_idx+1} to {percent}% (Duty {duty})")

def main():
    ctrl = FanController()
    
    devs = ctrl.list_devices()
    if not devs:
        print("No devices found.")
        return

    # Auto open first
    if not ctrl.open(devs[0]['path']):
        return
        
    print(f"Firmware Version: {ctrl.get_version()}")
    
    while True:
        print("\n--- Menu ---")
        print("1. Status (Show RPMs)")
        print("2. Set Fan Speed")
        print("3. Exit")
        choice = input("Select: ")
        
        if choice == '1':
            print("-" * 20)
            for i in range(4):
                rpm = ctrl.get_fan_rpm(i)
                print(f"Fan {i+1}: {rpm} RPM")
            print("-" * 20)
            
        elif choice == '2':
            try:
                fan = int(input("Fan (1-4): ")) - 1
                if fan < 0 or fan > 3:
                    print("Invalid Fan")
                    continue
                pwm = int(input("Speed % (0-100): "))
                ctrl.set_fan_pwm(fan, pwm)
            except ValueError:
                print("Invalid input")
                
        elif choice == '3':
            break
            
    ctrl.close()

if __name__ == "__main__":
    main()
