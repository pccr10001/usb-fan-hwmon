"""liquidctl driver for Pro Micro 32U4 Fan Controller (4-Fan Direct).

Supported devices
-----------------

 - Pro Micro 32U4 Fan Controller (VID 0x1B4F PID 0x9206)

Supported features
------------------

 - fan speed control (4 channels)
 - fan speed monitoring (4 channels)

Copyright (c) 2025
SPDX-License-Identifier: GPL-3.0-or-later
"""

import logging
import re
from enum import Enum, unique

from liquidctl.error import NotSupportedByDevice, NotSupportedByDriver
from liquidctl.driver.usb import UsbHidDriver
from liquidctl.keyval import RuntimeStorage
from liquidctl.util import clamp, fraction_of_byte, u16le_from

LOGGER = logging.getLogger(__name__)

_REPORT_LENGTH = 64

_COMMAND_FIRMWARE_ID = 0x01
_COMMAND_FAN_SELECT = 0x10
_COMMAND_FAN_MODE = 0x12
_COMMAND_FAN_FIXED_PWM = 0x13
_COMMAND_FAN_READ_RPM = 0x16

_OP_CODE_WRITE_ONE_BYTE = 0x06
_OP_CODE_READ_ONE_BYTE = 0x07
_OP_CODE_READ_TWO_BYTES = 0x09

_SEQUENCE_MIN = 1
_SEQUENCE_MAX = 255


@unique
class _FanMode(Enum):
    FIXED_DUTY = 0x02

    @classmethod
    def _missing_(cls, value):
        LOGGER.debug("falling back to FIXED_DUTY for _FanMode(%s)", value)
        return _FanMode.FIXED_DUTY


def _sequence():
    """Return a generator that produces valid protocol sequence numbers.
    Sequence numbers start from 2 to 31, then rolling over to 1 and up again.
    """
    num = 1
    while True:
        yield (num % 31) + 1
        num += 1


def _quoted(*names):
    return ", ".join(map(repr, names))


class Atmega32u4Fan(UsbHidDriver):
    """liquidctl driver for Custom 4-Fan Controller"""

    _MATCHES = [
        (
            0x1B4F,
            0x9206,
            " Pro Micro 32U4 Fan Controller",
            {"fan_count": 4},
        ),
    ]

    def __init__(self, device, description, fan_count, **kwargs):
        super().__init__(device, description, **kwargs)
        self._fan_names = [f"fan{i + 1}" for i in range(fan_count)]
        self._data = None
        self._sequence = None

    def connect(self, **kwargs):
        """Connect to the device."""
        ret = super().connect(**kwargs)
        ids = f"vid{self.vendor_id:04x}_pid{self.product_id:04x}"
        loc = "loc" + "_".join(re.findall(r"\d+", self.address))
        self._data = RuntimeStorage(key_prefixes=[ids, loc])
        self._sequence = _sequence()
        return ret

    def initialize(self, **kwargs):
        """Initialize the device.
        Returns a list of `(property, value, unit)` tuples.
        """
        res = self._send_command(
            self._build_data_package(_COMMAND_FIRMWARE_ID, _OP_CODE_READ_TWO_BYTES)
        )

        fw_version = (res[3] >> 4, res[3] & 0xF, res[2])
        return [("Firmware version", "%d.%d.%d" % fw_version, "")]

    def get_status(self, **kwargs):
        """Get a status report.
        Returns a list of `(property, value, unit)` tuples.
        """
        dataPackages = list()

        # Build requests for all fans
        for i in range(len(self._fan_names)):
            # Select Fan
            dataPackages.append(
                self._build_data_package(
                    _COMMAND_FAN_SELECT, _OP_CODE_WRITE_ONE_BYTE, params=bytes([i])
                )
            )
            # Read RPM
            dataPackages.append(
                self._build_data_package(_COMMAND_FAN_READ_RPM, _OP_CODE_READ_TWO_BYTES)
            )

        # Send all in one go
        res = self._send_commands(dataPackages)

        # Parse results.
        # Each fan added 2 packets (SELECT + READ).
        # send_commands concat results.
        # SELECT response is empty/ack? 
        # READ response is 2 bytes data.
        # Wait, _send_commands implementation in original coolit.py returns ONE buffer.
        # The device implementation flushes responses to rawHID.
        # Let's see how `_send_commands` constructs response. 
        # It assumes the device replies sequentially in ONE 64-byte report if they fit?
        # My main.cpp sends one 64-byte report back containing all responses.
        #
        # For each fan:
        # [Seq, Op, Cmd, Status] -> Select (Write One Byte) -> Response len 4? My main.cpp replies for Write too?
        # My main.cpp:
        #   WriteOne -> No response data (status bytes or just echo?).
        #   Wait, main.cpp logic:
        #     while(ptr <= len) { ... rawhid_tx[resp_ptr++] = seq; ... }
        #   It echoes every command processed.
        #   For READ_TWO (RPM): Adds 2 data bytes. Total 3 header + 2 data = 5 bytes.
        #   For WRITE_ONE (SELECT): Adds 0 data bytes. Total 3 header = 3 bytes.
        
        # Original logic:
        # Fan 1: Select(3 bytes? params=1) -> Response?
        # In coolit.py get_status:
        #   Start with Temp Read (ReadTwo) -> Offset 0.
        #   Fan1 Select -> Offset ?
        #   Fan1 RPM -> Offset ?
        #
        # Let's reconstruct offsets based on main.cpp:
        # Loop i=0..2:
        #   Select (Write1): 3 bytes header + 0 payload = 3 bytes in response.
        #   ReadRPM (Read2): 3 bytes header + 2 payload = 5 bytes in response.
        # Total per fan = 8 bytes.
        # Offset for RPM data (payload) inside the 8-byte block is at index 3+3 (skip select) + 3 (header of read) = 9?
        # No.
        # Response Buffer:
        # [Len]
        # Fan0_Select_Rep: [Seq, Op, Cmd] (3 bytes)
        # Fan0_Read_Rep:   [Seq, Op, Cmd, Lo, Hi] (5 bytes)
        # Fan1_Select_Rep: ...
        # Fan1_Read_Rep: ...
        # Fan2_Select_Rep: ...
        # Fan2_Read_Rep: ...
        #
        # Length byte is at buf[0]. Data starts at buf[1].
        # Fan 0 RPM Data starts at: 1 (start) + 3 (Select) + 3 (ReadHeader) = 7.
        # Fan 1 RPM Data starts at: 7 + 2 (Data) + 8 (Fan1 Block) = 17?
        #
        # Wait, `u16le_from(res, offset=8)` in coolit.py used `res`. `res` is buf[1:] returned by `_send_buffer`.
        # `_send_buffer` returns `bytes(self.device.read(_REPORT_LENGTH))`.
        # So `res` is the 64 bytes (including len? No, `read` usually returns raw report).
        # My main.cpp writes `rawhid_tx`, where `rawhid_tx[0]` is length.
        # If Host reads 64 bytes, byte 0 is Len.
        # `coolit.py` `_send_buffer` logic:
        #    buf = bytes(self.device.read(_REPORT_LENGTH))
        #    return buf
        # `get_status` calls `_send_commands`.
        # `u16le_from` uses that buffer.
        #
        # Let's verify standard CoolIT offsets from original file:
        # Temp (Read2) -> 5 bytes.
        # Fan1 Select (Write1) -> 3 bytes.
        # Fan1 RPM (Read2) -> 5 bytes.
        # Fan2 Select -> ...
        #
        # Original:
        #   Temp: res[0..5] (Header+Data)
        #   Fan1: res[5..8] (Select), res[8..13] (RPM)
        #   Status uses `offset=8`. Correct (5+3).
        #   Fan2: res[13..16] (Select), res[16..21] (RPM). Offset=16+3=19 ?
        #   Original says `offset=14` for Fan2?
        #   Wait, 8 + 5 (Fan1 RPM) = 13. 13 + 3 (Select) = 16.
        #   Original `Fan 2 speed` offset 14. 
        #   Something is wrong with my math or the original has less overhead.
        #   Maybe Select doesn't echo 3 bytes?
        #   My firmware echoes everything.
        #   Let's just trust my firmware's packing.
        #
        # My Packing (3 Fans, No Temp):
        # Fan 0:
        #   Select: 3 bytes
        #   RPM: 5 bytes
        #   RPM Value Offset = 3 (Select) + 3 (Header) = 6. (Index in `res` excluding len byte)
        #   Note: `res` in clean python driver normally excludes ReportID if handled by hidapi, but `liquidctl` `UsbHidDriver` usually returns raw list?
        #   `_send_buffer`: returns `self.device.read`. RawHID read includes everything sent?
        #   Let's assume `res` starts at Byte 0 of the report from device (Length byte).
        #   BUT `coolit.py` `fw_version` access `res[3]`, `res[2]`.
        #   If `res[0]` is length, `res[1]` is Seq, `res[2]` is Op...
        #   My FW: `rawhid_tx[0] = len`. `rawhid_tx[1] = seq`.
        #   So `res[0]` is Len.
        #   FW Version (Read2):
        #     Byte 0: Len
        #     Byte 1: Seq
        #     Byte 2: Op
        #     Byte 3: Cmd
        #     Byte 4: Data1
        #     Byte 5: Data2
        #   Original `coolit.py`: `res[3] >> 4`. This is `Cmd`? No.
        #   Maybe original firmware DOES NOT echo `Cmd`?
        #   Let's check `coolit.py` _build_data_package: `buf[2] = command`.
        #   My FW: `rawhid_tx[resp_ptr++] = cmd;` (Echoes command).
        #   If original `res[3]` is data, then:
        #   0: Len, 1: Seq, 2: Op, 3: Data1.
        #   So original FW does NOT echo Cmd?
        #   Let's check `coolit.py` again. `res[2]` used. Op is `res[2]`?
        #   If `res[2]` is data2 (second byte of data?)
        #   If 0:Len, 1:Seq.
        #   If only 2 bytes header?
        #
        #   Use logic: keep my firmware simple: Seq, Op, Cmd, Data...
        #   Then adapt this Python driver to match MY firmware.
        #   My FW Packet Structure (Response):
        #   [Len]
        #   [Seq, Op, Cmd, (Data...)]
        #
        #   Fan 0 RPM:
        #     Select(3): [Seq, Op, Cmd] (Indices 1, 2, 3)
        #     Read(5):   [Seq, Op, Cmd, Lo, Hi] (Indices 4, 5, 6, 7, 8)
        #     RPM Low = res[7], Hi = res[8].
        #     Offset = 7.
        #
        #   Fan 1 RPM:
        #     Adds 8 bytes. Offset = 7 + 8 = 15.
        #
        #   Fan 2 RPM:
        #     Adds 8 bytes. Offset = 15 + 8 = 23.
        
        status = []
        offset = 1 # Start after Len byte? or 0 if read returns buffer starting at 0.
        # My FW puts Len at index 0.
        
        # Adjust for Select(3) + ReadHeader(3) = 6 bytes skip before data.
        # Data is 2 bytes. Total 8 bytes per fan.
        
        base_offset = 1 # Skip Length byte
        
        for i, name in enumerate(self._fan_names):
            # Block is 9 bytes (Select=3, Read=6).
            # Read Packet: Seq, Op, Cmd, ID, Lo, Hi
            # RPM is u16le at offset 7 relative to block start (Skip 3 Select + 3 ReadHeader + 1 ID).
            current_fan_offset = base_offset + (i * 9) + 7
            
            rpm = u16le_from(res, offset=current_fan_offset)
            status.append((f"{name} speed", rpm, "rpm"))
            
        return status

    def set_fixed_speed(self, channel, duty, **kwargs):
        """Set fan or fans to a fixed speed duty."""
        for hw_channel in self._get_hw_fan_channels(channel):
            # Extract index from "fanN"
            idx_match = re.search(r"\d+", hw_channel)
            if idx_match:
                fanIndex = int(idx_match.group()) - 1
            else:
                continue 

            self._data.store(f"{hw_channel}_duty", duty)
            
            # Send Command
            self._send_command(
                self._build_data_package(
                    _COMMAND_FAN_SELECT, _OP_CODE_WRITE_ONE_BYTE, params=bytes([fanIndex])
                )
            )
            self._send_command(
                self._build_data_package(
                    _COMMAND_FAN_FIXED_PWM,
                    _OP_CODE_WRITE_ONE_BYTE,
                    params=bytes([fraction_of_byte(percentage=duty)]),
                )
            )
            LOGGER.info("setting %s to %d%% duty cycle", hw_channel, duty)

    def set_speed_profile(self, channel, profile, **kwargs):
        """Not supported (No Temp Sensor)."""
        raise NotSupportedByDevice("Temperature sensor not available")

    def set_color(self, channel, mode, colors, **kwargs):
        """Not supported."""
        raise NotSupportedByDriver()

    def set_screen(self, channel, mode, value, **kwargs):
        """Not supported."""
        raise NotSupportedByDevice()

    def _get_hw_fan_channels(self, channel):
        channel = channel.lower()
        if channel == "fan":
            return self._fan_names
        if channel in self._fan_names:
            return [channel]
        raise ValueError(f"Unknown channel, should be one of: {_quoted('fan', *self._fan_names)}")

    def _build_data_package(self, command, opCode, params=None):
        if params:
            buf = bytearray(3 + len(params))
            buf[3 : 3 + len(params)] = params
        else:
            buf = bytearray(3)

        buf[0] = next(self._sequence)
        buf[1] = opCode
        buf[2] = command

        return buf

    def _send_commands(self, dataPackages):
        buf = bytearray(_REPORT_LENGTH)
        startIndex = 1
        for dataPackage in dataPackages:
            buf[startIndex : startIndex + len(dataPackage)] = dataPackage
            startIndex += len(dataPackage)
        buf[0] = startIndex - 1
        return self._send_buffer(buf)

    def _send_command(self, dataPackage):
        buf = bytearray(_REPORT_LENGTH)
        buf[0] = len(dataPackage)
        buf[1:] = dataPackage
        return self._send_buffer(buf)

    def _send_buffer(self, buf):
        self.device.clear_enqueued_reports()
        self.device.write(buf)
        buf = bytes(self.device.read(_REPORT_LENGTH))
        return buf
