# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
Implement pyocd DebugProbe API with native CircuitPython DigitalInOut for standalone use.


* Author(s): Scott Shawcroft
"""

import digitalio
import time
from . import DebugProbe
__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_Debug_Probe.git"


SWD_ACK_OK = 0b001
SWD_ACK_WAIT= 0b010
SWD_ACK_FAULT = 0b100

class BitbangProbe(DebugProbe):
    def __init__(self, clk, dio, nreset=None, drive_mode=digitalio.DriveMode.OPEN_DRAIN):
        self.clk = clk
        self.dio = dio
        self.nreset = nreset
        self.drive_mode = drive_mode

    def get_accessible_pins(self, group: PinGroup) -> Tuple[int, int]:
        if group == DebugProbe.PinGroup.PROTOCOL_PINS:
            accessible = 0x3 # swdio and swclk
            if self.nreset:
                accessible |= 1 << 4
            return (accessible, accessible)
        return (0, 0)

    def read_pins(self, group: PinGroup, mask: int) -> int:
        raise NotImplementedError()

    def write_pins(self, group: PinGroup, mask: int, value: int) -> None:
        if group != DebugProbe.PinGroup.PROTOCOL_PINS:
            raise NotImplementedError()
        for bit, pin in ((0, self.clk), (1, self.dio), (4, self.nreset)):
            if not pin or (mask & (1 << bit)) == 0:
                continue
            v = (value & (1 << bit)) != 0
            pin.switch_to_output(v, drive_mode=self.drive_mode)

    def connect(self, protocol: Optional[Protocol] = None) -> None:
        """@brief Initialize DAP IO pins for JTAG or SWD"""
        self.clk.switch_to_output(True, drive_mode=digitalio.DriveMode.PUSH_PULL)
        self.dio.switch_to_output(True, drive_mode=self.drive_mode)

    def disconnect(self) -> None:
        self.clk.switch_to_input()
        self.dio.switch_to_input()
        if self.nreset:
            self.nreset.switch_to_input()

    def _swd_write(self, bit_count, value):
        clk = self.clk
        dio = self.dio
        clk.switch_to_output(True, drive_mode=digitalio.DriveMode.PUSH_PULL)
        dio.switch_to_output(True, drive_mode=self.drive_mode)
        for _ in range(bit_count):
            dio.value = (value & 1) == 1
            clk.value = False
            value >>= 1
            clk.value = True

    def _swd_read(self, bit_count) -> int:
        data = 0
        clk = self.clk
        dio = self.dio
        next_bit = 1
        for _ in range(bit_count):
            clk.value = False
            data |= next_bit if dio.value else 0
            clk.value = True
            next_bit <<= 1
        return data

    def swj_sequence(self, length: int, bits: int) -> None:
        self._swd_write(length, bits)

    def swd_sequence(self, sequences: Sequence[Union[Tuple[int], Tuple[int, int]]]) -> Tuple[int, Sequence[bytes]]:
        responses = []
        for sequence in sequences:
            if len(sequence) == 1:
                responses.append(self._swd_write(sequence[0]).tobytes())
            else:
                self._swd_write(sequence[0], sequence[1])
        return (True, responses)

    def jtag_sequence(self, cycles: int, tms: int, read_tdo: bool, tdi: int) -> Optional[int]:
        raise NotImplementedError()

    def set_clock(self, frequency: float) -> None:
        # We're bitbanging so we're not going fast.
        pass

    def reset(self) -> None:
        if self.nreset:
            self.nreset.switch_to_output(False)
            time.sleep(0.1)
            self.nreset.value = True

    def assert_reset(self, asserted: bool) -> None:
        if self.nreset:
            if asserted:
                self.nreset.switch_to_output(False)
            else:
                self.nreset.switch_to_input()

    def is_reset_asserted(self) -> bool:
        if self.nreset:
            return self.nreset.direction == digitalio.Direction.OUTPUT
        return False

    def _count_ones(self, value):
        one_count = 0
        for _ in range(32):
            if (value & 1) == 1:
                one_count += 1
            value >>= 1
        return one_count

    def _packet(self, header, read=False, write_word=None) -> int:
        ack = SWD_ACK_WAIT
        retries = 0
        while ack == SWD_ACK_WAIT:
            self._swd_write(8, header)
            clk = self.clk
            dio = self.dio
            dio.switch_to_input()
            clk.value = False
            clk.value = True
            ack = self._swd_read(3)
            if ack == SWD_ACK_WAIT:
                # # Do a data phase while we wait.
                # self._swd_read(33)
                clk.value = False
                clk.value = True
                dio.switch_to_output(True, drive_mode=self.drive_mode)
            retries += 1

        if ack != SWD_ACK_OK:
            clk.value = False
            clk.value = True
            dio.switch_to_output(True, drive_mode=self.drive_mode)
            raise RuntimeError(f"Ack not ok: 0b{ack:03b}")
            return
        rdata = 0
        if read:
            rdata = self._swd_read(33)
            one_count = self._count_ones(rdata)
            if one_count % 2 != rdata >> 32:
                raise RuntimeError("Parity failure")
            rdata &= 0xffffffff
        clk.value = False
        clk.value = True
        dio.switch_to_output(True, drive_mode=self.drive_mode)
        if write_word is not None:
            self._swd_write(32, write_word)
            one_count = self._count_ones(write_word)
            # The lowest bit will be 1 if odd and 0 if even and therefore matches
            # the parity bit setting.
            self._swd_write(1, one_count)

        return rdata

    def read_dp(self, addr: int) -> int:
        # Even ones checks bits 2 and 3 and compares them. If they are the same,
        # then the number of additional ones is 0 or 2 and parity matches Read
        # and DP's one count. Otherwise, it is inverted.
        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        # 0x85 is Read 1 and DP 0.
        rdata = self._packet(0x85 | addr << 1 | (0x20 if even_ones else 0), read=True)
        return rdata

    def write_dp(self, addr: int, data: int) -> None:
        # 0x81 is Read 0 and DP 0.
        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        self._packet(0x81 | addr << 1 | (0x20 if not even_ones else 0), write_word=data)

    def read_ap(self, addr: int) -> int:
        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        # 0x87 is Read 1 and DP 1
        rdata = self._packet(0x87 | addr << 1 | (0x20 if not even_ones else 0), read=True)
        return rdata

    def write_ap(self, addr: int, data) -> None:
        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        # 0x87 is Read 0 and DP 1
        self._packet(0x83 | addr << 1 | (0x20 if even_ones else 0), write_word=data)

    def read_ap_multiple(self, addr: int, count: int = 1) -> Sequence[int]:
        buf = bytearray(count * 4)
        result = memoryview(buf).cast("I")

        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        # 0x87 is Read 1 and DP 1
        command = 0x87 | addr << 1 | (0x20 if not even_ones else 0)
        for i in range(count):
            result[i] = self._packet(command, read=True)
        return result

    def write_ap_multiple(self, addr: int, values) -> None:
        even_ones = ((addr & 0x4) == 0) == ((addr & 0x8) == 0)
        command = 0x83 | addr << 1 | (0x20 if even_ones else 0)
        for v in values:
            self._packet(command, write_word=v)
        self.read_dp(0x0c)
 
