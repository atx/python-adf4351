# The MIT License (MIT)
#
# Copyright (C) 2016 Josef Gajdusek <atx@atx.name>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import spidev
try:
    from math import gcd
except ImportError:
    from fractions import gcd


def bit(x):
    return 1 << x

def bits(n):
    return (1 << n) - 1

def insert_val(n, v, o, m):
    return (n & ~m) | ((v << o) & m)


class Register:

    def __init__(self, bits):
        self.bits = bits
        self.val = 0x00000000

    def __get__(self, obj, obtype = None):
        if obj is None:
            return self
        return (self.val | self.bits) if self.val is not None else None

    def __set__(self, obj, val):
        self.val = val & ~0b111
        obj.write(val | self.bits)


class RegBit:

    def __init__(self, reg, bit):
        self.reg = reg
        self.bit = bit

    def __get__(self, obj, obtype = None):
        if obj is None:
            return self
        return bool(self.reg.__get__(obj) & self.bit)

    def __set__(self, obj, val):
        rval = self.reg.__get__(obj)
        if val:
            rval |= self.bit
        else:
            rval &= ~self.bit
        self.reg.__set__(obj, rval)


class RegVal:

    def __init__(self, reg, off, width):
        self.reg = reg
        self.off = off
        self.mask = ((1 << width) - 1) << self.off

    def __get__(self, obj, obtype = None):
        if obj is None:
            return self
        return (self.reg.__get__(obj) & self.mask) >> self.off

    def __set__(self, obj, val):
        rval = self.reg.__get__(obj)
        rval &= ~self.mask
        rval |= (val << self.off) & self.mask
        self.reg.__set__(obj, rval)


class ADF4351:

    R0_FRAC_OFF = 3
    R0_FRAC_MASK = bits(12) << R0_FRAC_OFF
    R0_INT_OFF = 15
    R0_INT_MASK = bits(16) << R0_INT_OFF

    R1_MOD_OFF = 3
    R1_MOD_MASK = bits(12) << R1_MOD_OFF
    R1_PHASE_OFF = 15
    R1_PHASE_MASK = bits(12) << R1_PHASE_OFF
    R1_PRESCALER = bit(27)
    R1_PHASE_ADJUST = bit(28)

    R2_COUNTER_RESET_ENABLE = bit(3)
    R2_CP_THREE_STATE_ENABLE = bit(4)
    R2_POWER_DOWN = bit(5)
    R2_PD_POLARITY_POSITIVE = bit(6)
    R2_LDP_6NS = bit(7)
    R2_LDF_INTN = bit(8)
    R2_CHARGE_PUMP_CURRENT_OFF = 9
    R2_DOUBLE_BUFFER_ENABLE = bit(13)
    R2_R_OFF = 14
    R2_R_MASK = bits(10) << R2_R_OFF
    R2_RDIV2_ENABLE = bit(24)
    R2_REFERENCE_DOUBLER_ENABLE = bit(25)
    R2_MUXOUT_OFF = 26
    R2_MUXOUT_THREE_STATE = 0 << R2_MUXOUT_OFF
    R2_MUXOUT_DVDD = 1 << R2_MUXOUT_OFF
    R2_MUXOUT_DGND = 2 << R2_MUXOUT_OFF
    R2_MUXOUT_R = 3 << R2_MUXOUT_OFF
    R2_MUXOUT_N = 4 << R2_MUXOUT_OFF
    R2_MUXOUT_ALD = 5 << R2_MUXOUT_OFF
    R2_MUXOUT_DLD = 6 << R2_MUXOUT_OFF
    R2_LOW_NOISE_SPUR_OFF = 29

    R3_CLK_DIV_OFF = 3
    R3_CLK_DIV_MASK = bits(12) << R3_CLK_DIV_OFF
    R3_CLK_DIV_MODE_OFF = 15
    R3_CLK_DIV_MODE_DISABLED = 0b00 << R3_CLK_DIV_MODE_OFF
    R3_CLK_DIV_MODE_FAST_LOCK = 0b01 << R3_CLK_DIV_MODE_OFF
    R3_CLK_DIV_MODE_RESYNC_ENABLE = 0b10 << R3_CLK_DIV_MODE_OFF
    R3_CSR = bit(18)
    R3_CHARGE_CANCEL = bit(21)
    R3_ABP = bit(22)
    R3_BAND_SELECT_MODE = bit(23)

    R4_OUTPUT_POWER_OFF = 3
    R4_OUTPUT_POWER__4DBM = 0b00 << R4_OUTPUT_POWER_OFF
    R4_OUTPUT_POWER__1DBM = 0b01 << R4_OUTPUT_POWER_OFF
    R4_OUTPUT_POWER_2DBM = 0b10 << R4_OUTPUT_POWER_OFF
    R4_OUTPUT_POWER_5DBM = 0b11 << R4_OUTPUT_POWER_OFF
    R4_RF_OUTPUT_ENABLE = bit(5)
    R4_AUX_OUTPUT_POWER_OFF = 6
    R4_AUX_OUTPUT_POWER__4DBM = 0b00 << R4_AUX_OUTPUT_POWER_OFF
    R4_AUX_OUTPUT_POWER__1DBM = 0b01 << R4_AUX_OUTPUT_POWER_OFF
    R4_AUX_OUTPUT_POWER_2DBM = 0b10 << R4_AUX_OUTPUT_POWER_OFF
    R4_AUX_OUTPUT_POWER_5DBM = 0b11 << R4_AUX_OUTPUT_POWER_OFF
    R4_AUX_OUTPUT_ENABLE = bit(8)
    R4_AUX_OUTPUT_SELECT = bit(9)
    R4_MTLD = bit(10)
    R4_VCO_POWER_DOWN = bit(11)
    R4_BAND_SELECT_CLOCK_DIV_OFF = 12
    R4_BAND_SELECT_CLOCK_DIV_MASK = bits(8) << R4_BAND_SELECT_CLOCK_DIV_OFF
    R4_DIVIDER_SELECT_OFF = 20
    R4_DIVIDER_SELECT_MASK = 0b111 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_1 = 0b000 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_2 = 0b001 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_4 = 0b010 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_8 = 0b011 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_16 = 0b100 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_32 = 0b101 << R4_DIVIDER_SELECT_OFF
    R4_DIVIDER_SELECT_64 = 0b110 << R4_DIVIDER_SELECT_OFF
    R4_FEEDBACK_SELECT = bit(23)

    R5_LD_PIN_MODE_OFF = 22
    R5_LD_PIN_MODE_LOW = 0b00 << R5_LD_PIN_MODE_OFF
    R5_LD_PIN_MODE_LOCK_DETECT = 0b01 << R5_LD_PIN_MODE_OFF
    R5_LD_PIN_MODE_HIGH = 0b11 << R5_LD_PIN_MODE_OFF

    r0 = Register(0b000)
    r1 = Register(0b001)
    r2 = Register(0b010)
    r3 = Register(0b011)
    r4 = Register(0b100)
    r5 = Register(0b101)

    r_counter = RegVal(r2, R2_R_OFF, 10)
    charge_pump_current = RegVal(r2, R2_CHARGE_PUMP_CURRENT_OFF, 4)

    clock_divider_val = RegVal(r3, R3_CLK_DIV_OFF, 12)
    band_select_high = RegBit(r3, R3_BAND_SELECT_MODE)

    output_power = RegVal(r4, R4_OUTPUT_POWER_OFF, 2)
    output_enable = RegBit(r4, R4_RF_OUTPUT_ENABLE)
    aux_output_power = RegVal(r4, R4_AUX_OUTPUT_POWER_OFF, 2)
    aux_output_enable = RegBit(r4, R4_AUX_OUTPUT_ENABLE)
    mute_till_lock_detect = RegBit(r4, R4_MTLD)
    feedback_fundamental = RegBit(r4, R4_FEEDBACK_SELECT)

    OUTPUT_DIVIDER_1 = 0b000
    OUTPUT_DIVIDER_2 = 0b001
    OUTPUT_DIVIDER_4 = 0b010
    OUTPUT_DIVIDER_8 = 0b011
    OUTPUT_DIVIDER_16 = 0b100
    OUTPUT_DIVIDER_32 = 0b101
    OUTPUT_DIVIDER_64 = 0b110

    def __init__(self, bus, dev, refclk):
        # TODO SPI
        self.spi = spidev.SpiDev(bus, dev)
        # Technically, the maximum speed is 20MHz, but my logic analyzer does
        # not go that high
        self.spi.max_speed_hz = 5000000
        # CPOL = 0, CPHA = 0
        self.spi.mode = 0b00
        self.refclk = refclk
        # Initialize to some sort of known state
        self.init()

    def init(self):
        self.r5 = ADF4351.R5_LD_PIN_MODE_LOCK_DETECT
        self.r4 = ADF4351.R4_MTLD
        self.feedback_fundamental = True
        self.output_enable = True
        self.output_power = 3
        self.r3 = 0x00000000
        self.clock_divider_val = 150
        self.r2 = 0x00000000
        self.r_counter = 1
        self.charge_pump_current = 0b0111
        self.r1 = 0x00000000
        self.r0 = 0x00000000

    def write(self, val):
        self.spi.xfer([(val >> x) & 0xff for x in range(24, -1, -8)])

    def set_frequency(self, fout, spacing = 100e3):
        # Based on https://ez.analog.com/thread/13743
        # TODO: Reference doubler/divider
        fpfd = self.refclk
        outdivval = 0
        outdiv = 1
        while fout * outdiv < 2200e6:
            outdiv *= 2
            outdivval += 1
        if self.feedback_fundamental:
            N = fout * outdiv / fpfd
        else:
            N = fout / fpfd
        INT = int(N)
        MOD = int(fpfd / spacing)
        FRAC = int((N - INT) * MOD)

        div = gcd(MOD, FRAC)
        MOD //= div
        FRAC //= div

        if MOD == 1:
            MOD = 2

        # TODO: PDF max error check

        # Band select clock speed
        fpfdm = fpfd / 1e6
        if self.band_select_high:
            bandseldiv = int(2 * fpfdm)
            if 2 * fpfdm - bandseldiv > 0:
                bandseldiv += 1
        else:
            bandseldiv = int(8 * fpfdm)
            if 8 * fpfd - bandseldiv > 0:
                bandseldiv += 1
        bandseldiv = min(bandseldiv, 255)

        # Write the register values
        r4 = insert_val(self.r4, bandseldiv,
                        ADF4351.R4_BAND_SELECT_CLOCK_DIV_OFF,
                        ADF4351.R4_BAND_SELECT_CLOCK_DIV_MASK)
        r4 = insert_val(r4, outdivval,
                        ADF4351.R4_DIVIDER_SELECT_OFF,
                        ADF4351.R4_DIVIDER_SELECT_MASK)
        r1 = insert_val(self.r1, MOD,
                        ADF4351.R1_MOD_OFF,
                        ADF4351.R1_MOD_MASK)
        r1 = insert_val(r1, 0b0001,
                        ADF4351.R1_PHASE_OFF,
                        ADF4351.R1_PHASE_MASK)
        r0 = insert_val(self.r0, FRAC,
                        ADF4351.R0_FRAC_OFF,
                        ADF4351.R0_FRAC_MASK)
        r0 = insert_val(r0, INT,
                        ADF4351.R0_INT_OFF,
                        ADF4351.R0_INT_MASK)
        self.r4 = r4
        self.r1 = r1
        self.r0 = r0

    def close(self):
        self.spi.close()

