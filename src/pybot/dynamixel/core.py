# -*- coding: utf-8 -*-

#  Copyright 2013 Eric PASCUAL <eric <at> pobot <dot> org>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.

""" A lightweight Dynamixel bus interface layer """

__author__ = "Eric Pascual (eric@pobot.org)"


class Instruction(object):
    """ A static class defining symbolic names for instructions."""
    PING = 1
    READ_DATA = 2
    WRITE_DATA = 3
    REG_WRITE = 4
    ACTION = 5
    RESET = 6
    SYNC_WRITE = 0x83


class StatusMask(object):
    """ The masks for the error status byte. """
    InstructionError = 1 << 6
    OverloadError = 1 << 5
    ChecksumError = 1 << 4
    RangeError = 1 << 3
    OverheatingError = 1 << 2
    AngleLimitError = 1 << 1
    InputVoltageError = 1

    @staticmethod
    def as_str(err):
        res = []
        for s in [s for s in dir(StatusMask) if not s.startswith('_')]:
            attr = getattr(StatusMask, s)
            if type(attr) is int and attr & err:
                res.append(s)

        return ','.join(res)


class Register(object):
    """ The Dynamixel servos registers base set.

    Defined for AX12 and similar models.
    """
    ModelNumber = 0
    FirmwareVersion = 2
    Id = 3
    BaudRate = 4
    ReturnDelay = 5
    CWAngleLimit = 6
    CCWAngleLimit = 8
    TemperatureLimit = 11
    LowVoltageLimit = 12
    HighVoltageLimit = 13
    MaxTorque = 14
    StatusReturnLevel = 16
    AlarmLED = 17
    AlarmShutdown = 18
    DownCalibration = 20
    UpCalibration = 22
    TorqueEnable = 24
    LED = 25
    CWComplianceMargin = 26
    CCWComplianceMargin = 27
    CWComplianceSlope = 28
    CCWComplianceSlope = 29
    GoalPosition = 30
    MovingSpeed = 32
    TorqueLimit = 34
    CurrentPosition = 36
    CurrentSpeed = 38
    CurrentLoad = 40
    CurrentVoltage = 42
    CurrentTemperature = 43
    RegisteredInstruction = 44
    Moving = 46
    Lock = 47
    Punch = 48

    all_regs = {
        ModelNumber: ("Model Number", 2, False),
        FirmwareVersion: ("Firmware Version", 1, False),
        Id: ("Id", 1, True, 0, 0xfd),
        BaudRate: ("Baud Rate", 1, True, 0, 0xfe),
        ReturnDelay: ("Return Delay", 1, True, 0, 0xfe),
        CWAngleLimit: ("CW Angle Limit", 2, True, 0, 0x3ff),
        CCWAngleLimit: ("CCW Angle Limit", 2, True, 0, 0x3ff),
        TemperatureLimit: ("Temperature Limit", 1, True, 0, 0x96),
        LowVoltageLimit: ("Low Voltage Limit", 1, True, 0x32, 0xfa),
        HighVoltageLimit: ("High Voltage Limit", 1, True, 0x32, 0xfa),
        MaxTorque: ("Max Torque", 2, True, 0, 0x3ff),
        StatusReturnLevel: ("Status Return Level", 1, True, 0, 2),
        AlarmLED: ("Alarm Led", 1, True, 0, 0x7f),
        AlarmShutdown: ("Alarm Shutdown", 1, True, 0, 0x7f),
        DownCalibration: ("Down Calibration", 2, False),
        UpCalibration: ("Up Calibration", 2, False),
        TorqueEnable: ("Torque Enable", 1, True, 0, 1),
        LED: ("LED", 1, True, 0, 1),
        CWComplianceMargin: ("CW Compliance Margin", 1, True, 0, 0xfe),
        CCWComplianceMargin: ("CCW Compliance Margin", 1, True, 0, 0xfe),
        CWComplianceSlope: ("CW Compliance Slope", 1, True, 0, 0xfe),
        CCWComplianceSlope: ("CCW Compliance Slope", 1, True, 0, 0xfe),
        GoalPosition: ("Goal Position", 2, True, 0, 0x3ff),
        MovingSpeed: ("Moving Speed", 2, True, 0, 0x3ff),
        TorqueLimit: ("Torque Limit", 2, True, 0, 0x3ff),
        CurrentPosition: ("Current Position", 2, False),
        CurrentSpeed: ("Current Speed", 2, False),
        CurrentLoad: ("Current Load", 2, False),
        CurrentVoltage: ("Current Voltage", 1, False),
        CurrentTemperature: ("Current Temperature", 1, False),
        RegisteredInstruction: ("Registered Instruction", 1, True, 0, 1),
        Moving: ("Moving", 1, False),
        Lock: ("Lock", 1, True, 1, 1),
        Punch: ("Punch", 2, True, 0, 0x3ff)
    }

    # register descriptor tuple field indexes
    _m_label = 0
    _m_size = 1
    _m_writeable = 2
    _m_min_value = 3
    _m_max_value = 4

    class __metaclass__(type):  # pylint disable=W
        def __iter__(cls):
            """ Makes the class type iterable. """
            return cls.all_regs.iterkeys()

    @classmethod
    def itervalues(cls):
        return cls.all_regs.itervalues()

    @classmethod
    def iterkeys(cls):
        return cls.all_regs.iterkeys()

    _decoder = {}

    @classmethod
    def label(cls, reg):
        return cls.all_regs[reg][cls._m_label]

    @classmethod
    def size(cls, reg):
        return cls.all_regs[reg][cls._m_size]

    @classmethod
    def is_writeable(cls, reg):
        return cls.all_regs[reg][cls._m_writeable]

    @classmethod
    def check_value(cls, reg, value):
        reg_meta = cls.all_regs[reg]
        if not reg_meta[cls._m_min_value] <= value <= reg_meta[cls._m_max_value]:
            raise ValueError('range error (reg=%s value=%d)' % (reg, value))

    @classmethod
    def clamp_value(cls, reg, value):
        reg_meta = cls.all_regs[reg]
        return min(max(value, reg_meta[cls._m_min_value]), reg_meta[cls._m_max_value])

    @classmethod
    def check_id(cls, reg):
        if reg not in cls.all_regs:
            raise ValueError('invalid register id (%d)' % reg)

    @classmethod
    def dumps(cls, reg, value):
        try:
            displayed_value = cls._decoder[reg](value)
        except KeyError:
            displayed_value = str(value)
        return '- [%02d] %-30s : %s (0x%0.2x)' % (reg, cls.label(reg), displayed_value, value)

    @classmethod
    def dump(cls, reg, value):
        print(cls.dumps(reg, value))

    @classmethod
    def iter(cls):
        return cls.all_regs.iterkeys()


BAUDRATE = {
    1: 1000000,
    3: 500000,
    4: 400000,
    7: 250000,
    9: 200000,
    16: 115200,
    34: 57600,
    103: 19200,
    207: 9600
}


def _decode_baudrate(value):
    return BAUDRATE[value]


STATUS_LEVEL = {
    0: 'no_reply',
    1: 'read_data_only',
    2: 'all'
}


def _decode_status_level(value):
    return STATUS_LEVEL[value]


def _decode_error_status(value):
    return StatusMask.as_str(value)


def _decode_reginst(value):
    return 'yes' if value else 'no'


Register._decoder = {
    Register.BaudRate: _decode_baudrate,
    Register.StatusReturnLevel: _decode_status_level,
    Register.AlarmLED: _decode_error_status,
    Register.AlarmShutdown: _decode_error_status,
    Register.RegisteredInstruction: _decode_reginst
}


class DmxlError(RuntimeError):
    pass


class InvalidReplyStartError(DmxlError):
    def __init__(self, data):
        super(InvalidReplyStartError, self).__init__()
        self.data = data

    def __str__(self):
        return 'invalid reply start : %s' % self.data


class IdMismatchError(DmxlError):
    def __init__(self, received, expected):
        super(IdMismatchError, self).__init__()
        self.received, self.expected = received, expected

    def __str__(self):
        return 'id mismatch: rcv=%d exp=%d' % (self.received, self.expected)


class InstructionError(DmxlError):
    def __init__(self, status):
        super(InstructionError, self).__init__()
        self.status = status

    def __str__(self):
        return 'instruction failed with status=%s' % StatusMask.as_str(self.status)
