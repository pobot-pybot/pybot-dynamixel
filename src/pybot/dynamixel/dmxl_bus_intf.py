#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import serial
from pybot.dynamixel import InvalidReplyStartError, IdMismatchError, InstructionError, Instruction, Register, DmxlError

__author__ = 'Eric Pascual'


class DynamixelBusInterface(object):
    """ Model of a Dynamixel bus interfaces accessed though a virtual serial port.

    Unlike other libraries commonly available, we don't add a model layer for
    representing the servos as instances, since they are pretty straightforward objects
    which don't deserve adding an extra overhead just for wrapping the access to their
    registers and presenting them as properties.

    Use PyDynamixel (http://code.google.com/p/pydynamixel/) for instance if you
    look for this kind of interface.

    Based on the the standard Robotis USB2Dynamixel, so you can instantiate
    it for this interface, but it's better to use the USB2Dynamixel sub-class, since
    it defaults the port properly.
    """

    BROADCASTING_ID = 0xFE
    INTERFACE_ID = 0xFD

    def __init__(self, port, baudrate=1000000, timeout=0.1, debug=False, simulate=False):
        """
        Constructor.

        Defaults values for parameters are OK for most of the out of the box
        interfaces, using the bus at its maximum speed.
        """
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._serial.flushInput()
        self._serial.flushOutput()
        self._debug = debug or simulate
        self._simulate = simulate
        self._serial_lock = threading.Lock()

    def write(self, data):
        """ Writes data to the bus.

        :param data: the data to be sent
        :type data: list or string
        """
        if isinstance(data, list):
            # stringify a byte list
            data = ''.join([chr(b) for b in data])

        if self._debug:
            print(':Tx>[%s] %s' % (
                threading.current_thread().name,
                ' '.join('%02x' % ord(b) for b in data)
            ))

        if self._simulate:
            return

        self._serial.write(data)
        self._serial.flush()

    def read(self, count=1):
        """ Reads a given number of bytes from the bus.

        See pyserial.Serial class for details about the behavior of the
        read() method with respect to timeout settings.

        :param int count: the count of bytes to be read
        :returns: the read data or None if timeout set and exhausted
        :rtype: string
        """
        return self._serial.read(count)

    def get_reply(self, servo_id):
        """ Awaits for a servo reply and returns its payload and error status.

        :param int servo_id: the expected servo id in the reply
        :returns: a tuple containing the payload of the reply as a byte list and the error status
        :rtype: (list of byte, int)
        """
        if self._simulate:
            print ('<Rx: -- no Rx data when running in simulated I/O mode --')
            return [], 0

        hdr = self._serial.read(2)
        if hdr != '\xff\xff':
            self._serial.flushInput()
            raise InvalidReplyStartError([ord(b) for b in hdr])
        received_id = ord(self._serial.read(1))
        if received_id != servo_id:
            self._serial.flushInput()
            raise IdMismatchError(received_id, servo_id)
        data_length = ord(self._serial.read(1)) - 2
        err = ord(self._serial.read(1))
        data = [ord(c) for c in self._serial.read(data_length)] if data_length else []
        chk = ord(self._serial.read(1))
        if self._debug:
            rx = ' '.join('%02x' % b for b in [0xff, 0xff, received_id, data_length + 2, err] + data + [chk])
            print('<Rx:[%s] %s' % (threading.current_thread().name, rx))
        self._serial.flushInput()
        return data, err

    @staticmethod
    def _checksum(data):
        """ Computes the checksum of a data buffer."""
        checksum = 0
        for m in data:
            checksum += m
        checksum = (~checksum) % 256
        return checksum

    @staticmethod
    def _check_error(err):
        """ Checks an error status value and raises an exception if not ok """
        if err:
            raise InstructionError(err)

    def write_instruction(self, servo_id, instruction_bytes):
        """ Sends an instruction and waits for its reply (if not a broadcast).

        :param int servo_id: the target servo id or the broadcast id
        :param list instruction_bytes: the instruction as a byte list
        :returns: the received reply as a byte list or nothing for a broadcast
        :rtype: list of byte
        :raises InstructionError: if the received error status is not OK
        """
        _bytes = [0xff, 0xff, servo_id, len(instruction_bytes) + 1] + instruction_bytes
        chk = self._checksum(_bytes[2:])
        # wrap the dialog in a lock so that nobody can place a request
        # until this one is complete
        with self._serial_lock:
            self.write(_bytes + [chk])

            # no reply expected if broadcasted instruction
            if servo_id == DynamixelBusInterface.BROADCASTING_ID:
                return
            # same when writing to the id register (0x03)
            if instruction_bytes[0] == Instruction.WRITE_DATA and instruction_bytes[1] == Register.Id:
                return

            reply, err = self.get_reply(servo_id)
            self._check_error(err)
            return reply

    def read_register(self, servo_id, reg_addr):
        """ Reads a register from a given servo.

        :param int servo_id: the servo id
        :param int reg_addr: the register address
        :returns: the register value
        :rtype: int
        :raises ValueError: if asking to read from the broadcasting id
        """
        if servo_id == self.BROADCASTING_ID:
            raise ValueError('id cannot be broadcast one for a read')

        reg_size = Register.size(reg_addr)
        instruction = [Instruction.READ_DATA, reg_addr, reg_size]
        data = self.write_instruction(servo_id, instruction)
        if reg_size == 1:
            return data[0]
        else:
            return data[0] + (data[1] << 8)

    def read_interface_register(self, reg_addr):
        """ Reads a register belonging to the interface itself.

        By default, this is just a shorthand for the normal register read,
        but passing the interface id as the servo one. Sub-classes can
        override it to add some checking (for instance registers not
        defined for the interface)
        """
        return self.read_register(self.INTERFACE_ID, reg_addr)

    def write_register(self, servo_id, reg_addr, value, immediate=True, clamped=False):
        """ Writes a register to a given servo.

        :param int servo_id:the servo id
        :param in reg_addr: the register address
        :param int value: the register value
        :param bool immediate: if True (default), an immediate write is used, otherwise a reg_write
        :param bool clamped: if True, the value is clamped in the register domain. If False, a ValueError
        exception is raised if the value is out of bounds
        :raises ValueError: if the value is outside the allowed range for the register and clamped
        option not used, or if the register is read-only
        """
        if not Register.is_writeable(reg_addr):
            raise ValueError('read-only register (%d)' % reg_addr)
        if clamped:
            value = Register.clamp_value(reg_addr, value)
        else:
            Register.check_value(reg_addr, value)

        inst = [Instruction.WRITE_DATA if immediate else Instruction.REG_WRITE,
                reg_addr, value & 0xff]
        if Register.size(reg_addr) != 1:
            inst.append(value >> 8)
        self.write_instruction(servo_id, inst)

    def write_interface_register(self, reg, value):
        """ Same as read_interface_register but for writing."""
        self.write_register(self.INTERFACE_ID, reg, value)

    def reg_write_register(self, servo_id, reg, value):
        """ Writes a register in registered (ie delayed) mode.

        Shorthand for :py:meth:`write_register(servo_id, reg, value, immediate=False)`
        """
        self.write_register(servo_id, reg, value, False)

    def write_registers(self, servo_id, reg_start, values, immediate=True):
        """ Writes a set of contiguous registers to a given servo.

        ..note:
            No checking is done before sending the instruction for involved
            registers write access and value range validity, as done in
            single writes. If the resulting instruction is invalid, this
            will trigger an error return status from the servo.

        :param int servo_id: the servo id
        :param int reg_start: the address of the first register to be written
        :param values: an iterable containing the values of the registers
        :param bool immediate: if True (default), an immediate write is used, otherwise a reg_write
        :raises ValueError: if values parameter is not iterable
        """
        reg = reg_start
        _bytes = []
        for val in values:
            _bytes.append(val & 0xff)
            sz = Register.size(reg)
            if sz == 2:
                _bytes.append((val >> 8) & 0xff)
            reg += sz

        try:
            self.write_instruction(servo_id,
                                   [Instruction.WRITE_DATA
                                    if immediate
                                    else Instruction.REG_WRITE, reg_start
                                    ] + _bytes
                                   )
        except TypeError:
            raise ValueError('values parameter must be iterable')

    def reg_write_registers(self, servo_id, reg_start, values):
        """ Writes a set of contiguous registers to a given servo in delayed mode.

        Shorthand for :py:meth:`write_registers(servo_id, reg_start, values, immediate=False)`
        """
        self.write_registers(servo_id, reg_start, values, False)

    def action(self):
        """ Executes all pending registered writes."""
        self.write_instruction(self.BROADCASTING_ID, [Instruction.ACTION])

    def ping(self, servo_id):
        """ Pings a given servo.

        :param int servo_id: the id of the servo to ping
        :returns: True if replied, False otherwise
        :rtype: bool
        """
        try:
            self.write_instruction(servo_id, [Instruction.PING])
            return True
        except DmxlError:
            return False

    def sync_write(self, reg_start, data):
        """ Synchronous write of a set of contiguous registers to a set of servos.

        Data to be written must be provided as a collection of tuples, each one
        containing the id of the target servos and the values to be written into
        the registers, starting from reg_start. Register values must themselves be
        a collection of values, all value collections being of the same length of
        course (will raise a ValueError otherwise).

        Beware when using tuples as collections that single item ones must include
        a comma before the closing paren, otherwise they will be considered as a scalar.

        ..note:
            As for multiple writes, no individual checking is done for the write access
            of involved registers and for the compliance of provided values with allowed
            ranges

        Example:
            # writes registers ReturnDelay, CWAngleLimit and CCWAngleLimit for servos
            # with ids 1, 2 and 3

            bus.sync_write(
                dmxl_lib.Register.ReturnDelay,
                (
                    (1, (0, 0x10, 0x200)),
                    (2, (0, 0x20, 0x170)),
                    (3, (0x10, 0x30, 0x150))
                )
            )

        :param int reg_start: the address of the first register to be written
        :param data: the data to be written
        :type data: list of tuple
        :raises ValueError: if data parameter has not the expected structure
        """
        if not hasattr(data, '__iter__'):
            raise ValueError('data must be iterable')

        _, values_0 = data[0]
        try:
            reg_count = len(values_0)
        except TypeError:
            raise ValueError('write data must be iterable')
        reg = reg_start
        reg_size = []
        for _ in values_0:
            sz = Register.size(reg)
            reg += sz
            reg_size.append(sz)

        inst = [Instruction.SYNC_WRITE, reg_start, reg - reg_start]
        for servo_id, values in data:
            try:
                if len(values) != reg_count:
                    raise ValueError('value list size mismatch')
            except TypeError:
                raise ValueError('reg values must be iterable')

            inst.append(servo_id)
            for val, sz in zip(values, reg_size):
                inst.append(val & 0xff)
                if sz == 2:
                    inst.append((val >> 8) & 0xff)

        self.write_instruction(self.BROADCASTING_ID, inst)

    def scan(self, first=1, last=253):
        """ Scans the bus to find available servos.

        :param int first, last: the bounds of the id range to explore. By default, scans the maximum possible range,
        which can take a bit if time. Specify real bounds will speed up things a lot.
        :returns: the list of the ids of found servos
        :rtype: list
        """
        saved_timeout = self._serial.getTimeout()
        self._serial.setTimeout(0.05)
        try:
            return [servo_id for servo_id in range(first, last + 1) if self.ping(servo_id)]

        finally:
            self._serial.setTimeout(saved_timeout)

    def dump_regs(self, servo_id):
        """ Dumps all registers of a given target in a user friendly format and returns the result as
        a multi-lines string.

        :param int servo_id: id of the target servo (or interface)
        :returns: the registers dump
        :rtype: str
        """
        return '\n'.join(
            (Register.dumps(reg, self.read_register(servo_id, reg)) for reg in Register.all_regs.iterkeys())
        )


class USB2Dynamixel(DynamixelBusInterface):
    """ The USB2Dynamixel interface, with appropriate defaults for the constructor."""

    def __init__(self, port='/dev/ttyUSB0', **kwargs):
        DynamixelBusInterface.__init__(self, port, **kwargs)


class USB2AX(DynamixelBusInterface):
    """ Xevel's USB2AX v3.x interface.

    It adds some extra features, such as the SYNC_READ.

    See https://paranoidstudio.assembla.com/wiki/show/paranoidstudio/USB2AX for details
    """

    class Instruction(Instruction):
        """ Instruction set specific extension. """
        BOOT_LOADER = 8
        SYNC_READ = 0x84

    def __init__(self, port='/dev/ttyACM0', **kwargs):
        DynamixelBusInterface.__init__(self, port, **kwargs)

    def read_interface_register(self, reg_addr):
        if reg_addr > Register.Id:
            raise ValueError('reg %d (%s) is not defined for interface' % (reg_addr,
                                                                           Register.label(reg_addr)))
        return self.read_register(self.INTERFACE_ID, reg_addr)

    def write_interface_register(self, reg, value):
        raise DmxlError('interface registers are read-only')

    def sync_read(self, servo_ids, reg_addr):
        """ Reads a register from several servos in one command.

        :param list servo_ids: the list of servo ids
        :param int reg_addr: the register to be read
        :returns: the list of register values
        :rtype: list
        """
        reg_size = Register.size(reg_addr)
        inst = [self.Instruction.SYNC_READ, reg_addr, reg_size] + servo_ids
        reply = self.write_instruction(self.INTERFACE_ID, inst)

        # assemble the returned value set depending of the register size
        if reg_size == 1:
            return reply
        else:
            return [reply[i] + (reply[i + 1] << 8) for i in range(0, len(reply), 2)]