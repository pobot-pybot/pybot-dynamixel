# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import time

from pybot_dynamixel import Register, DynamixelBusInterface, DmxlError
import pybot_core.log as logging

_logger = logging.getLogger(logging.abbrev(__name__))


class Joint(object):
    """ Joints are moved by servos, and this class acts as the interface
    between the user's vision based in terms of the physical aspects of the joint
    and the corresponding commands and settings of the servo. By instance, when
    dealing with position we talk about angles and not servo steps.
    """
    ORIENTATION_CCW = 1
    ORIENTATION_CW = -1

    DEFAULT_SETUP = {
        'angles_origin': 0,
        'angles_orientation': ORIENTATION_CCW,
        'angles_range': (0, 300),
        'angles_resolution': 300. / 1023,
        'speed_resolution': 360. * 114 / 1023 / 60,
    }

    _setup_is_valid = False

    def __init__(self, servo_id, bus_interface,
                 angles_origin=DEFAULT_SETUP['angles_origin'],
                 angles_orientation=DEFAULT_SETUP['angles_orientation'],
                 angles_range=DEFAULT_SETUP['angles_range'],
                 angles_resolution=DEFAULT_SETUP['angles_resolution'],
                 speed_resolution=DEFAULT_SETUP['speed_resolution'],
                 servo_setup=None):
        """
        :param int servo_id: the id of the joint servo
        :param DynamixelBusInterface bus_interface: the Dynamixel interface used to talked to the servos
        :param int angles_origin: the servo position corresponding to 0 degrees
        (default : 0)
        :param int angles_orientation: the direction in which angles are counted positive
        (default: counter-clockwise, ie trigonometric)
        :param tuple angles_range: a tuple containing the valid range for angles
        (default: (0, 300) which is the max travel for AX12 and alike)
        :param float angles_resolution: degrees per position register unit
        (default: AX12 value, is 1023 steps for 300 degrees)
        :param float speed_resolution: degrees/sec per speed register unit
        (default: AX12 value, is 1023 steps for 114 rpm)
        :param dict servo_setup: a dictionary containing the values to be set to the servo registers.
        Keys are the register names (see :py:class:`.core.Register` class)
        (default: None)
        :raises ValueError if parameters out of range, of if inconsistency in angles related settings
        (origin, orientation and range).
        """
        if not 1 <= servo_id <= 253:
            raise ValueError('invalid servo id')
        if not (bus_interface and isinstance(bus_interface, DynamixelBusInterface)):
            raise TypeError('invalid or missing bus_interface parameter')

        self._logger = _logger.getChild('joint.%d' % servo_id)

        self._servo_id = servo_id
        self._bus = bus_interface

        self.angles_origin = angles_origin
        self.angles_orientation = angles_orientation
        self.angles_range = angles_range
        self.angles_resolution = angles_resolution
        self.speed_resolution = speed_resolution

        self.servo_setup = {
            Register.ReturnDelay: 0,
            Register.TorqueLimit: 800,
            Register.MovingSpeed: 0x3ff,
            Register.TorqueEnable: False
        }

        if servo_setup:
            self.servo_setup.update(servo_setup)

        if self.angles_orientation == Joint.ORIENTATION_CCW:
            self.servo_setup[Register.CWAngleLimit] = self.angle_to_pos(angles_range[0])
            self.servo_setup[Register.CCWAngleLimit] = self.angle_to_pos(angles_range[1])
        else:
            self.servo_setup[Register.CCWAngleLimit] = self.angle_to_pos(angles_range[0])
            self.servo_setup[Register.CWAngleLimit] = self.angle_to_pos(angles_range[1])

        if self.servo_setup[Register.CWAngleLimit] >= self.servo_setup[Register.CCWAngleLimit]:
            msg = 'inconsistency in setup of angles orientation, origin and range'
            self._logger.error(msg)
            raise ValueError(msg)

        self._setup_is_valid = True
        try:
            self.apply_servo_setup()
        except DmxlError as e:
            msg = 'servo communication error'
            self._logger.error("%s (%s)", msg, e)
            raise DmxlError(msg)

        if self._logger.isEnabledFor(logging.DEBUG):
            self._logger.debug('Joint(id=%d) initialized' % self._servo_id)
            self._logger.debug('- setup:')
            for k, v in ((k, self.servo_setup[k]) for k in sorted(self.servo_setup.keys())):
                self._logger.debug('  + ' + Register.dumps(k, v))
            self._logger.debug(' - servo registers:')
            for k in sorted(Register.all_regs.keys()):
                v = self.read_register(k)
                self._logger.debug('  + ' + Register.dumps(k, v))

    def write_register(self, register, value, immediate=True, clamped=False):
        self._bus.write_register(self._servo_id, register, value, immediate, clamped)

    def read_register(self, register):
        return self._bus.read_register(self._servo_id, register)

    @property
    def servo_id(self):
        return self._servo_id

    @staticmethod
    def _dump_setup(setup):
        return '\n'.join((Register.dumps(k, v) for k, v in ((k, setup[k]) for k in sorted(setup.keys()))))

    def dump_regs(self):
        self._bus.dump_regs(self._servo_id)

    def apply_servo_setup(self):
        if self._setup_is_valid:
            for reg, value in self.servo_setup.iteritems():
                self.write_register(reg, value)

        else:
            raise RuntimeError('cannot apply invalid settings')

    def angle_to_pos(self, angle):
        """ Converts a position expressed in degrees and wrt the configuration of the servo into its equivalent
        expressed in servo units.
        :param float pos: position in degress
        :return: position in servo units
        :rtype: int
        """
        low, high = self.angles_range
        if not low <= angle <= high:
            self._logger.warning('[%d] angle (%.1f) clamped to range [%.1f,%.1f],', self._servo_id, angle, low, high)
            angle = min(high, max(low, angle))

        return max(0,
                   min(1023,
                       int(round(angle / self.angles_resolution * self.angles_orientation)
                           + self.angles_origin)
                       )
                   )

    def pos_to_angle(self, pos):
        """ Converts a position expressed in servo units into its equivalent in expressed in degrees and
        wrt the configuration of the servo.
        :param int pos: position in servo units
        :return: position in degress
        :rtype: float
        """
        return (pos - self.angles_origin) * self.angles_resolution * self.angles_orientation

    def set_goal_angle(self, angle, speed=None, immediate=True, wait=False):
        """ Sets the target position, given in degrees.

        The moving speed can be specified as degrees per second. If not, the last set speed
         is used.
        :param float angle: target angle in degrees (wrt configured range settings)
        :param float speed: optional speed in degrees per second
        :param bool immediate: if True the movement is executed at once, otherwise it is registered for
        execution on next "execute" command.
        :param bool wait: if True, the method waits for the position is reached or the servo has stopped
        :return: the target position in register units
        :rtype: int
        """
        pos = self.angle_to_pos(angle)
        if speed:
            speed_reg_value = abs(int(speed / self.speed_resolution))
            self.write_register(Register.MovingSpeed, value=speed_reg_value, clamped=True)
        self.write_register(Register.GoalPosition, pos, immediate)

        if wait:
            while self.is_moving():
                time.sleep(0.1)

        return pos

    def get_current_angle(self):
        """ Returns the current position, in degrees and respective to the position range which is configured
        for the joint.
        """
        pos = self.read_register(Register.CurrentPosition)
        return self.pos_to_angle(pos)

    def is_moving(self):
        """ Returns True id the joint is currently moving.
        """
        return self.read_register(Register.Moving)

    def enable_torque(self, state):
        """ Enable/disable torque application, i.e. hold the current position or not.
        :param bool state: apply torque or not
        """
        self.write_register(Register.TorqueEnable, state)

    def in_range(self, angle):
        """ Returns if a given angle is within bounds set for this joint.
        :param float angle: the angle to check
        :return: True if angle in range, False otherwise
        """
        low, high = self.angles_range
        return low <= angle <= high


class JointsController(object):
    """ A controller of a collection of joints powered by servos.
    """
    def __init__(self, bus_interface):
        """
        :param bus_interface: the Dynamixel bus interface
        """
        self._interface = bus_interface
        self._joints = {}

    @property
    def joints(self):
        """ The dictionary of joints attached to the controller.
        :return:
        """
        return self._joints

    def configure_joints(self, cfg):
        """ Creates the joints attached to the controller, according to the given configuration data.
        :param dict cfg: the joints configuration data, keyed by the joint names
        """
        for joint_name, joint_cfg in cfg.iteritems():
            d = Joint.DEFAULT_SETUP.copy()
            d.update(joint_cfg)
            joint = Joint(
                bus_interface=self._interface,
                **d
            )
            self._joints[joint_name] = joint

    def get_pose(self):
        """ Returns the set of the joints current position (as an angle in degrees).
        :return: the current pose
        :rtype: dict of float
        """
        return [(name, joint.get_current_angle()) for name, joint in self._joints.iteritems()]

    def enable_torque(self, enabled):
        """ Enables or disables the torque application for all the servos.
        :param bool enabled: torque application status
        """
        for joint in self._joints.itervalues():
            joint.enable_torque(enabled)

    def check_pose(self, pose):
        """ Checks if all positions included in the pose are within limits of the
        involved joints.
        :param pose: the pose to check
        :type pose: list of tuple (name, angle)
        :return: True if all positions within limits
        """
        return [name for name, angle in pose if not self._joints[name].in_range(angle)]

