# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import json
import time
import sys

from .core import USB2AX, Register
from joints import JointsController


class GestureController(JointsController):
    def __init__(self, bus_interface):
        super(GestureController, self).__init__(bus_interface)
        self._use_sync_read = isinstance(bus_interface, USB2AX)

    def execute_gesture(self, gesture):
        """ Executes a gesture, which is a timed sequence of poses.

        The steps of the gesture are executed in sequence, the complete execution of
        the current one being waited for before processing the next one.

        Each step is composed of a pose and a delay to reach it. If the delay is null,
        the move will be executed as fast as possible.

        A pause can be inserted between two steps by setting the pose to None or to an
        empty dictionary. The delay specifies the duration of the pause.

        :param Gesture gesture: an instance of Gesture describing the move
        """
        all_ids = set()
        current_pose = dict(self.get_pose())

        for pose, delay in gesture.sequence:
            if pose:
                goals = {}
                for joint_id, goal_pos in pose.iteritems():
                    joint = self._joints[joint_id]
                    if delay:
                        speed = abs((goal_pos - current_pose[joint_id]) / delay)
                    else:
                        speed = sys.maxint
                    goals[joint.servo_id] = joint.set_goal_angle(goal_pos, speed, immediate=False)

                #  accumulate the ids of servos involved in the whole sequence
                # (we'll use this list for the final torque hold state)
                all_ids |= set(goals.keys())

                # make all pending moves to occur
                self._interface.action()

                # Wait for all the involved joints to complete their move.

                # Note that we don't use the moving status here since it seems to
                # be updated at a pace slower than real time, which leads to delays
                # between successive moves of the sequence
                if self._use_sync_read:
                    servo_ids = goals.keys()
                    goal_positions = [goals[servo_id] for servo_id in servo_ids]
                    while True:
                        current_positions = self._interface.sync_read(servo_ids, Register.CurrentPosition)
                        reached = [abs(a - b) <= gesture.tolerance
                                   for a, b in zip(goal_positions, current_positions)]
                        if all(reached):
                            break
                            # we could optimize by removing ids of servo having reached
                            # their goal positions, but there are good chances that the
                            # added computational overhead will not be balanced by the
                            # time saved by not querying these servos (apart perhaps
                            # for very crowded servo assemblies, but this has to be
                            # assessed)

                else:
                    while goals:
                        for servo_id, goal_position in goals.items():
                            error = abs(goal_position
                                        - self._interface.read_register(servo_id, Register.CurrentPosition))
                            if error <= gesture.tolerance:
                                del goals[servo_id]

                current_pose.update(pose)

            else:
                # an empty pose in the sequences means a... pause
                time.sleep(delay)

        if not gesture.hold_torque:
            for servo_id in all_ids:
                self._interface.write_register(servo_id, Register.TorqueEnable, 0)

    def execute_deprecated(self, gesture):
        """ Executes a gesture.

        The statements of the list are executed in sequence, the complete execution of
        the current one being waited for before processing the next one.

        Each statements is a dictionary containing the motion settings for each involved
        joints, joints being configured to move simultaneously.

        A motion settings is a tuple composed of :
        - a target angle
        - an optional move speed

        :param Gesture gesture: an instance of Gesture describing the move
        """

        all_ids = []

        for stmt in gesture.sequence:
            # will contain the goal positions keyed by the corresponding servo id
            motion_settings = {}

            for joint_id, joint_settings in stmt.iteritems():
                servo_id, goal_pos = self._setup_joint_move(joint_id, joint_settings)
                motion_settings[servo_id] = goal_pos

            #  accumulate the ids of servos involved in the whole sequence
            # (we'll use this list for the final torque hold state)
            for _id in [_id for _id in motion_settings.keys() if _id not in all_ids]:
                all_ids.append(_id)

            # make all pending moves to occur
            self._interface.action()

            # Wait for all the involved joints to complete their move.

            # Note that we don't use the moving status here since it seems to
            # be updated at a pace slower than real time, which leads to delays
            # between successive moves of the sequence
            if self._use_sync_read:
                servo_ids = motion_settings.keys()
                goal_positions = [motion_settings[servo_id] for servo_id in servo_ids]
                while True:
                    current_positions = self._interface.sync_read(servo_ids, Register.CurrentPosition)
                    reached = [abs(a - b) <= gesture.tolerance
                               for a, b in zip(goal_positions, current_positions)]
                    if all(reached):
                        break
                        # we could optimize by removing ids of servo having reached
                        # their goal positions, but there are good chances that the
                        # added computational overhead will not be balanced by the
                        # time saved by not querying these servos (apart perhaps
                        # for very crowded servo assemblies, but this has to be
                        # assessed)

            else:
                while motion_settings:
                    for servo_id, goal_position in motion_settings.items():
                        error = abs(goal_position
                                    - self._interface.read_register(servo_id, Register.CurrentPosition))
                        if error <= gesture.tolerance:
                            del motion_settings[servo_id]

        if not gesture.hold_torque:
            for servo_id in all_ids:
                self._interface.write_register(servo_id, Register.TorqueEnable, 0)

    def _setup_joint_move(self, joint_id, joint_settings):
        """ Internal method preparing the move of a joint involved in the pose.

        :param str joint_id: the id of the joint
        :param tuple joint_settings: the statement to be prepared (angle [, move_speed])
        :returns: id of the joint servo, goal position equivalent to the angle
        :rtype tuple:
        """
        # default speed to None if not specified
        angle, speed = joint_settings + (None,)

        joint = self._joints[joint_id]
        pos = joint.set_goal_angle(angle, speed, immediate=False)
        return joint.servo_id, pos


class InvalidMove(Exception):
    def __init__(self, from_pose, to_pose):
        super(InvalidMove, self).__init__()
        self.from_pose, self.to_pose = from_pose, to_pose

    def __str__(self):
        return "cannot move from '%s' to '%s'" % (self.from_pose, self.to_pose)


class InvalidPose(Exception):
    def __init__(self, pose):
        super(InvalidPose, self).__init__()
        self.pose = pose

    def __str__(self):
        return 'invalid pose : %s' % (self.pose,)


class Gesture(object):
    """ A gesture is a timed sequence of poses.

    It is basically built around a list of tuples, each one containing a pose and the
    delay (in seconds) used to go there from the previous one.

    Setting the delay for the first pose must take in account the current state of the
    arm to avoid move speeds above acceptable ones.

    Gestures support an `hold_torque` option for maintaining the torque applied at
    the end of the move to hold the last position.

    In addition, the `tolerance` option is used to set the threshold for end position
    reach detection.
    """
    def __init__(self, sequence, hold_torque=False, tolerance=10):
        """
        :param list sequence : the timed sequence of poses
        :param bool hold_torque: if True, actuators' torque will be maintained when the sequence is
        completed (default: False)
        :param int tolerance: the absolute value of the error between the target position and
        the current one under which a movement in progress will be considered
        as complete. Don't use 0 here since depending on the compliance
        settings of the servo, there are room for the target position
        not being exactly reached, and thus the move never being considered as
        complete. Side effect : using 1024 (or more) is equivalent to executing
        all the moves at the same time, since they will always be considered as complete.
        """
        self.sequence = sequence
        self.hold_torque = hold_torque
        self.tolerance = tolerance

    def clone(self):
        return Gesture(sequence=self.sequence[:], hold_torque=self.hold_torque, tolerance=self.tolerance)

    def as_json(self, indent=0):
        return json.dumps(
            {
                'sequence': self.sequence,
                'hold_torque': self.hold_torque,
                'tolerance': self.tolerance
            },
            indent=indent
        )

    @staticmethod
    def from_json(data):
        d = json.loads(data)
        return Gesture(**d)


class GestureEngine(object):
    """ A kindof state machine managing the possible arm gestures between poses.
    """

    def __init__(self, arm):
        self._gestures = {}
        self._current_pose = None
        self._arm = arm

    def __repr__(self):
        return '<%s pose:%s>' % (self.__class__.__name__, self._current_pose)

    def add_gesture(self, from_pose, to_pose, sequence,
                    hold_torque=False,
                    tolerance=10):
        """ Defines an arm gesture from a pose to another one.

        Parameters:
            from_pose:
                starting pose
            to_pose:
                ending pose
            sequence:
                sequence of join moves for executing the gesture
            hold_torque:
                tells if joint torque must be held after the gesture completion
            tolerance:
                the clearance to the target joint position
        """
        if from_pose not in self._gestures:
            self._gestures[from_pose] = {}
        self._gestures[from_pose][to_pose] = Gesture(sequence, hold_torque, tolerance)

    def set_gestures(self, gestures):
        self._gestures = gestures
        self._current_pose = None

    def get_current_pose(self):
        """ Returns the current arm pose. """
        return self._current_pose

    def set_current_pose(self, pose_id):
        """ Sets the current pose.

        The pose must be one of those defined by the add_gesture method,
        otherwise an InvalidPose exception is triggered.
        """
        if pose_id in self._gestures:
            self._current_pose = pose_id
        else:
            raise InvalidPose(pose_id)

    def initialize(self):
        """ Defines the initial pose and moves the arm to it.

        Must be overriden by sub-classes
        """
        raise NotImplementedError()

    def move_to(self, pose_id):
        """ Executes a gesture by moving from the current pose to the requested
        one.

        Acceptability of the gesture with respect to the current pose
        of the arm is checked, based on the gestures table derived
        from all the add_gesture calls.

        Parameters:
            pose_id:
                the target pose, which must have been defined previously
                with the add_gesture method.

        Raises:
            InvalidMove if gesture cannot be done
        """
        if not self._current_pose:
            raise RuntimeError('current pose is not defined')

        if pose_id == self._current_pose:
            return

        try:
            gesture = self._gestures[self._current_pose][pose_id]

        except KeyError:
            raise InvalidMove(self._current_pose, pose_id)

        else:
            # if logger:
            #     logger.info("moving from '%s' to '%s'" % (self._current_pose, pose_id))
            self._arm.execute_gesture(gesture)
            self._current_pose = pose_id
            # if logger:
            #     logger.info('current pose is now : %s' % self._current_pose)

