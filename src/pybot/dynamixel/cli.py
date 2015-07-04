# -*- coding: utf-8 -*-
#
# dmxl_cli.py
#
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
#
#

""" Common definitions for command line interface dmxl tools """

__author__ = "Eric Pascual (eric@pobot.org)"

# from __future__ import print_function
import os
import argparse
from . import core as dmxl


if os.name == 'posix':
    DFLT_PORT = '/dev/ttyUSB0'
else:
    DFLT_PORT = 'COM1'


def dmxl_id(s):
    """Servo id argparse option value checker.

    The normal usage of this function is as an option type specifier when
    definition an option argument, but it can also be used as a value converter.

    Arguments:
        s:
            the option value as provided in the command line

    Returns:
        the id as an integer

    Raises:
        argparse.ArgumentTypeError if invalid value passed
    """

    try:
        dmxlid = int(s)
        if dmxlid not in range(1, 255):
            raise argparse.ArgumentTypeError('value not in range [1..254]')
        return dmxlid

    except ValueError:
        raise argparse.ArgumentTypeError('not a valid integer (%s)' % s)


def dmxl_regnum(s):
    """Servo register number argparse option value checker.

    See dmxl_id function for detailed documentation.
    """

    try:
        intval = int(s)
    except ValueError:
        raise argparse.ArgumentTypeError('not a valid integer (%s)' % s)

    dmxl.Register.check_id(intval)
    return intval


def add_bus_argparse_argument_group(parser):
    """Adds common options to an argparse parser being defined.

    Added options are:
        - port on which the bus interface is connected
        - baud rate
        - time out

    They are gathered in a group named 'Bus options'.

    Arguments:
        parser:
            the parser being defined
    """
    group = parser.add_argument_group('Bus options')
    group.add_argument('-p', '--port', dest='port',
                       help='the serial port of the bus interface',
                       default=DFLT_PORT)
    group.add_argument('-b', '--baudrate', dest='baudrate', type=int,
                       help='the bus speed',
                       default=1000000)
    group.add_argument('-T', '--timeout', dest='timeout', type=float,
                       help='bus timeout (in secs.)',
                       default=0.05)


def add_argparse_general_options(parser):
    """ Adds common general options.

    Added options are:
        - verbose

    Arguments:
        parser:
            the parser being defined
    """
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help='verbose output')
    parser.add_argument('-D', '--debug', dest='debug', action='store_true',
                        help='debug mode (will trace communications)')


def get_argument_parser(**kwargs):
    """ Returns a command line parser initialized with common settings.

    Settings used :
    - general options as defined by add_argparse_general_options
    - Dynamixel bus options as defined by add_bus_argparse_argument_group

    The parser is also set for displaying default values in help
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        **kwargs
    )
    add_argparse_general_options(parser)
    add_bus_argparse_argument_group(parser)
    return parser
