#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  dmxl-setid.py
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


""" Sets the id of a connected Dynamixel servo.

A broadcast command is used to allow setting a new id without having 
to know the current one, and thus the command will work only if there
is a single servo connected to the bus.
"""

from pybot.dynamixel.dmxl_bus_intf import USB2AX, USB2Dynamixel, DynamixelBusInterface, Register
from pybot.dynamixel.cli import get_argument_parser, dmxl_id
from pybot.core import cli

__author__ = 'Eric PASCUAL (POBOT)'


def get_args():
    parser = get_argument_parser(description=__doc__)

    group = parser.add_argument_group('Command options')
    group.add_argument(
        '-i', '--newid',
        dest='newid',
        type=dmxl_id,
        required=True,
        help='the new id to set'
    )

    group.add_argument(
        '-F', '--force',
        action='store_true',
        dest='force',
        help='if used, no check will be done to see if more than one servo is connected'
    )

    return parser.parse_args()


def run_script(args):
    intf_class = USB2AX if args.xevel else USB2Dynamixel
    try:
        intf = intf_class(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
    except Exception as e:
        cli.die(e)
    else:
        if not args.force:
            print('Scanning bus on %s to find connected servos...' % args.port)
            nb_servos = len(intf.scan())

            if nb_servos == 0:
                cli.print_err('no servo found on the bus')
                return 1

            if nb_servos > 1:
                cli.print_err('only ONE servo must be connected to the bus (%d servos found)' % nb_servos)
                return 1

        intf.write_register(DynamixelBusInterface.BROADCASTING_ID, Register.Id, args.newid)
        print("Servo id changed to %d." % args.newid)
        return 0


def main():
    return run_script(get_args())

if __name__ == '__main__':
    main()
