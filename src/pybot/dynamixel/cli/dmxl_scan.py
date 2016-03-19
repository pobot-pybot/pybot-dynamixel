#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  dmxl-scan.py
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


""" Scans the Dynamixel bus and reports ids of detected servos """

from textwrap import dedent

from pybot.dynamixel.dmxl_bus_intf import USB2AX, USB2Dynamixel, DynamixelBusInterface, Register
from pybot.dynamixel.cli import get_argument_parser, dmxl_id
from pybot.core import cli

__author__ = 'Eric PASCUAL (POBOT)'


def get_args():
    parser = get_argument_parser(description=__doc__)
    parser.add_argument(
        '-f', '--fromid',
        dest='from_id',
        type=int,
        help='start scan at this id',
        default=1
    )
    parser.add_argument(
        '-t', '--toid',
        dest='to_id',
        type=int,
        help='stop scan at this id',
        default=253
    )

    return parser.parse_args()


def run_script(args):
    if args.verbose:
        print(dedent("""
            accessing bus interface:
              - port     : %s
              - baudrate : %d
              - timeout  : %.3f s
              """)
              % (args.port, args.baudrate, args.timeout)
              )

    try:
        intf_class = USB2AX if args.xevel else USB2Dynamixel
        intf = intf_class(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
    except Exception as e:
        cli.die(e)
    else:
        print('Scanning bus on %s from id=%d to id=%d...' % (args.port, args.from_id, args.to_id))
        ids = intf.scan(args.from_id, args.to_id)
        if ids:
            print ('%d servo(s) found with id(s) : %s' %
                   (len(ids), ', '.join([str(_id) for _id in ids]))
                   )

        else:
            print('No servo found.')

        return 0


def main():
    return run_script(get_args())


if __name__ == '__main__':
    main()
