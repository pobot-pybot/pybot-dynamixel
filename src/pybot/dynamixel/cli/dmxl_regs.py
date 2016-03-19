#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  dmxl-regs.py
#
#  Copyright 2013 Eric PASCUAL <eric@pobot.org>
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

""" Displays or modifies Dynamixel servo registers. """

import re
import argparse
from textwrap import dedent

from pybot.dynamixel.dmxl_bus_intf import USB2AX, USB2Dynamixel, DynamixelBusInterface, Register
from pybot.dynamixel.cli import get_argument_parser, dmxl_id, dmxl_regnum
from pybot.core import cli

__author__ = 'Eric Pascual (POBOT)'


def get_args():
    parser = get_argument_parser(description=__doc__)

    options_grp = parser.add_argument_group('Command options')
    options_grp.add_argument(
        '-i', '--id',
        dest='id',
        type=dmxl_id,
        help='the servo id',
        default=1
    )
    options_grp.add_argument(
        '-I', '--interface',
        dest='intf',
        action='store_true',
        help='''if used, command targets the interface instead of a servo
                       (-i ignored in this case)'''
    )
    action_grp = parser.add_mutually_exclusive_group(required=True)
    action_grp.add_argument(
        '-l', '--list',
        dest='list',
        action='store_true',
        help='list all registers names and ids'
    )
    action_grp.add_argument(
        '-a', '--all',
        dest='readall',
        action='store_true',
        help='display all registers'
    )
    action_grp.add_argument(
        '-r', '--read', dest='read',
        type=dmxl_regnum,
        help='the register to read',
    )
    action_grp.add_argument(
        '-w', '--write',
        dest='write',
        type=WriteStatement.check,
        help='register write statement (syntax: %s)' % WriteStatement.SYNTAX
    )

    args = parser.parse_args()

    # unify target identification
    if args.id == DynamixelBusInterface.INTERFACE_ID:
        args.intf = True
    elif args.intf:
        args.id = DynamixelBusInterface.INTERFACE_ID

    return args


def write_register(intf, dmxlid, reg, value):
    try:
        intf.write_register(dmxlid, reg, value)
    except Exception as e:  # pylint: disable=W0703
        cli.print_err(str(e))


def display_register(intf, dmxlid, reg):
    try:
        value = intf.read_register(dmxlid, reg)
        Register.dump(reg, value)
    except RuntimeError as e:
        if dmxlid == DynamixelBusInterface.INTERFACE_ID:
            pass  # all registers are not relevant for interfaces
        else:
            cli.print_err(str(e))
            raise
    except Exception as e:
        cli.print_err(str(e))
        raise


def list_registers():
    print('AX12 registers ids and names :')
    for reg in Register:
        print ('- [%02d] %-30s' % (reg, Register.label(reg)))


def run_script(args):  # pylint: disable=W0621
    # process the 'list registers' action which involves no equipment
    if args.list:
        list_registers()
        return

    try:
        intf_class = USB2AX if args.xevel else USB2Dynamixel
        intf = intf_class(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            debug=args.debug
        )
    except Exception as e:  # pylint: disable=W0703
        cli.die(e)

    else:
        if args.intf or intf.ping(args.id):

            if args.intf:
                print('Interface register(s) :')
            else:
                print('Servo id=%s register(s) :' % args.id)

            if args.readall:
                # we want to display all registers
                intf.dump_regs(args.id)
            elif args.read is not None:
                # we want to read a specific register
                display_register(intf, args.id, args.read)
            elif args.write:
                # we want to write to a specific register
                reg, value = WriteStatement.parse(args.write)
                if reg == Register.Id:
                    print(dedent("""
    Warning : Writing to register 0x%0.2x will change the servo id.
            Use dmxl_setid.py script for doing this in a secure way.
                            """) % Register.Id
                    )
                    return
                write_register(intf, args.id, reg, value)
                display_register(intf, args.id, reg)

        else:
            cli.print_err('no servo found with id=%d' % args.id)


class WriteStatement(object):
    RE = r'^(\d+)=(\d+)$'
    SYNTAX = '<regnum>=<value>'

    @staticmethod
    def check(s):
        """ check that the passed value matches the format <regnum>=<integer_value>"""
        match = re.match(WriteStatement.RE, s)
        if match:
            reg, val = (int(x) for x in match.groups())
            try:
                Register.check_id(reg)
                if not Register.is_writeable(reg):
                    raise argparse.ArgumentTypeError('read-only register (%d)' % reg)
                Register.check_value(reg, val)
                return s
            except ValueError as e:
                raise argparse.ArgumentTypeError(e)
        raise argparse.ArgumentTypeError('invalid write statement (%s)' % s)

    @staticmethod
    def parse(s):
        return (int(x) for x in re.match(WriteStatement.RE, s).groups())


def main():
    return run_script(get_args())


if __name__ == '__main__':
    main()
