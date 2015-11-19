#!/usr/bin/env python
#
# -*- coding: utf-8 -*-
#
# formatter.py - format html from cplusplus.com to groff syntax
#
# Copyright (C) 2010 - 2015  Wei-Ning Huang (AZ) <aitjcize@gmail.com>
# All Rights reserved.
#
# This file is part of cppman.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

import argparse
import fcntl
import Queue
import serial
import struct
import subprocess
import sys
import termios
import time

READ_SCRIPT = """
from pyb import Pin, ADC
from time import sleep

adc = ADC(Pin('%(pin)s'))
while True:
  print(adc.read() * 3.3 / 4096)
  sleep(1.0 / %(frequency)f)
"""

GNUPLOT_SCRIPT = """
set terminal dumb %(size)s
set autoscale

set xrange [-%(duration)d:0]
set yrange [0:3.3]
set ylabel "Voltage"

plot '-' notitle with lines
%(data)s
e
"""

class Pyboard(object):
  """Pyboard class. Taken from the micropython project."""
  def __init__(self, device, baudrate=115200):
    self.serial = serial.Serial(device, baudrate=baudrate, interCharTimeout=1)

  def close(self):
    self.serial.close()

  def read_until(self, min_num_bytes, ending, timeout=10, data_consumer=None):
    data = self.serial.read(min_num_bytes)
    if data_consumer:
      data_consumer(data)
    timeout_count = 0
    while True:
      if data.endswith(ending):
        break
      elif self.serial.inWaiting() > 0:
        new_data = self.serial.read(1)
        data = data + new_data
        if data_consumer:
          data_consumer(new_data)
        timeout_count = 0
      else:
        timeout_count += 1
        if timeout is not None and timeout_count >= 100 * timeout:
          break
        time.sleep(0.01)
    return data

  def enter_raw_repl(self):
    self.serial.write(b'\r\x03\x03') # ctrl-C twice: interrupt any running program

    # flush input (without relying on serial.flushInput())
    n = self.serial.inWaiting()
    while n > 0:
      self.serial.read(n)
      n = self.serial.inWaiting()

    self.serial.write(b'\r\x01') # ctrl-A: enter raw REPL
    data = self.read_until(1, b'raw REPL; CTRL-B to exit\r\n>')
    if not data.endswith(b'raw REPL; CTRL-B to exit\r\n>'):
      print(data)
      raise PyboardError('could not enter raw repl')

    self.serial.write(b'\x04') # ctrl-D: soft reset
    data = self.read_until(1, b'soft reboot\r\nraw REPL; CTRL-B to exit\r\n')
    if not data.endswith(b'soft reboot\r\nraw REPL; CTRL-B to exit\r\n'):
      print(data)
      raise PyboardError('could not enter raw repl')

  def exit_raw_repl(self):
    self.serial.write(b'\r\x02') # ctrl-B: enter friendly REPL

  def follow(self, timeout, data_consumer=None):
    # wait for normal output
    data = self.read_until(1, b'\x04', timeout=timeout, data_consumer=data_consumer)
    if not data.endswith(b'\x04'):
      raise PyboardError('timeout waiting for first EOF reception')
    data = data[:-1]

    # wait for error output
    data_err = self.read_until(1, b'\x04', timeout=timeout)
    if not data_err.endswith(b'\x04'):
      raise PyboardError('timeout waiting for second EOF reception')
    data_err = data_err[:-1]

    # return normal and error output
    return data, data_err

  def exec_raw_no_follow(self, command):
    if isinstance(command, bytes):
      command_bytes = command
    else:
      command_bytes = bytes(command, encoding='utf8')

    # check we have a prompt
    data = self.read_until(1, b'>')
    if not data.endswith(b'>'):
      raise PyboardError('could not enter raw repl')

    # write command
    for i in range(0, len(command_bytes), 256):
      self.serial.write(command_bytes[i:min(i + 256, len(command_bytes))])
      time.sleep(0.01)
    self.serial.write(b'\x04')

    # check if we could exec command
    data = self.serial.read(2)
    if data != b'OK':
      raise PyboardError('could not exec command')

  def exec_raw(self, command, timeout=10, data_consumer=None):
    self.exec_raw_no_follow(command);
    return self.follow(timeout, data_consumer)

  def eval(self, expression):
    ret = self.exec_('print({})'.format(expression))
    ret = ret.strip()
    return ret

  def exec_(self, command):
    ret, ret_err = self.exec_raw(command)
    if ret_err:
      raise PyboardError('exception', ret, ret_err)
    return ret

  def execfile(self, filename):
    with open(filename, 'rb') as f:
      pyfile = f.read()
    return self.exec_(pyfile)


def GetTerminalSize():
  """Retrieve terminal window size."""
  ws = struct.pack('HHHH', 0, 0, 0, 0)
  ws = fcntl.ioctl(0, termios.TIOCGWINSZ, ws)
  lines, columns, unused_x, unused_y = struct.unpack('HHHH', ws)
  return columns, lines - 2


def Oscilloscope(args):
  sciprt = READ_SCRIPT % {'pin': args.pin, 'frequency': args.frequency}
  history_size = args.duration * args.frequency
  terminal_size = '%d %d' % GetTerminalSize()

  # Clear screen
  sys.stdout.write('\033[2J')

  def Plot(history):
    data = '\n'.join(['%f %s' % ((-len(history) + i) / float(args.frequency), x)
                      for i, x in enumerate(history)])
    script = GNUPLOT_SCRIPT % {'size': terminal_size,
                               'duration': args.duration, 'data': data}
    p = subprocess.Popen('gnuplot', stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE)
    stdout, unused_stderr = p.communicate(script)
    sys.stdout.write('\033[1;1f')
    sys.stdout.write(stdout)

  nonlocal_data = {'buf': [], 'history': []}
  def read_value(b): 
    if b == '\n':
      history = nonlocal_data['history']
      buf = nonlocal_data['buf']
      buf.remove('\r')
      history.insert(0, ''.join(buf))
      history = history[:history_size]
      nonlocal_data['buf'] = []
      Plot(history[::-1])
    else:
      nonlocal_data['buf'].append(b)

  pyb = Pyboard(args.device, args.baudrate)
  pyb.enter_raw_repl()
  pyb.exec_raw(sciprt, timeout=None, data_consumer=read_value)


def main():
  parser = argparse.ArgumentParser(description='PyBoard oscilloscope')
  parser.add_argument('--device', default='/dev/ttyACM0',
            help='pyboard serial device')
  parser.add_argument('--baudrate', default=115200,
            help='serial device baurdate')
  parser.add_argument('--pin', default='X19',
            help='ADC PIN to use for measurment')
  parser.add_argument('--frequency', default=2, type=int,
            help='Sampling frequency')
  parser.add_argument('--duration', default=10, type=int,
            help='Visible duration')

  args = parser.parse_args()
  Oscilloscope(args)


if __name__ == '__main__':
  main()
