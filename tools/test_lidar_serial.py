#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""LiDAR hardware check: serial health check + device info"""
import serial, time, sys

try:
    s = serial.Serial('/dev/rplidar', 115200, timeout=1)
    print('Port opened: %s @ %d' % (s.name, s.baudrate))

    # STOP -> RESET -> wait -> flush -> HEALTH
    s.write(b'\xA5\x25'); time.sleep(0.02)
    s.write(b'\xA5\x40'); time.sleep(0.5)
    s.reset_input_buffer()
    time.sleep(0.1)

    s.write(b'\xA5\x52'); time.sleep(0.15)
    resp = s.read(7)
    print('HEALTH resp (%d bytes): %s' % (len(resp), resp.encode('hex') if resp else 'NONE'))

    if len(resp) >= 7:
        status = ord(resp[6]) if isinstance(resp[6], str) else resp[6]
        msg = {0: 'OK', 1: 'ERROR', 2: 'WARNING'}.get(status, 'UNKNOWN(%d)' % status)
        print('Status: %s' % msg)

    # Device info
    s.reset_input_buffer()
    s.write(b'\xA5\x50'); time.sleep(0.15)
    info = s.read(20)
    print('INFO resp (%d bytes): %s' % (len(info), info.encode('hex') if info else 'NONE'))

    s.close()
    print('Done.')
except Exception as e:
    print('Error: %s' % e)
