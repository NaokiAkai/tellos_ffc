#!/usr/bin/env python3

import socket
import time

tello_ip = '192.168.10.1'
tello_port = 8889

ssid = 'Buffalo-G-5890'
password = '45vw7yrxidc75'

tello_address = (tello_ip , tello_port)
cmd = 'ap ' + ssid + ' ' + password
print('tello_address is', tello_address, 'cmd is', cmd)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.sendto('command'.encode('utf-8'), tello_address)
time.sleep(1)
sock.sendto(cmd.encode('utf-8'), tello_address)
sock.close()