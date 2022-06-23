#!/usr/bin/env python3

import socket
import time

tello_ip = '192.168.10.1'
tello_port = 8889

# ssid = 'Buffalo-A-CDF0' # cannot connect to A
# ssid = 'Buffalo-G-CDF0'
# password = 'geje4ubjc7ged'
ssid = 'Buffalo-G-5890'
password = '45vw7yrxidc75'

tello_address = (tello_ip , tello_port)
cmd = 'ap ' + ssid + ' ' + password
print('tello_address is', tello_address, 'cmd is', cmd)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind(tello_address)

sock.sendto('command'.encode('utf-8'), tello_address)
time.sleep(1)
# sock.sendto('ap Buffalo-A-CDF0 geje4ubjc7ged'.encode('utf-8'), tello_address)
sock.sendto(cmd.encode('utf-8'), tello_address)
# print('sent command. wait for response.')
# data, addr = sock.recvfrom(1024)
# print('received message:', data)
sock.close()