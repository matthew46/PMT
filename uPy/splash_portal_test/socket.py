# ---------------------------------------
#  _____  __  __ _______        __   ___  
# |  __ \|  \/  |__   __|      /_ | / _ \ 
# | |__) | \  / |  | |    __   _| || | | |
# |  ___/| |\/| |  | |    \ \ / / || | | |
# | |    | |  | |  | |     \ V /| || |_| |
# |_|    |_|  |_|  |_|      \_/ |_(_)___/ 
# ----------------------------------------
#  Version 1.0
#  microPython Firmware esp32spiram-idf3-20191220-v1.12
#  Filename : main.py

import network
import machine
import urequests
import usocket
import ujson
import utime
import ubinascii
import ssl

target_host = "www.google.com"
target_port = 80
# Create a station object to store our connection
station = network.WLAN(network.STA_IF)

# activate station
station.active(True)
station.scan()
station.connect("Lowes-Guest-WiFi","")
while not station.isconnected():
    utime.sleep(1)

# Create STREAM TCP socket
sock = usocket.socket(usocket.AF_INET,usocket.SOCK_STREAM)

# time in seconds
sock.settimeout(0.5)

# address resolving
sockaddr = usocket.getaddrinfo(target_host, target_port)

# send some data 
request = "GET / HTTP/1.1\r\nHost:%s\r\n\r\n" % target_host
try:
    sock.connect(sockaddr[0][-1])
    utime.sleep(1)
    sock.send(request.encode()) 
    # receive some data 
    response = sock.recv(4096)  
    http_response = repr(response)
    http_response_len = len(http_response)
    print(str(http_response[0:14]))
except OSError as e:
    print(e)
    