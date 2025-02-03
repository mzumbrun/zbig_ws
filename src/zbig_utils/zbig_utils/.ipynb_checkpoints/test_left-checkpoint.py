#!/usr/bin/env python3
import socket
import time

# Network settings
local_ip = '192.168.8.144'  # Replace with your local IP

remote_ip = '192.168.8.232'  # Left motor
local_port = 2390
remote_port = 2390

#remote_ip = '192.168.8.173'  # Right motor
#local_port = 2490
#remote_port = 2490

# Create UDP sockets
def create_udp_socket(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket
    sock.bind((ip, port))
    return sock

# Send data over UDP
def send_data(sock, addr, data):
    sock.sendto(data.encode('utf-8'), addr)

# Receive data over UDP
def receive_data(sock, timeout=1):
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(1024)
        return data.decode('utf-8')
    except socket.timeout:
        return None

# Create sockets
local_sock = create_udp_socket(local_ip, local_port)
remote_addr = (remote_ip, remote_port)

while True:
    # Send data to Arduino
    data_to_send = 's'
    send_data(local_sock, remote_addr, data_to_send)
    
    data_to_send = 'p2.8'
    send_data(local_sock, remote_addr, data_to_send)
    
    data_to_send = 'i0.0005'
    send_data(local_sock, remote_addr, data_to_send)
    
    data_to_send = 'd.0000002'
    send_data(local_sock, remote_addr, data_to_send)
    
    # print(f"Sent: {data_to_send}")

    # Receive data from Arduino
    received_data = receive_data(local_sock)
    if received_data:
        print(f"Left command & measured rad/s and PID: {received_data}")

    #time.sleep(1)