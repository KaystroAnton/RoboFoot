import socket

UDP_IP = "192.168.0.106"
UDP_PORT = 2000
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
socket.SOCK_DGRAM) # UDP
#sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

sock.bind(('127.0.0.1',8888))
print('get it')
data, address = sock.recvfrom(1024)
print(data)
