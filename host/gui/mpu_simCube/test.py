# # copilot
# import socket
# import time

# s = socket.socket()
# s.connect(("127.0.0.1", 5000))
# x = 0
# y = 0
# z = 0
# while True:
#     s.send(f"{x} {y} {z}".encode())
#     time.sleep(0.02)
#     x += 1
#     # y += 1
#     # z += 1

import socket, time, math

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5000))
t = 0
try:
    while True:
        # Generates smooth oscillation
        ax, ay, az = math.sin(t)*30, math.cos(t)*45, math.sin(t*0.5)*20
        s.sendall(f"{ax} {ay} {az}\n".encode())
        t += 0.05
        time.sleep(0.10)
except: s.close()