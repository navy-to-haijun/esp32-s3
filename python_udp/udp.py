from time import sleep
import socket
import struct

import time
import threading
import collections
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt 


def main():
    
    
    # plt.ion()
    # y = []
    # x = []
    # # 缓存数据
    # buff_data = collections.deque(maxlen=250)   

    udp_addr = ('192.168.7.7', 8888)
    # UDP通信类型
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(udp_addr)

    while True:
        # 等待接收数据
        buff, addr = udp_socket.recvfrom(1024) 
        print(f'[From {addr[0]}:{ addr[1]}]:resive {len(buff)} bytes')
        # 判断数据是否为12个
        if len(buff) == 12:
            data = struct.unpack('<fff', buff)
            print(data)
            print()
            # plt.clf() 
            # y.append(data[0])
            # x.append(data[1])
            # plt.plot(x,y)
            # plt.pause(0.001)

            # plt.show()



        

if __name__ == '__main__':
    main()
    # xpoints = np.array([0, 6])
    # ypoints = np.array([0, 100])

    # plt.plot(xpoints, ypoints)
    # plt.show()






