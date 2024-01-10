from PyQt6.QtCore import QThreadPool, QRunnable, QTimer
from PyQt6.QtWidgets import QMainWindow, QApplication,QLabel, QVBoxLayout, QWidget, QHBoxLayout
import pyqtgraph as pg
import sys 
import os
from random import randint
import numpy as np

import socket
import struct
import queue



class udp_thread(QRunnable):
    '''
    工作线程，接受回调函数
    '''
    def __init__(self, q:queue.Queue):
        super(udp_thread, self).__init__()

        # IP
        self.udp_addr = ('192.168.7.10', 8888)
        # 设置UDP通信
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 绑定 IP
        self.udp_socket.bind(self.udp_addr)
        # 设置超时时间
        self.udp_socket.settimeout(0.5)
        # 原始数据
        self.rawbuff = []
        # 队列
        self.q  = q
    
    def run(self):
        '''
        实现run
        '''
        while True:
            try:
                self.rawbuff, addr = self.udp_socket.recvfrom(1024)
                # print(f'[From {addr[0]}:{ addr[1]}]:resive {len(self.rawbuff)} bytes')
                # 解析数据
                self.parseData()

            except socket.timeout:
                print("UDP timeout")
    
    def parseData(self):
        '''
        解析接收到的数据
        '''
         # 判断数据是否为12个
        if len(self.rawbuff) == 12:
            # 解析格式：小端，三个float
            data = struct.unpack('<fff', self.rawbuff) 
            # 存储到队列中
            self.q.put(data)



class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        # 设置窗口标题
        self.setWindowTitle("可视化")
        # 设置窗口的最小尺寸
        self.setMinimumSize(600, 400)
        self.win = pg.GraphicsLayoutWidget()
        # 抗锯齿
        pg.setConfigOptions(antialias=True)
        # 添加plot
        self.p1 = self.win.addPlot(title = "MPU6050 Data")
        # 使能网格
        self.p1.showGrid(x=True, y=True)
        # 固定尺度
        self.p1.setYRange(-180, 180, padding=0)
        # 图例
        self.p1.addLegend()
        # 添加坐标轴说明
        self.p1.setLabel('left', "<span style=\"color:red;font-size:12px\">角度 </span>")
        self.p1.setLabel('bottom', "<span style=\"color:red;font-size:12px\">时间 </span>")
        # 设置曲线样式
        self.pen1 = pg.mkPen(color='r', width = 2)
        self.pen2 = pg.mkPen(color='b', width = 2)
        self.pen3 = pg.mkPen(color='g', width = 2)
        # 绘制曲线，返回句柄
        self.curve1 = self.p1.plot(pen = self.pen1, name = "pitch")
        self.curve2 = self.p1.plot(pen = self.pen2, name = "roll")
        self.curve3 = self.p1.plot(pen = self.pen3, name = "yaw")
        
        # self.curve1.addLegend()
        # 固定数据长度
        self.data = np.empty((200, 3))
        # 其他组件
        self.lable1 = QLabel("12")
        #布局
        layout1 = QVBoxLayout()

        layout1.addWidget(self.win)
        layout1.addWidget(self.lable1)

        widget = QWidget()
        widget.setLayout(layout1)
        self.setCentralWidget(widget)

        # UDP线程和qt间的数据传输使用队列
        self.data_queue = queue.Queue(maxsize=100)
        # 创建线程池
        self.threadpool = QThreadPool()
        print(f'最大线程数：{self.threadpool.maxThreadCount()}')
        # 启动udp线程
        udp = udp_thread(self.data_queue)
        self.threadpool.start(udp)
        self.data_queue.put
        
        # 启动 定时器
        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.updataPlot)
        self.timer.start()
    
    def updataPlot(self):
        '''
        更新plot
        '''
        num = 0
        while not self.data_queue.empty():
            num = num + 1
            data = self.data_queue.get()
            self.lable1.setText(f'pitch:{data[0]:.2f},roll:{data[1]:.2f},yaw{data[2]:.2f}')
            #数据左移动1位
            self.data[:-1, :] = self.data[1:, :]
            # 更新最后一位的数据
            self.data[-1, 0] = data[0]
            self.data[-1, 1] = data[1]
            self.data[-1, 2] = data[2]

            self.curve1.setData(self.data[:,0])
            self.curve2.setData(self.data[:,1])
            self.curve3.setData(self.data[:,2])



        
    def closeEvent(self, event):
        '''
        重构closeEvent, 关闭敞窗口时，退出所有线程
        '''
        os._exit(0)


        






if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())