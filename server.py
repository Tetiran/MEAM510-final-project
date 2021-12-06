from inputs import get_gamepad
import socket
import math
from queue import Queue
from threading import Thread

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtCore import Qt
import PyQt5
import sys, time

UDP_IP = "192.168.1.6"
UDP_PORT = 5005
radius= 32768

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
x = 0
y = 0

def joystick():
    global x
    global y
    while 1:
        events = get_gamepad()
        for event in events:
            if (event.ev_type != "Sync"):
                if (event.code == 'ABS_X'):
                    x = event.state
                elif (event.code == 'ABS_Y'):
                    y = event.state

                r = math.sqrt(x**2 + y**2)
                
                if (r> radius):
                    y = math.floor(y * radius/r)
                    x = math.floor(x * radius/r)

class Window(QMainWindow):
    def __init__(self, positions):
       super().__init__()
       self.data = positions
       self.setGeometry(300, 300, 800, 600)
       self.setWindowTitle("Field Viewer")
       self.show()

    def paintEvent(self, event):
        w = self.width()
        painter = QPainter(self)

        #60 x 155 in field
        offset=.03*w
        f_width= .94*w
        f_height = f_width*(60/144)
        foot = f_width /12

        # draw field
        painter.setPen(QPen(Qt.black,  3, Qt.SolidLine))
        blue =QColor()
        blue.setHsv(240, 50, 255)
        red =QColor()
        red.setHsv(0, 50, 255)
        painter.fillRect(round(offset), round(offset), round(f_width/2), round(f_height), blue)
        painter.fillRect(round(offset+f_width/2), round(offset), round(f_width/2), round(f_height), red)
        painter.drawRect(round(offset), round(offset), round(f_width), round(f_height))
        painter.drawRect(round(4.5 * foot + offset),  round(offset + foot), round(3*foot), round(3*foot))
        painter.drawLine(round(offset+6*foot), round(offset), round(offset+6*foot), round(offset+5*foot))


def server():

    while True:
        ## post at 20hz
        time.sleep(.05)
        sock.sendto(bytes('x_'+str(x), 'utf-8'), (UDP_IP, UDP_PORT))
        sock.sendto(bytes('y_'+str(y), 'utf-8'), (UDP_IP, UDP_PORT))


def window():
    positions = {}
    app = QApplication(sys.argv)
    window = Window(positions)
    sys.exit(app.exec_())

t1 = Thread(target = joystick)
t2 = Thread(target = server)
t3 = Thread(target = window)
#t1.start()
t2.start()
t3.start()