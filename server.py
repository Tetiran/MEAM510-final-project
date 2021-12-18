from inputs import get_gamepad
import socket
import math
from threading import Thread

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtCore import Qt
import PyQt5
import sys, time
import re

UDP_IP = "192.168.1.6"
UDP_IP_BROADCAST = "192.168.1.255"
UDP_PORT = 5005
UDP_PORT_TELEM = 5006
UDP_PORT_ROBOTS = 2510
UDP_PORT_CANS = 1510
radius= 32768

CommandSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
TelemSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

RobotSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

CanSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

RobotSocket.bind(('', UDP_PORT_ROBOTS))
CanSocket.bind(('', UDP_PORT_CANS))
TelemSocket.bind(('',UDP_PORT_TELEM))
RobotSocket.setblocking(0)
CanSocket.setblocking(0)
TelemSocket.setblocking(0)



x = 0
y = 0
Grip = True
WallFollow = False
MoveTo = False
BeaconTrack = False

MoveToX = 4000
MoveToY = 4000
MoveToControlFreq = 10
MoveToFinalDistance = 200


## config
Grip1Closed = -40
Gip1Open = 40
Grip2Closed = 180
Grip2Open = 45
TeamNumber = 7

FollowDistance = 25
FollowPGain = .005
FollowIGain = 0.0
FollowControlFreq = 20
FollowVelocity =.5
TurnDistance = 20
TurnPGain = .5
RotationDelay = 500
MoveToPGain =.2
MoveToForwardVelocity = .2
BeaconGain = .5

CANRADIUS = 2.6/2
#vive coordinates corresponding to top left corner of board  bonus square
X_MIN_SQUARE = 3220
Y_MIN_SQUARE = 3455
#vive coorindates corresponding to bottom right of board bonus square
X_MAX_SQUARE = 5210
Y_MAX_SQUARE = 5223

#vive coordinates corresponding to top left corner of board  bonus square
X_MIN_SQUARE_CANS = 4979
Y_MIN_SQUARE_CANS = 4735
#vive coorindates corresponding to bottom right of board bonus square
X_MAX_SQUARE_CANS = 2946
Y_MAX_SQUARE_CANS = 2928


cans = {}
robots = {}

window = 0

def joystick():
    BTN_TR_state = False
    BTN_SOUTH_STATE = False
    BTN_WEST_STATE = False
    BTN_WEST_EAST = False
    global x
    global y
    global Grip
    global WallFollow
    global MoveTo
    global BeaconTrack
    while 1:
        events = get_gamepad()
        for event in events:
            if (event.ev_type != "Sync"):
                if (event.code == 'ABS_X'):
                    x = event.state
                elif (event.code == 'ABS_Y'):
                    y = event.state
                elif (event.code == 'BTN_TR'):
                    BTN_TR_state= not BTN_TR_state
                    if BTN_TR_state:
                        Grip = not Grip
                elif (event.code == 'BTN_SOUTH'):
                    BTN_SOUTH_STATE= not BTN_SOUTH_STATE
                    if BTN_SOUTH_STATE:
                        WallFollow = not WallFollow
                elif (event.code == 'BTN_WEST'):
                    BTN_WEST_STATE= not BTN_WEST_STATE
                    if BTN_WEST_STATE:
                        MoveTo = not MoveTo
                elif (event.code == 'BTN_EAST'):
                    BTN_WEST_EAST= not BTN_WEST_EAST
                    if BTN_WEST_EAST:
                        BeaconTrack = not BeaconTrack

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
        offset = .03*w
        f_width = .94*w
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

        black =QColor()
        black.setHsv(255, 255, 255)

        painter.setPen(black)
        painter.setFont(QFont('Decorative', 10))
        painter.drawText(10, 15, "Gripper state {}".format("Open" if Grip else "closed"))
        

        #draw cans
        for can in cans:
            number =  int(can)
            x_coord = cans[can][0]
            y_coord = cans[can][1]

            ONE_FOOT_VIVE_X = abs((X_MAX_SQUARE_CANS-X_MIN_SQUARE_CANS)/3)
            ONE_FOOT_VIVE_Y = abs((Y_MAX_SQUARE_CANS-Y_MIN_SQUARE_CANS)/3)
            VIVE_TOP_LEFT_X = X_MIN_SQUARE_CANS + 4.5*  ONE_FOOT_VIVE_X
            VIVE_TOP_LEFT_Y = Y_MIN_SQUARE_CANS + 1.0 * ONE_FOOT_VIVE_Y

            CanXFootOffset = -(x_coord-VIVE_TOP_LEFT_X)/ONE_FOOT_VIVE_X
            CAnYFootOffset = -(y_coord-VIVE_TOP_LEFT_Y)/ONE_FOOT_VIVE_Y
            green =QColor()
            green.setHsv(170, 98, 78)
            painter.setPen(green)
            painter.drawEllipse(round(offset+foot*CanXFootOffset), round(offset+foot*CAnYFootOffset), 15, 15)
            painter.setPen(black)
            painter.setFont(QFont('Decorative', 10))
            painter.drawText(round(offset+foot*CanXFootOffset), round(offset+foot*CAnYFootOffset), str(number))

        #draw robots
        for bot in robots:
            number =  int(bot)
            x_coord = robots[bot][0]
            y_coord = robots[bot][1]
            ONE_FOOT_VIVE_X = (X_MAX_SQUARE-X_MIN_SQUARE)/3
            ONE_FOOT_VIVE_Y = (Y_MAX_SQUARE-Y_MIN_SQUARE)/3
            VIVE_TOP_LEFT_X = X_MIN_SQUARE - 4.5* ONE_FOOT_VIVE_X
            VIVE_TOP_LEFT_Y = Y_MIN_SQUARE - 1.0 * ONE_FOOT_VIVE_Y

            RobotXFootOffset = (x_coord-VIVE_TOP_LEFT_X)/ONE_FOOT_VIVE_X
            RobotYFootOffset = (y_coord-VIVE_TOP_LEFT_Y)/ONE_FOOT_VIVE_Y

            orange =QColor()
            orange.setHsv(39, 81, 97)
            painter.setPen(orange)
            painter.drawEllipse(round(offset+foot*RobotXFootOffset), round(offset+foot*RobotYFootOffset), 35, 35)
            painter.setPen(black)
            painter.setFont(QFont('Decorative', 10))
            painter.drawText(round(offset+foot*RobotXFootOffset), round(offset+foot*RobotYFootOffset), str(number))



def server(window):
    LastTime = [0,0,0]
    freq = [20, 5, 3]
    SentConfig= False
        
    while 1:
        if not SentConfig:
            CommandSocket.sendto(bytes('c_' + str(Grip1Closed) + '_'+ str(Gip1Open) + '_' 
            + str(Grip2Closed)+ '_' + str(Grip2Open)+ '_' + str(TeamNumber)+ '_' + str(FollowDistance)
            + '_' + str(FollowPGain)+ '_' + str(FollowIGain)+ '_' + str(FollowControlFreq) + '_' + str(FollowVelocity)
            + '_' + str(TurnDistance)+ '_' + str(TurnPGain) + '_' + str(RotationDelay) + '_' + str(MoveToControlFreq)
            + '_' + str(MoveToPGain) + '_' + str(MoveToFinalDistance) + '_' + str(MoveToForwardVelocity)
            + '_' + str(BeaconGain)
            , 'utf-8'), (UDP_IP, UDP_PORT))
            SentConfig = True

        ms = time.time()*1000.0
        ## post at 20hz
        if (ms> LastTime[0] + 1000/freq[0]):
            LastTime[0]=ms
            CommandSocket.sendto(bytes('x_'+str(x), 'utf-8'), (UDP_IP, UDP_PORT))
            CommandSocket.sendto(bytes('y_'+str(y), 'utf-8'), (UDP_IP, UDP_PORT))
        if (ms> LastTime[1] + 1000/freq[1]):
            LastTime[1]=ms
            CommandSocket.sendto(bytes('g_'+str(1 if Grip else 0), 'utf-8'), (UDP_IP, UDP_PORT))
        if (ms> LastTime[2] + 1000/freq[2]):
            CommandSocket.sendto(bytes('w_'+str(1 if WallFollow else 0), 'utf-8'), (UDP_IP, UDP_PORT))
            CommandSocket.sendto(bytes('m_'+str(1 if MoveTo else 0) + '_'+ str(MoveToX) + '_'+ str(MoveToY), 'utf-8'), (UDP_IP, UDP_PORT))
            CommandSocket.sendto(bytes('t_'+str(1 if BeaconTrack else 0), 'utf-8'), (UDP_IP, UDP_PORT))
            LastTime[2]=ms

        try:
            data, addr = RobotSocket.recvfrom(1024) # buffer size is 1024 bytes
            if (len(data) >= 12):
                data= data.decode('utf-8')
                num = int(data[0])
                posx = int(data[2:6])
                posy = int(data[7:11])
                print("robot message: {}".format(data))
                robots[str(num)] = (posx, posy)
                window.update()
        except socket.error as e:
            pass        
        try:
            data, addr = CanSocket.recvfrom(1024) # buffer size is 1024 bytes
            if (len(data) >= 12):
                data= data.decode('utf-8')
                num = int(data[0])
                posx = int(data[2:6])
                posy = int(data[7:11])
                cans[str(num)] = (posx, posy)
                window.update()
                print("can message: {}".format(data))
        except socket.error as e:
            pass
        try:
            data, addr = TelemSocket.recvfrom(1024) # buffer size is 1024 bytes
            if (len(data) >= 12):
                data= data.decode('utf-8')
                if(int(data[0]) == TeamNumber):
                    posx = int(data[2:6])
                    posy = int(data[7:11])
                    robots[str(TeamNumber)] = (posx, posy)
                    window.update()
        except socket.error as e:
            pass

            
                
def main():
    positions = {}
    app = QApplication(sys.argv)
    window = Window(positions)
    t1 = Thread(target = joystick)
    t2 = Thread(target = server, args=[window])
    t1.start()
    t2.start()
    sys.exit(app.exec_())
    print('yes')
  
if __name__=="__main__":
    main()

