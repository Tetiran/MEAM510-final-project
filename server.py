from inputs import get_gamepad
import socket
import math
from threading import Thread

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtCore import Qt
import PyQt5
import sys, time

UDP_IP = "192.168.1.6"
UDP_IP_BROADCAST = "192.168.1.255"
UDP_PORT = 5005
UDP_PORT_ROBOTS = 2510
UDP_PORT_CANS = 1510
radius= 32768

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

CommandSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

RobotSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

CanSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

RobotSocket.bind(('', UDP_PORT_ROBOTS))
CanSocket.bind(('', UDP_PORT_CANS))
RobotSocket.setblocking(0)
CanSocket.setblocking(0)



x = 0
y = 0
Grip = False
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
TeamNumber = 8

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

cans = []
robots = []

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
        inch = foot/12

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

        #draw cans
        for can in cans:
            number = cans[can][0]
            #fix coorindate offset and scale (might need to switch height and width depending on whatx and y corresond to)
            x_coord = (cans[can][1] - X_MIN_VIVE)/(X_MAX_VIVE - X_MIN_VIVE)*f_width
            y_coord = (cans[can][2] - Y_MIN_VIVE)/(Y_MAX_VIVE - Y_MIN_VIVE)*f_height
            painter.drawEllipse(x_coord, y_coord, round(CANRADIUS*inch), round(CANRADIUS*inch), black)
            # painter.drawText(x_coord, y_coord,  round(inch), round(inch))

        #draw robots
        for bot in robots:
            number =  robots[bot][0]
            x_coord = robots[bot][0]
            y_coord = robots[bot][0]
            painter.drawRect(x_coord, y_coord, round(foot), round(foot), black)



def server():
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
            if (len(data) == 13):
                if data.decode('utf-8')[0] == str(TeamNumber):
                    print("received message: {}".format(data.decode('utf-8')))
        except socket.error as e:
            pass        
        try:
            data, addr = CanSocket.recvfrom(1024) # buffer size is 1024 bytes
            if (len(data)):
                print("received message: {}".format(data.decode('utf-8')))
        except socket.error as e:
            pass
            
                
def main():
    t1 = Thread(target = joystick)
    t2 = Thread(target = server)
    t1.start()
    t2.start()
    positions = {}
    app = QApplication(sys.argv)
    window = Window(positions)
    sys.exit(app.exec_())
    print('yes')
  
if __name__=="__main__":
    main()

