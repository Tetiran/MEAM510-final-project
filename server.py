from inputs import get_gamepad
import socket
import math

UDP_IP = "192.168.1.6"
UDP_PORT = 5005
radius= 32768

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP


joy_x=0
joy_y=0
while 1:
    events = get_gamepad()
    for event in events:
        if (event.ev_type != "Sync"):
            print(event.ev_type, event.code, event.state)
            if (event.code == 'ABS_X'):
                joy_x = event.state
            elif (event.code == 'ABS_Y'):
                joy_y = event.state

            r = math.sqrt(joy_x**2 + joy_y**2)
            
            if (r> radius):
                joy_y = math.floor(joy_y * radius/r)
                joy_x = math.floor(joy_x * radius/r)
            
            sock.sendto(bytes('x_'+str(joy_x), 'utf-8'), (UDP_IP, UDP_PORT))
            sock.sendto(bytes('y_'+str(joy_y), 'utf-8'), (UDP_IP, UDP_PORT))