# Test for Razor 9DOF IMU
# Jose Julio @2009
# This script needs VPhyton, pyserial and pywin modules

# First Install Python 2.6.4
# Install pywin from http://sourceforge.net/projects/pywin32/
# Install pyserial from http://sourceforge.net/projects/pyserial/files/
# Install Vphyton from http://vpython.org/contents/download_windows.html

from visual import *
import serial
import string
import math

from time import time

def quatConjugate(q):
    #q = map(float, q)
    q0, q1, q2, q3 = q
    return (q0, -q1, -q2, -q3)

def quaternionToEuler(q):
    #q = map(float, q)
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    euler = []
    #euler.append(atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1)) ## psi
    #euler.append(-asin(2 * q[1] * q[3] + 2 * q[0] * q[2])) ## theta
    #euler.append(atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1)) ## phi
    euler.append(math.atan2(2*(q0*q3+q2*q3), 1-2*(q1**2+q2**2))) ##roll
    euler.append(-math.asin(2*(q0*q2-q3*q1))) ##pitch
    euler.append(math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))) ##YAW

    return euler

def quatProd(a, b):
    #a = map(float, a)
    #b = map(float, b)
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    q0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3
    q1 = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2
    q2 = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1
    q3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0
    return (q0, q1, q2, q3)

grad2rad = 3.141592/180.0

# Check your COM port and baud rate
ser = serial.Serial(port='COM4',baudrate=115200, timeout=1)

# Main scene
scene=display(title="9DOF Razor IMU test")
scene.range=(1.2,1.2,1.2)
#scene.forward = (0,-1,-0.25)
scene.forward = (1,0,-0.25)
scene.up=(0,0,1)

# Second scene (Roll, Pitch, Yaw)
scene2 = display(title='9DOF Razor IMU test',x=0, y=0, width=500, height=200,center=(0,0,0), background=(0,0,0))
scene2.range=(1,1,1)
scene.width=500
scene.y=200

scene2.select()
#Roll, Pitch, Yaw
cil_roll = cylinder(pos=(-0.4,0,0),axis=(0.2,0,0),radius=0.01,color=color.red)
cil_roll2 = cylinder(pos=(-0.4,0,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
cil_pitch = cylinder(pos=(0.1,0,0),axis=(0.2,0,0),radius=0.01,color=color.green)
cil_pitch2 = cylinder(pos=(0.1,0,0),axis=(-0.2,0,0),radius=0.01,color=color.green)
#cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
#cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
arrow_course = arrow(pos=(0.6,0,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)

#Roll,Pitch,Yaw labels
label(pos=(-0.4,0.3,0),text="Roll",box=0,opacity=0)
label(pos=(0.1,0.3,0),text="Pitch",box=0,opacity=0)
label(pos=(0.45,0.3,0),text="Yaw",box=0,opacity=0)
label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=color.yellow)
label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=color.yellow)
label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=color.yellow)
label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=color.yellow)
label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=color.yellow)
label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=color.yellow)
label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=color.yellow)
label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=color.yellow)

L1 = label(pos=(-0.4,0.22,0),text="-",box=0,opacity=0)
L2 = label(pos=(0.1,0.22,0),text="-",box=0,opacity=0)
L3 = label(pos=(0.7,0.3,0),text="-",box=0,opacity=0)

# Main scene objects
scene.select()
# Reference axis (x,y,z)
arrow(color=color.green,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
arrow(color=color.green,axis=(0,1,0), shaftwidth=0.02 , fixedwidth=1)
arrow(color=color.green,axis=(0,0,0.5), shaftwidth=0.02, fixedwidth=1)
# labels
label(pos=(0,0,0.8),text="9DOF Razor IMU test",box=0,opacity=0)
label(pos=(1,0,0),text="X",box=0,opacity=0)
label(pos=(0,1,0),text="Y",box=0,opacity=0)
label(pos=(0,0,0.5),text="Z",box=0,opacity=0)
# IMU object
platform = box(length=1, height=0.05, width=1, color=color.red)
p_line = box(length=1.15,height=0.08,width=0.1,color=color.yellow)
plat_arrow = arrow(color=color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)


#f = open("Serial"+str(time())+".txt", 'w')
hq = None

roll=0
pitch=0
yaw=0
while 1:
    line = ser.readline()
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    words = map(float, words[:-1])
    print words
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info
        if len(s) == 1:
            if s == 'h':
                hq = quatConjugate(words)
            if s == 'n':
                hq = None
            if s == 'e':
                break
            else:
                pass
    if len(words) == 4:
        if hq:
            euler = quaternionToEuler(quatProd(hq, words))
        else:
            euler = quaternionToEuler(words)
    if len(words) > 2:
        try:
            roll = euler[0]
            pitch = euler[1]
            yaw = -euler[2]
        except:
            print "Invalid line", words

        axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
        up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
        platform.axis=axis
        platform.up=up
        platform.length=1.0
        platform.width=0.65
        plat_arrow.axis=axis
        plat_arrow.up=up
        plat_arrow.length=0.8
        p_line.axis=axis
        p_line.up=up
        cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
        cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
        cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
        cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
        arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)
        L1.text = str(float(words[0]))
        L2.text = str(float(words[1]))
        L3.text = str(float(words[2]))        
ser.close
#f.close
