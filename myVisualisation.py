# -*- coding: cp1251 -*-
#import serial
from visual import *
from pylab import pi, array, mat, arcsin, deg2rad, cos, sin
import pywinusb.hid as hid
from struct import unpack

class Quaternion(object):
    """Quaternions for 3D rotations"""
    def __init__(self, q = (1.0,0.0,0.0,0.0)):
        self.q = array(q, dtype=float)

    def conjugate(self):
        q0, q1, q2, q3 = self.q
        return self.__class__((q0, -q1, -q2, -q3))

    def __repr__(self):
        return "Quaternion: " + self.q.__repr__()

    def __mul__(self, other):
        # multiplication of two quaternions.
        a0, a1, a2, a3 = self.q
        b0, b1, b2, b3 = other.q
        q0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3
        q1 = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2
        q2 = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1
        q3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0

        return self.__class__((q0, q1, q2, q3))

    def asDCM(self):
        """Return the rotation matrix of the (normalized) quaternion
        http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf"""
        qw, qx, qy, qz = self.q

        ##http://www.rossprogrammproduct.com/translations/Matrix%20and%20Quaternion%20FAQ.htm#Q54

        return mat([[1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],
                         [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
                         [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)]])

    def asMatrix(self):
        return mat(self.q)

    ## like this http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    def asEuler(self): 
            qw, qx, qy, qz = self.q
            head = atan2(2*qy*qw-2*qx*qz, 1-2*qy*qy-2*qz*qz)  
            att = arcsin(2*qx*qy+2*qz*qw)
            bank = atan(2*qx*qw-2*qy*qz, 1-2*qx*qx-2*qz*qz)
            return (head, att, bank)

    def fromEulerTuple(self, euler):
        hea, att, ban = euler
        c1 = cos(hea/2)
        c2 = cos(att/2)
        c3 = cos(ban/2)
        s1 = sin(hea/2)
        s2 = sin(att/2)
        s3 = sin(ban/2)
        self.q[0] = c1*c2*c3 - s1*s2*s3
        self.q[1] = s1*s2*c3 + c1*c2*s3
        self.q[2] = s1*c2*c3 + c1*s2*s3
        self.q[3] = c1*s2*c3 - s1*c2*s3

    def norm(self):
        q0, q1, q2, q3 = self.q
        return sqrt(q0**2+q1**2+q2**2+q3**2)

    def asVector(self):
        return vector(self.q[1], self.q[2], self.q[3])

    def asCoeffs(self):
        return tuple(self.q)

    def fromCoeffs(self, tupleCoeffs):
        self.q = array(tupleCoeffs)

class ObjectScene(object):
    def __init__(self):
        self.scene=display(title="9DOF quaternion visualizer")
        self.scene.range=(2,2,2)

        self.homeLabel = label(pos=(-1.7,-1.5,0),text="Point board X axis to your monitor then press 'h'",box=0,opacity=0, xoffset = 1)
        label(pos=(-1.7,-1.7,0),text="Press 'e' to exit",box=0,opacity=0, xoffset = 1)
        label(pos=(-1.7,1.8,0),text="Quaternions:",box=0,opacity=0, xoffset = 1)
        self.lq0 = label(pos=(-1.7,1.65,0),text="q0",box=0,opacity=0, xoffset = 1)
        self.lq1 = label(pos=(-1.7,1.5,0),text="q1",box=0,opacity=0, xoffset = 1)
        self.lq2 = label(pos=(-1.7,1.35,0),text="q2",box=0,opacity=0, xoffset = 1)
        self.lq3 = label(pos=(-1.7,1.2,0),text="q3",box=0,opacity=0, xoffset = 1)

        label(pos=(0,1.8,0),text="Euler angles:",box=0,opacity=0, xoffset = 1)
        self.le0 = label(pos=(0,1.65,0),text="yaw (phi)",box=0,opacity=0, xoffset = 1)
        self.le1 = label(pos=(0,1.5,0),text="pitch (theta)",box=0,opacity=0, xoffset = 1)
        self.le2 = label(pos=(0,1.35,0),text="roll (psi)",box=0,opacity=0, xoffset = 1)

        self.f = frame(pos=(-1,0,0.5))
        '''
        ax = arrow(frame=f, pos=(0,0,0), axis=(1.3,0,0), shaftwidth=0.03, color=(0,0,1))
        lx = label(frame=f, pos=(1.3,0.1,0),text="X",box=0,opacity=0, xoffset = 1, color=(0,0,1))
        ay = arrow(frame=f, pos=(0,0,0), axis=(0,1.1,0), shaftwidth=0.03, color=(0,1,0))
        ly = label(frame=f, pos=(0.1,1.0,0),text="Y",box=0,opacity=0, xoffset = 1, color=(0,1,0))
        az = arrow(frame=f, pos=(0,0,0), axis=(-0.7,-0.7,1), shaftwidth=0.01, color=(1,0,0))
        lz = label(frame=f, pos=(-0.9,-0.6,1),text="Z",box=0,opacity=0, xoffset = 1, color=(1,0,0))'''

        
        self.eng = cone(frame=self.f, pos=(0,0,0), axis=(0.2,0,0),  radius=0.05, color=(1,0,0))
        self.fusel = cylinder(frame=self.f, pos=(0.2,0,0), axis=(1,0,0),  radius=0.1, color=(0,0,1))
        self.nose = cone(frame=self.f, pos=(1.2,0,0), axis=(0.2,0,0),  radius=0.1, color=(0,0,1))
        self.wingT = box(frame=self.f, pos=(0.9,0.025,0), axis=(1,0,0),  length=0.4, width=1, height=0.05, color=(0,1,0))
        self.wingL = box(frame=self.f, pos=(0.9,-0.025,0), axis=(1,0,0),  length=0.4, width=1, height=0.05, color=(1,0,1))
        self.tailL = box(frame=self.f, pos=(0.36,0.2,-0.025), axis=(1,0,0),  length=0.3, width=0.05, height=0.3, color=(1,1,0))
        self.tailL = box(frame=self.f, pos=(0.36,0.2,0.025), axis=(1,0,0),  length=0.3, width=0.05, height=0.3, color=(0,1,1))

        self.rmY = Quaternion((  sqrt(2.0)/2.0   , 0             , sqrt(2.0)/2.0 , 0             ))
        self.rmX = Quaternion((  -sqrt(2.0)/2.0  , sqrt(2.0)/2.0 , 0             , 0             ))
        self.rmZ = Quaternion((  sqrt(2.0)/2.0   , 0             , 0             , sqrt(2.0)/2.0 ))
        self.rmC = self.rmY*self.rmX

        self.initAxis = Quaternion((0, 1, 0, 0))
        self.initUp = Quaternion((0, 0, 0, 1))

        self.hq = None

    def setQuaternionLabels(self, quaternionObject):
        self.lq0.text = 'q0 = %+10f'%quaternionObject.q[0]
        self.lq1.text = 'q1 = %+10f'%quaternionObject.q[1]
        self.lq2.text = 'q2 = %+10f'%quaternionObject.q[2]
        self.lq3.text = 'q3 = %+10f'%quaternionObject.q[3]

    def setHomeLabel(self, labelText):
        self.homeLabel.text = labelText

    def setEulerLabel(self, eulerTuple):
        head, att, bank = eulerTuple
        self.le0.text = "bank     = %+07.2f"%rad2deg(head)
        self.le1.text = "attitude = %+07.2f"%rad2deg(att)
        self.le2.text = "heading  = %+07.2f"%rad2deg(bank)

    def rotate3DQuaternion(self, quaternionObject):
        result = None
        if self.scene.kb.keys: # event waiting to be processed?
            s = self.scene.kb.getkey() # get keyboard info
            if len(s) == 1:
                if s == 'h':
                    self.hq = quaternionObject.conjugate()
                if s == 'n':
                    self.hq = None
                if s == 'e':
                    result = 'exit'
                else:
                    pass
        if self.hq:
            self.setHomeLabel("Disable home position by pressing 'n'")
            quaternionObject = self.hq*quaternionObject
        else:
            self.setHomeLabel("Point board X axis to your monitor then press 'h'")
        roll, pitch, yaw = quaternionObject.asEuler()
        self.setEulerLabel((roll, pitch, yaw))
        self.setQuaternionLabels(quaternionObject)
        res = self.rmC*quaternionObject
        axis = (res*self.initAxis)*res.conjugate()
        up = (res*self.initUp)*res.conjugate()
        self.f.axis = axis.asVector()
        self.f.up = up.asVector()
        return result

    def delete(self):
        self.scene.visible = 0


bufQuatern = Quaternion((1.0, 0.0, 0.0, 0.0))
def joyHandler(data):
    eulers = unpack("<hhh", str(bytearray(data[1:7])))
    hea, att, ban = map(deg2rad, eulers)
    bufQuatern.fromEulerTuple((hea, att, ban))
                
class CommunicationDevice(object):
    def __init__(self, devString = 'joystick', devParams = ()):
        self.devString = devString
        self.devParams = devParams
        self.device = None

    def devOpen(self):
        if self.devString == 'serial':
            print 'Using serial input (Quaternions)'
            port, speed = devParams
            self.device = serial.Serial(port, speed, timeout = 1)
        elif self.devString == 'joystick':
            print 'Using USB joystick (Euler angles)'
            try:
                self.device = hid.HidDeviceFilter(vendor_id = 0x483).get_devices()[0]
            except:
                print 'There was an error, probably joystick is not connected'
                raise Exception('No joystick connected')
            print 'Connecting to joystick:\r\n', self.device
            self.device.open()
            self.device.set_raw_data_handler(joyHandler)

    def updateQuaternion(self, quaternion):
        if self.devString == 'serial':             
            l = self.device.readline()
            got = l.split(',')
            if len(got) == 5:
                try:
                    quaternion.fromCoeffs(map(float, got[:-1]))
                except ValueError:
                    pass
                    #print 'Skipping', repr(itm)
        elif self.devString == 'joystick':
            quaternion.fromCoeffs(bufQuatern.asCoeffs())
            while not self.device.is_plugged():
                pass    
        else:
            quaternion.fromCoeffs((0.360423, 0.439679, 0.391904, 0.723317))
        return quaternion

    def devClose(self):
        self.device.close()    

def main():
    comm = CommunicationDevice('joystick')
    try:
        comm.devOpen()
    except:
        print 'Communication device error... Exiting'
        return
      
    vis = ObjectScene()
    quatern = Quaternion()
    
    while True:
        sleep(0.01)  # python window freezes whithout short sleep in my PC
        comm.updateQuaternion(quatern)
        result = vis.rotate3DQuaternion(quatern)
        if result == 'exit':
            vis.delete()
            break
    comm.devClose()
    print '\nExiting...'   
                

if __name__ == '__main__':
    main()
        
