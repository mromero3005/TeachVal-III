import math
import array
'''
Created on Feb 22, 2018

@author: ggcuser
'''
from numpy import sign

class Location:
    '''
    classdocs
    '''
    C = 2*math.pi  # of radians
    B = 7072/C  # # of motor steps in 1 radian original 7072
    S = 7072/C
    E = 4158/C
    W = 1536/C
    S6 = 2330/C
    
    
    S1 = 7072/C  # # of motor steps in 1 radian original 7072
    S2 = 7072/C
    S3 = 4158/C
    S4 = 1536/C
    S5 = S4
    S6 = 2330/C
    
    P1 = 0 #page 157 line 133
    P2 = -508
    P3 = 1162
    P4 = 364
    P5 = P4
    P6 = 0

    H, L, LL = 7.625, 7.00, 3.8  # lengths of arms and height of shoulder
    X, Y, Z, PITCH, ROLL = 0,0,0,0,0 # cartesian values

    global BASE, SHOULDER, ELBOW, RIGHTWRIST, LEFTWRIST, GRIPPER
    BASE, SHOULDER, ELBOW, RIGHTWRIST, LEFTWRIST, GRIPPER = 0,0,0,0,0,0
    NAME = ""
    error = ""
    valid = True
    OS = 1.625

#     def __init__(self, x, y, z, p, r, n):
#         X = x - Location.OS
#         Y = y
#         Z = z
#         PITCH = p
#         ROLL = r
#         BASE = self.findBase((X + Location.OS), Y)
#         RR = self.getRR(X+Location.OS, Y)
#         R0 = self.getR0(RR, PITCH)
#         alpha = self.findAlpha(RR, R0, Z, PITCH)
#         beta = self.findBeta(RR, R0, Z, PITCH)
#         SHOULDER = self.findShoulder(alpha, beta)
#         ELBOW = self.findElbow(alpha, beta)
#         RIGHTWRIST = self.findRightWrist(PITCH, ROLL)
#         LEFTWRIST = self.findLeftWrist(PITCH, ROLL)
#         NAME = n

    ################ Forward Solutions from motor steps to cartesian ###############3
    def findX(self, b, s, e, p):
        RR = Location.L*math.cos(-s/Location.S) + Location.L*math.cos(-e/Location.E) + \
            Location.LL*math.cos(math.radians(p))
        return RR * math.cos(b/Location.B)
    
    def findY(self, b, s, e, p):
        RR = Location.L*math.cos(-s/Location.S) + Location.L*math.cos(-e/Location.E) + Location.LL*math.cos(math.radians(p))
        return RR*math.sin(b/Location.B)
    
    def findZ(self, s, e, p):
        return Location.H + (Location.L*math.sin(-s/Location.S)) + (Location.L*math.sin(-e/Location.E)) + (Location.LL*math.sin(math.radians(p)))
    
    def findP(self, r, l):
        return 0.5*((-Location.l/Location.W) + (-r/Location.W))
    
    ################### Backward solutions to convert from cartesian to motor steps ###########


    ################### My Own Backward Solution #############################################
    
    def cartToSteps(self, x, y, z, r, p):
        import serial
        import time
        # Referenced page 159 of TCM Manual
#         x = 5 + x
        r1 = 1
        ser = serial.Serial('/dev/tty.usbserial')  #For Mac
        #ser = serial.Serial('/dev/ttyUSB0')  #For Linux
        #ser = serial.Serial('com4')  #com4 first on left, com3 first on right. For Windows
        ser.close()
        ser.baudrate = 9600
        ser.open()
        if x == 0:
            base = round(sign(y) * math.pi/2) 
            print 'entered x=0 condition'
        else :
            base = round(math.atan(y/x) * Location.B)
            T1 = math.atan(y/x)
            print base
            print 'value of base in else condition = ' + str(base)
        RR = math.hypot(x, y)
        if RR < 2.25 and z < 15 :
            print 'hand too close to body'
            
        if RR > 17.8:
            print 'reach out of range'
        LW = -((math.radians(p) + math.radians(r)) * Location.W)
        RW = -((math.radians(p) - math.radians(r)) * Location.W)
        r0 = (RR - Location.LL * math.cos(math.radians(p))) - Location.H
        if r0 < 3.5 and x < 2.25 and z < 1.25:
            if p < -90 / Location.C:
                print 'hand interferes with base'
        z0 = (z - Location.LL * math.sin(math.radians(p))) - Location.H
        if r0 == 0:
            beta = sign(z0* math.pi/2)
        else:
            beta = math.atan(z0/r0)
#         alpha = math.atan(math.sqrt(((4*Location.L*Location.L)/((r0*r0)+(z0*z0))) - 1 ))
        alpha = (r0*r0) + (z0*z0)
        alpha = (4*Location.L * Location.L) / (alpha -1)
        if alpha < 0:
            print 'reach out of range for shoulder and elbow'
        else:
            math.atan(math.sqrt(alpha))
        T2 = alpha + beta
        T3 = beta - alpha
        T4 = p - r - r1 * T1
        T5 = p + r + r1 * T1
        W1 = (Location.S1 * T1 + 0.5) - Location.P1
        W2 = (Location.S2 * T2 + 0.5) - Location.P2
        W3 = (Location.S3 * T3 + 0.5) - Location.P3
        W4 = (Location.S4 * T4 + 0.5) - Location.P4
        W5 = (Location.S5 * T5 + 0.5) - Location.P5
        #line 5210 - 5250 if condition in manual
        
#         shoulder = round(-((alpha + beta) * Location.S))
        elbow = round(-((beta - alpha) * Location.E))
#         print base , shoulder, elbow
#         base = base - 0.0
#         shoulder = shoulder - -1846.0
#         elbow = elbow - -66.0
#         print 'new base =' + str(base) + 'new shoulder =' + str(shoulder) + 'new elbow =' + str(elbow)
        ser.write('@STEP 240,' + str(W1) + ',' + str(W2) + ',' + str(W3) + ',' + str(W4)+ ',' + str(W5)+ ','+ str(W3) + ',0\r')
        time.sleep(3)  # in seconds
        ser.close()
        
    #################### End my own Backward Solution ########################################
    def findBase(self, x, y):
        if x == 0.0:
            return round((math.sin(y) * (math.pi/2)))
        else:
            return round(((math.atan(y/x))*Location.B))

    def findShoulder(self, a, b):
        return round((-(a+b)*Location.S))

    def getRR(self, x, y):
        return math.hypot(x, y)

    def getR0(self, rr, p):
        return rr - Location.LL*math.cos(math.radians(p))

    def findAlpha(self, RR, R0, z, p):  # RR not used?? check later if can do without
        Z0 = (z - Location.LL*math.sin(math.radians(p))) - Location.H
        temp = math.sqrt(((4*Location.L*Location.L)/((R0*R0)+(Z0*Z0))) - 1)
        if(temp == 0.0):
            return 0.0
        return math.atan(temp)

    def findBeta(self, RR, R0, z, p):  #RR not used?? check later if can do without
        Z0 = (z - Location.LL*math.sin(math.radians(p))) - Location.H
        if(R0 == 0.0):
            return 0.0
        return math.atan(Z0/R0)

    def findElbow(self, a, b):
        return round((-(b-a)*Location.E))

    def findRightWrist(self, p, r):
        return round((-((math.radians(p) - math.radians(r))*Location.W)))

    def findLeftWrist(self, p, r):
        return round((-((math.radians(p) + math.radians(r))*Location.W)))

    def move(self, x, y, z, p, r):
        return Location.move2(Location(x, y, z, p, r, "null"))  # used "null" args

    def move2(self, loc):  # the letter
        steps = [] * 5  # make array of size 5 named steps
        steps.append(loc.getBase() - BASE)
        steps.append(loc.getShoulder() - SHOULDER)
        steps.append(loc.getRightWrist() - RIGHTWRIST)
        steps.append(loc.getLeftWrist() - LEFTWRIST)
        return steps

    def getBase(self):
        return BASE

    def getShoulder(self):
        return SHOULDER

    def getRightWrist(self):
        return RIGHTWRIST

    def getLeftWrist(self):
        return LEFTWRIST

    def getElbow(self):
        return ELBOW

    def getGripper(self):
        return GRIPPER

    def getX(self):
        return Location.X+Location.OS
    
    def getY(self):
        return Location.Y
    
    def getZ(self):
        return Location.Z
    
    def getRoll(self):
        return Location.ROLL
    
    def getPitch(self):
        return Location.PITCH
    
    def isValid(self):
        return Location.valid
    
    def getMotorStep(self):
        temp = {BASE, SHOULDER, ELBOW, RIGHTWRIST, LEFTWRIST, GRIPPER}
        return temp
    #####################################################################End Location class
# class RobotComm:
#     import serial
#     #ser = serial.Serial('/dev/ttyUSB0')  #For Linux
#     #ser = serial.Serial('/dev/tty.usbserial')  #For Mac
#     #ser = serial.Serial('com4')  #com4 first on left, com3 first on right. For Windows
# 
#     def step(self, SP, B, S, E, R, L, G):
#         return '@STEP '+SP+' '+B+' '+S+' '+E+' '+R+' '+L+' '+(G+E)+'\r'



#############################################################################End of RobotComm class   

#         class Microbot:
#     
#             import serial
#             import time
#     
#             Home = Location(5, 0, 0, -90, 0, "HOME")
#             FAST = 240
#             MEDIUM = 190
#             SLOW = 140
#             speed = FAST
    
def main():
    #self.move(0, 0, 0, 0, 0)
    Home = (5, 0, 0, -90, 0)
    location = Location()
    location.cartToSteps(-6.0, 0.0, 0.0, -90, 0.0)
    print 'Finished operation'

if __name__ == '__main__':

    main()
