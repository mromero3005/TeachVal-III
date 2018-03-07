import math
import array
'''
Created on Feb 22, 2018

@author: ggcuser
'''

class Location(object):
    '''
    classdocs
    '''
    C = 2*math.pi  # of radians
    B = 7072/C  # # of motor steps in 1 radian
    S = 7072/C
    E = 4158/C
    W = 1536/C

    H, L, LL = 7.68, 7.00, 3.8  # lengths of arms and height of shoulder
    X, Y, Z, PITCH, ROLL = 0.0  # cartesian values

    global BASE, SHOULDER, ELBOW, RIGHTWRIST, LEFTWRIST, GRIPPER
    BASE, SHOULDER, ELBOW, RIGHTWRIST, LEFTWRIST, GRIPPER = 0
    NAME = ""
    error = ""
    valid = True
    OS = 1.625

    def __init__(self, x, y, z, p, r, n):
        X = x - Location.OS
        Y = y
        Z = z
        PITCH = p
        ROLL = r
        BASE = self.findBase((X + Location.OS), Y)
        RR = self.getRR(X+Location.OS, Y)
        R0 = self.getR0(RR, PITCH)
        alpha = self.findAlpha(RR, R0, Z, PITCH)
        beta = self.findBeta(RR, R0, Z, PITCH)
        SHOULDER = self.findShoulder(alpha, beta)
        ELBOW = self.findElbow(alpha, beta)
        RIGHTWRIST = self.findRightWrist(PITCH, ROLL)
        LEFTWRIST = self.findLeftWrist(PITCH, ROLL)
        NAME = n

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
    
    def cartToSteps(self, x, y, z, r, p, r1):
        import serial
        import time
        ser = serial.Serial('/dev/tty.usbserial')  #For Mac
        #ser = serial.Serial('/dev/ttyUSB0')  #For Linux
        #ser = serial.Serial('com4')  #com4 first on left, com3 first on right. For Windows
        ser.close()
        ser.baudrate = 9600
        ser.open()
        base = math.atan(y/x)
        RR = math.sqrt(x**2 + y**2)
        o5 = p + r + r1 * base
        o4 = p - r - r1 * base
        r0 = RR - Location.LL * math.cos(p)
        z0 = z - Location.LL * math.sin(p) - Location.H
        beta = math.atan(z0/r0)
        alpha = math.atan(4*Location.L**2/(r0**2 + z0**2) - 1 )
        shoulder = alpha + beta
        elbow = beta - alpha
        
        ser.write('@STEP 240,base,0,0,0,0,0,0\r')
        time.sleep(3)  # in seconds
        
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
class RobotComm:
    import serial
    #ser = serial.Serial('/dev/ttyUSB0')  #For Linux
    ser = serial.Serial('/dev/tty.usbserial')  #For Mac
    #ser = serial.Serial('com4')  #com4 first on left, com3 first on right. For Windows

    def step(self, SP, B, S, E, R, L, G):
        return '@STEP '+SP+' '+B+' '+S+' '+E+' '+R+' '+L+' '+(G+E)+'\r'



#############################################################################End of RobotComm class   

        class Microbot:
    
            import serial
            import time
    
            Home = Location(5, 0, 0, -90, 0, "HOME")
            FAST = 240
            MEDIUM = 190
            SLOW = 140
            speed = FAST
            
    
    
    
    


    def main(self):
        #self.move(0, 0, 0, 0, 0)
        self.step()

        if __name__ == '__main__':

            self.main()
