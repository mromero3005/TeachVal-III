import serial
import time

#ser = serial.Serial('/dev/ttyUSB0')  #For Linux
ser = serial.Serial('/dev/tty.usbserial')  #For Mac
#ser = serial.Serial('com4')  #com4 first on left, com3 first on right. For Windows
ser.close()

ser.baudrate = 9600

ser.open()

#ser.write('@STEP 240,-100,-100,-300,-100,-100,-100,0\r')  #simultaneous all joints
ser.write('@STEP 240,884.0,1255.77,-37.12,0,0,0,0\r') 
time.sleep(3)
##
#ser.write('@STEP 240,100,100,300,100,100,100,0\r')  #simultaneous reverse
# ser.write('@STEP 240,-100,0,0,0,0,0,0\r')
# time.sleep(3)

# ser.write('@STEP 240,-100,0,50,0,0,0,0\r')
# time.sleep(3)

    
##ser.write('@RESET\r') #clears all stepper registers
##
##time.sleep(3)
#Fast = 240, Med = 190, Slow = 140
###Joint 1 = base Joint 2 = Shoulder
##ser.write('@STEP 240,300,0,0,0,0,0,0\r')#joint 1 move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 240,-300,0,0,0,0,0,0\r')#joint 1 reverse move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 240,0,-300,0,0,0,0,0\r')#joint 2 move @ speed '3' 141 half steps /sec
##time.sleep(3)
# ser.write('@STEP 240,0,300,0,0,0,0,0\r')#joint 2 reverse move @ speed '3' 141 half steps /sec
# time.sleep(3)
##ser.write('@STEP 183,0,0,-300,0,0,0,0\r')#joint 3 move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,300,0,0,0,0\r')#joint 3 reverse move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,-300,0,0,0\r')#joint 4 move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,300,0,0,0\r')#joint 4 reverse move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,0,-300,0,0\r')#joint 5 move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,0,300,0,0\r')#joint 5 reverse move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,0,0,-300,0\r')#joint 6 move @ speed '3' 141 half steps /sec
##time.sleep(3)
##ser.write('@STEP 183,0,0,0,0,0,300,0\r')#joint 6 reverse move @ speed '3' 141 half steps /sec
    
ser.close()
