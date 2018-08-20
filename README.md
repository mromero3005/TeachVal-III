# TeachVal-III
TeachVal-III is a python program that allows for manipulation of the Microbot TeachMover robot arm. The existing TeachVal-II was written in Java and serial port communication has been phased out of newer Java releases. This led me to chose Python to continue using the TeachMover robot with current serial port support. PySerial is the library that we use for serial port communication and this program works with Mac, Linux and Windows. 

What is needed to run this program:
1. Install Python 2.7.xx https://www.python.org/downloads/
2. Install pySerial module for serial port communication https://pythonhosted.org/pyserial/pyserial.html#installation
3. If on Windows set Python path in Environment variable Step: My Computer > Properties > Advanced System Settings > Environment Variables > Just add the path as C:\Python27 (or wherever you installed Python)
4. Run location.py in your favorite Python compatible IDE or command line if you prefer.
