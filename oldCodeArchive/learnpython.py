import cv2
import time
#from matplotlib import pyplot as plt
import os #for CPU temperature
import array #stock python array handling
import serial #serial port

a = int
b = float

# ---- Function definition for converting scales ------
def remap(unscaled, to_min, to_max, from_min, from_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))

#---- Function Timer, returns current time
def how_long(start, activity):
    print('%s took %.3fs' % (activity, time.time()-start))
    return time.time()

def doit(in1,in2):
    out1=in1+in2
    out2=in1*in2
    return([out1,out2])

def main():
    var1,var2=doit(3,2)
    print([var1,var2])

if __name__ == '__main__':
    main()
#python interpreter renames xyz.py program global __name__ to __main__
#if some other .py script uses thisscript.py as import thisscript.py then it's __name__ = actual name, not__main__

