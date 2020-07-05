"""Quadcopter Interceptor project Simple Motion Detection using frame delta
Puts bounding box in arbritraty stating place, that is the target
Runs the selected 'tracker' that the user sets as three letter value.
'IF' statements are sorted to run tracker test
time to track per frame have been logged
Press <esc> to quit.
"""
#recall: cv2.VideoCapture() does not work with with PiCamera on ribbon cable
#30 FPS on C920 Logitech webcam (this cam uses 15fps in low light automatically)
#6 to 14 ms with FullScale USB3A LWIR cam, no NUC, recall datasheet 14bit pixel arrays
#
#!/usr/bin/env python3
import cv2
import time
#from matplotlib import pyplot as plt
import os #for CPU temperature
import array #stock python array handling
import serial #serial port

readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytes
digichannel = array.array('I',Ld) #initialize a basic python array

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

def stop_loop(frame,cam):
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
#   cv2.imshow('my webcam', frame[:,:,0])
    cv2.waitKey(500)
    cv2.destroyAllWindows()
    cam.release()
    #break must be in the if statement in the loop
    
#Bitwise or is |, regular or is ||   #Bitwise and is &, regular and is &&
def parse_serin(readin):
    achannel[0]  = ((readin[1]     | readin[2]<<8)                   & 0x07FF)
    achannel[1]  = ((readin[2]>>3  | readin[3]<<5)                   & 0x07FF)
    achannel[2]  = ((readin[3]>>6  | readin[4]<<2 | readin[5]<<10)   & 0x07FF)
    achannel[3]  = ((readin[5]>>1  | readin[6]<<7)                   & 0x07FF)
    achannel[4]  = ((readin[6]>>4  | readin[7]<<4)                   & 0x07FF)
    achannel[5]  = ((readin[7]>>7  | readin[8]<<1 | readin[9]<<9)    & 0x07FF)
    achannel[6]  = ((readin[9]>>2  | readin[10]<<6)                  & 0x07FF)
    achannel[7]  = ((readin[10]>>5 | readin[11]<<3)                  & 0x07FF)
    achannel[8]  = ((readin[12]    | readin[13]<<8)                  & 0x07FF)
    achannel[9]  = ((readin[13]>>3 | readin[14]<<5)                  & 0x07FF)
    achannel[10] = ((readin[14]>>6 | readin[15]<<2 | readin[16]<<10) & 0x07FF)
    achannel[11] = ((readin[16]>>1 | readin[17]<<7)                  & 0x07FF)
    achannel[12] = ((readin[17]>>4 | readin[18]<<4)                  & 0x07FF)
    achannel[13] = ((readin[18]>>7 | readin[19]<<1 |readin[20]<<9)   & 0x07FF)
    achannel[14] = ((readin[20]>>2 | readin[21]<<6)                  & 0x07FF)
    achannel[15] = ((readin[21]>>5 | readin[22]<<3)                  & 0x07FF)
    if (readin[23] & (1<<0)):
        digichannel[0] = 1
    else:
        digichannel[0] = 0
    if(readin[23] & (1<<1)):
        digichannel[1] = 1
    else:
        digichannel[1] = 0
    if(readin[23] & (1<<2)):
        digichannel[2] = 1
    else:
        digichannel[2] = 0
    if(readin[23] & (1<<3)):
        digichannel[3] = 1
    else:
        digichannel[3] = 0

def parse_serout(achannel,digichannel):
    sendout[0] = 0x0F #valid first byte to flight controller
    sendout[1] =  (achannel[0]  & 0x07FF) & 0xFF #07FF is 11111111111 to filter value to 11bit
    sendout[2] =  (((achannel[0]  & 0x07FF) >> 8)  | ((achannel[1] << 3) & 0x07FF)) & 0xFF
    sendout[3] =  (((achannel[1]  & 0x07FF) >> 5)  | ((achannel[2] << 6) & 0x07FF)) & 0xFF
    sendout[4] =  ((achannel[2]  & 0x07FF) >> 2) & 0xFF
    sendout[5] =  (((achannel[2]  & 0x07FF) >> 10) | ((achannel[3] << 1) & 0x07FF)) & 0xFF
    sendout[6] =  (((achannel[3]  & 0x07FF) >> 7)  | ((achannel[4] << 4) & 0x07FF)) & 0xFF
    sendout[7] =  (((achannel[4]  & 0x07FF) >> 4)  | ((achannel[5] << 7) & 0x07FF)) & 0xFF
    sendout[8] =  ((achannel[5]  & 0x07FF) >> 1) & 0xFF
    sendout[9] =  (((achannel[5]  & 0x07FF) >> 9)  | ((achannel[6] << 2) & 0x07FF)) & 0xFF
    sendout[10] = (((achannel[6]  & 0x07FF) >> 6)  | ((achannel[7] << 5) & 0x07FF)) & 0xFF
    sendout[11] = ((achannel[7]  & 0x07FF) >> 3) & 0xFF
    sendout[12] = (achannel[8]  & 0x07FF) & 0xFF
    sendout[13] = (((achannel[8]  & 0x07FF) >> 8)  | ((achannel[9] << 3) & 0x07FF)) & 0xFF
    sendout[14] = (((achannel[9]  & 0x07FF) >> 5)  | ((achannel[10] << 6) & 0x07FF)) & 0xFF
    sendout[15] = ((achannel[10] & 0x07FF) >> 2) & 0xFF
    sendout[16] = (((achannel[10] & 0x07FF) >> 10) | ((achannel[11] << 1) & 0x07FF)) & 0xFF
    sendout[17] = (((achannel[11] & 0x07FF) >> 7)  | ((achannel[12] << 4) & 0x07FF)) & 0xFF
    sendout[18] = (((achannel[12] & 0x07FF) >> 4)  | ((achannel[13] << 7) & 0x07FF)) & 0xFF
    sendout[19] = ((achannel[13] & 0x07FF) >> 1) & 0xFF
    sendout[20] = ((achannel[13] & 0x07FF) >> 9  | (achannel[14] & 0x07FF) << 2) & 0xFF
    sendout[21] = ((achannel[14] & 0x07FF) >> 6  | (achannel[15] & 0x07FF) << 5) & 0xFF
    sendout[22] = ((achannel[15] & 0x07FF) >> 3) & 0xFF
    sendout[23]=0;#init digital channels byte to 0
    if digichannel[0]==1:
        sendout[23]=1
    if digichannel[1]==1:
        sendout[23]=sendout[23] | (1<<1)
    if digichannel[2]==1:
        sendout[23]=sendout[23] | (1<<2)
    if digichannel[3]==1:
        sendout[23]=sendout[23] | (1<<3)
    sendout[23] = sendout[23] & 0xFF
    sendout[24]=readin[24];#footer byte 0x00

def main():
    ser = serial.Serial(#init serial
    port='/dev/ttyAMA0',
    baudrate=100000,
    bytesize=8,
    parity='E',
    stopbits=2,
    timeout=None,
    )
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second
#    cam.set(cv2.CAP_PROP_CONVERT_RGB,1)
#    cam.set(cv2.CAP_PROP_FPS,60)
#    cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)#1280)#1920)
#    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)#720)#1080)
#    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
#    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen
#    ret_val, frame = cam.read() #warm-up frame usually takes .250sec
#    tracker = cv2.TrackerMOSSE_create() #0.040s
#    count=0
#    bbox = (220, 100, 100, 100)#init bounding box
#    ok = tracker.init(frame, bbox)#start tracker with first bbox
    while True:#*******************************************
        readin=ser.read(25)#readin 25bytes or timeout
        if readin[0]==0x0F:#SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
            parse_serin(readin)
#            print(achannel[0])
#            print(achannel[1])
#            print(achannel[2])
#            print(achannel[3])
#            time.sleep(0.01)
            parse_serout(achannel,digichannel)
#            print([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
#            print([readin[0],readin[1],readin[2],readin[3],readin[4],readin[5],readin[6],readin[7],readin[8],readin[9],readin[10],readin[11],readin[12],readin[13],readin[14],readin[15],readin[16],readin[17],readin[18],readin[19],readin[20],readin[21],readin[22],readin[23],readin[24]])
            #ser.write([readin[0],readin[1],readin[2],readin[3],readin[4],readin[5],readin[6],readin[7],readin[8],readin[9],readin[10],readin[11],readin[12],readin[13],readin[14],readin[15],readin[16],readin[17],readin[18],readin[19],readin[20],readin[21],readin[22],readin[23],readin[24]])
            ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
        ser.reset_input_buffer()
        #if digichannel[3]==1
        #    user targeting mode here
        ##readin=ser.read(25)#readin 25bytes or timeout
#        start=time.time()
#        ret_val, frame = cam.read() #read frame from camera
#        #start=how_long(start,'cam.read took ')
#        start=time.time()
#        ok, bbox = tracker.update(frame)#run tracker; ok=success or not /
#        #bbox(0)=x1 bbox(1)=y1 bbox(2)=Xwidth bbox(3)=Yheight
#        start=how_long(start,'tracker took ')
## --------- On Track Routine ----------
##            if cx < 370 and cx > 270:
##                print "On Track"
##
##            # --------- Steer Left Routine ----------
##            if cx <= 270:
##                LSteer = 270 - cx
##                SteerLeft = remap(LSteer, 0, 45, 1, 270)
##                print ("Turn Left: ", SteerLeft)           
#         # Draw bounding box
#        if ok:
#            # Tracking success
#            p1 = (int(bbox[0]), int(bbox[1]))
#            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
#            #cv2.rectangle(img,topleftcorner-x1y1, bottomrightcorner-x2y2, (color RGB 8bit), thickness, linetype, shift)
#            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
#        else :
#            # Tracking failure
#            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
#            print("Tracking failure detected")
#        # Display tracker type on frame
#        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
#        # Display result
#        cv2.imshow("Show", frame)
#        if cv2.waitKey(1) == 27:# esc to quit
#            stop_loop(frame,cam)
#            break  
#        count += 1
#        if count>1000:
#            stop_loop(frame,cam)
#            break


if __name__ == '__main__':
    main()
#python interpreter renames xyz.py program global __name__ to __main__
#if some other .py script uses thisscript.py as import thisscript.py then it's __name__ = actual name, not__main__

