"""Quadcopter Interceptor project 
Only ATT Flight Mode achannel[4]=306
Tracker uses auto thresholding by contrast zoom by 50% in the track window scene
- no need to set a threshold or threshold width
"""
#
#tracks using only 'blue' minimum x and y magnitude point
#ccblobfinder works, but not used in tracking
#
#recall: cv2.VideoCapture() does not work with with PiCamera on ribbon cable
#30 FPS on C920 Logitech webcam (this cam uses 15fps in low light automatically)
#6 to 14 ms with FullScale USB3A LWIR cam, no NUC, recall datasheet 14bit pixel arrays
#
#this is not using python enviroments #!/usr/bin/env python3
import cv2
import time
#from matplotlib import pyplot as plt
import os #for CPU temperature
import array #stock python array handling
import serial #serial port
import numpy as np
import subprocess#for setting cam properties at shell level
readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
global achannel
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytesdigichannel = array.array('I',Ld) #initialize a basic python array
#mode = int #0 start / 1 user targeting / 2 auto track / 3 auto pilot
rstickhorz = int #right stick horz center bias (trim)
rstickvert = int #right stick vert center bias (trim)

framewidth = int(640)#int(1920)#int(640)
frameheight = int(480)#int(1080)#int(480)

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))

#---- Function Timer, returns current time
def how_long(start, activity):
    print('%s time %.3fs' % (activity, time.time()-start))
    return time.time()

def stop_loop(frame,cam):#break jumps us out of inner most loop and used in an if statement of the loop
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
#    cv2.waitKey(100)
    cam.release()
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()
    
def parse_serin(readin):#Bitwise or is |, regular or is ||   #Bitwise and is &, regular and is &&
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
#    if (readin[23] & (1<<0)):#0 for radios with 'digichannels'
#        digichannel[0] = 1
#    else:
#        digichannel[0] = 0
#    if(readin[23] & (1<<1)):#1
#        digichannel[1] = 1
#    else:
#        digichannel[1] = 0
#    if(readin[23] & (1<<2)):#2
#        digichannel[2] = 1
#    else:
#        digichannel[2] = 0
#    if(readin[23] & (1<<3)):#3
#        digichannel[3] = 1
#    else:
#        digichannel[3] = 0

def parse_serout(achannel,readin):#digichannel):
    achannel[4]=306
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
#    sendout[23]=0;#init digital channels byte to 0, for 'digichannel' radios
#    if digichannel[0]==1:
#        sendout[23]=1
#    if digichannel[1]==1:
#        sendout[23]=sendout[23] | (1<<1)
#    if digichannel[2]==1:
#        sendout[23]=sendout[23] | (1<<2)
#    if digichannel[3]==1:
#        sendout[23]=sendout[23] | (1<<3)
#    sendout[23] = sendout[23] & 0xFF
    sendout[23]=readin[23];#no digichannels used on Radiolink AT9S
    sendout[24]=readin[24];#footer byte 0x00
    return sendout
    

def return_mouse_click(event,xmouse,ymouse,flags,param):#special var return via global vars
    global tpoint
    global clickflag
    global x
    global y
    print("Mouse ",repr(event))
    if event == 1:#left mouse button depress
        clickflag=1
        tpoint[0]=xmouse
        tpoint[1]=ymouse
    return tpoint#,varBreak


def simple_tracker(tpoint,trfrmsize,frame):#tpoint is [x,y] / width is frame width / frame is total camera pict
#    trfrmsize=350#pixel area being processed makes timing difference
#    x=40.00#pixel magnitude width
#    halfx=x/2#use half magnitude to go up and down from user selected pixel target point
#    #This tracker likes to drift to origin, maybe numerical issue of Python?
#    if tpoint[0]<2:
#        tpoint[0]=int(framewidth/2)#X graph axis center
#    if tpoint[1]<2:
#        tpoint[1]=int(frameheight/2)#Y graph axis center
    #if Target is dark,low values, wrt background; then expect Target values to be low
    bx1=int(tpoint[0]-trfrmsize)
    by1=int(tpoint[1]-trfrmsize)
    bx2=int(tpoint[0]+trfrmsize)
    by2=int(tpoint[1]+trfrmsize)
    if bx1<1:
        bx1=int(1)
    if by1<1:
        by1=int(1)
    if bx2>(framewidth-1):
        bx2=int((framewidth-1))
    if by2>(frameheight-1):
        by2=int((frameheight-1))
        
    box=[int(bx1),int(by1),int(bx2),int(by2)]
    subframe=frame[int(by1):int(by2),int(bx1):int(bx2)]
    
    subframeB=subframe[:,:,0]
    subframeG=subframe[:,:,1]
    subframeR=subframe[:,:,2]
    
#only good for dark objects less
    pos=255-subframeB
    pos=np.clip(pos,0,255)
    amt=((np.amax(pos)-np.amin(pos))*.5)+np.amin(pos)
    shf=pos-np.amin(amt)

    BandPass=np.clip(shf,0,255)
    mconst=240/(np.amax(BandPass))
    BandPass=np.uint8(BandPass*mconst)
    
    sumBx=np.sum(BandPass,axis=0)#sums columns into a single row for X
    sumBy=np.sum(BandPass,axis=1)#sums rows into a single column for Y
    sBxMax=np.argmax(sumBx)#pixel x location where min occurs
    sByMax=np.argmax(sumBy)
    tpoint=[int(sBxMax+bx1),int(sByMax+by1)]

    retframe=frame
    retframe[int(by1):int(by2),int(bx1):int(bx2),0]=BandPass
    retframe[int(by1):int(by2),int(bx1):int(bx2),1]=BandPass
    retframe[int(by1):int(by2),int(bx1):int(bx2),2]=BandPass
    return tpoint,retframe,box#subframe,subframeB,subframeG,subframeR#bbox


def main():
    achannel[2]=int(307)
    achannel[6]=int(307)
    global tpoint
#    global SelctPtValB
#    global SelctPtValG
#    global SelctPtValR
#    global clickflag
    centerXcol=int(framewidth/2)#roll (quadcopter speak)
    centerYrow=int(frameheight/2)#pitch
    maxHorz=1630
    minHorz=320
    maxVert=1630
    minVert=320    
    xError1=0
    yError1=0
    xError2=0
    yError2=0
    xIntegral=0
    yIntegral=0
    xDer=0
    yDer=0
    Kp=.02
    Ki=.02
    Kd=.05
    clickflag=0#flag indicating mouse click or target selection
    loop=1#while loop control
    loopimshow=int(0)#used for counting loop interval for imshow
    mode=int(7)
    amode=int(7)
    tpoint=[1,1]
    tpointusr=[1,1]
    tpoint[0]=int(framewidth/2)#X graph axis center
    tpoint[1]=int(frameheight/2)#Y graph axis center
    box=[1,1,1,1]#box is size set by user VrBKnob during targeting; but changes to targeting area box in track mode
    trfrmsize=350#pixel area being processed
    box[0]=int(tpoint[0]-trfrmsize)#X1 for targeting box
    box[1]=int(tpoint[1]-trfrmsize)#Y1
    box[2]=int(tpoint[0]+trfrmsize)#X2 for targeting box
    box[3]=int(tpoint[1]+trfrmsize)#Y2
#    SelctPtValB=int(60)
#    SelctPtValG=int(100)
#    SelctPtValR=int(100)


    rsh = np.empty(0) #right stick horz center bias (trim)
    rsv = np.empty(0) #right stick vert center bias (trim)
    
    ser = serial.Serial(#init serial
    port='/dev/ttyAMA0',
    baudrate=100000,
    bytesize=8,
    parity='E',
    stopbits=2,
    timeout=None,
    )
    
    cam_props = {'brightness': 128,#brightness (int)    : min=0 max=255 step=1 default=-8193 value=128
                 'contrast': 128,#contrast (int)    : min=0 max=255 step=1 default=57343 value=128
                 'saturation': 128,#saturation (int)    : min=0 max=255 step=1 default=57343 value=128
                 'white_balance_temperature_auto': 0,#white_balance_temperature_auto (bool)   : default=1 value=1
                 'gain': 50,#gain (int)    : min=0 max=255 step=1 default=57343 value=0
                 'power_line_frequency': 2,#power_line_frequency (menu)   : min=0 max=2 default=2 value=2
                 'white_balance_temperature': 4000,#white_balance_temperature (int)    : min=2000 max=6500 step=1 default=57343 value=4000 flags=inactive
                 'sharpness': 128,#sharpness (int)    : min=0 max=255 step=1 default=57343 value=128
                 'backlight_compensation': 0,#backlight_compensation (int)    : min=0 max=1 step=1 default=57343 value=0
                 'exposure_auto': 1,#exposure_auto (menu)   : min=0 max=3 default=0 value=1 is manual
                 'exposure_absolute': 300,#exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=250 flags=inactive
                 'exposure_auto_priority': 1,#exposure_auto_priority (bool)   : default=0 value=1
                 'pan_absolute': 0,#pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'tilt_absolute': 0,#tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'focus_absolute': 0,#250 is near; 0 is far;focus_absolute (int)    : min=0 max=250 step=5 default=8189 value=0 flags=inactive
                 'focus_auto': 0,#focus_auto (bool)   : default=1 value=1
                 'zoom_absolute': 100}#zoom_absolute (int)    : min=100 max=500 step=1 default=57343 value=100
    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))], shell=False)
    subprocess.call(['v4l2-ctl -d /dev/video0 -l'], shell=False)
    time.sleep(0.5)
    #Do it again to make sure it the camera gets it
    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))], shell=False)
    subprocess.call(['v4l2-ctl -d /dev/video0 -l'], shell=False)
    time.sleep(0.5)
    
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second

#    cam.set(cv2.CAP_PROP_CONVERT_RGB,1)#useing regular color webcam
#    cam.set(cv2.CAP_PROP_FPS,60)#try to get 60fps from webcam
    cam.set(cv2.CAP_PROP_FRAME_WIDTH,framewidth)#1280)#1920)#reasonable resolution for fast image processing
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,frameheight)#720)#1080)
    
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    cv2.setMouseCallback('Show',return_mouse_click)#event driven hook to this window
    cv2.resizeWindow('Show',framewidth,frameheight)#640,480)
#    cv2.namedWindow("Showbbox", cv2.WINDOW_NORMAL)#
#    cv2.namedWindow("ShowDevFrame", cv2.WINDOW_NORMAL)#
##    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
    ret_val, frame = cam.read() #must init & warm-up cam, this frame usually takes .250sec
    time.sleep(.25)
    
    for i in range(3):#Init right stick center for trimmed condition
        readin=ser.read(25)#take reading 3 times, use last one to elimnate any start up noise
        if readin[0]==0x0F:#bias is needed to be known when stick used for targeting
            parse_serin(readin)
            rstickhorz=achannel[0]#-993 # horz center bias
            rstickvert=achannel[1]#-1010 # vert center bias
            print(rstickhorz,rstickvert)
        else:#default value if assessment fails
            print("0x0F not readin")
            rstickhorz=980
            rstickvert=980
            print(rstickhorz,rstickvert)
        ser.reset_input_buffer()
    valhorz=rstickhorz#init valhorz and valvert for autopilot
    valvert=rstickvert
    start=time.time()
    while loop==1:#*******************************************
        ret_val=0
        ret_val, frame = cam.read() #read frame from camera
        readin=ser.read(25)#readin 25bytes or timeout
#        print("aux3=",achannel[6])
#        print("thottle=",achannel[2])
        if readin[0]==0x0F:#Valid SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
            parse_serin(readin)
            trfrmsize=int(achannel[7]-290)
            if ret_val==1: #Frame read is good
                #Targeting Mode
                if (achannel[9]>1200):#F switch On 'user targeting mode'
                    mode=mode+1
                    if mode>10:
                        mode=11
                        xError1=0
                        yError1=0
                        xError2=0
                        yError2=0
                        xIntegral=0
                        yIntegral=0
                        xDer=0
                        yDer=0
                        rsh=int(achannel[0]-rstickhorz)
                        rsv=int(rstickvert-achannel[1])
                        if abs(rsh) > 8 :#Horizontal position adjust (use > if statement to remove noise near zero)
                            tpoint[0]=int(tpoint[0]+(rsh/15))
                            if tpoint[0] < 1:
                                tpoint[0] = int(1)
                            if tpoint[0] > framewidth:
                                tpoint[0]=int(framewidth-1)
                        if abs(rsv) > 8 :#Vertical position adjust
                            tpoint[1]=int(tpoint[1]+(rsv/15))
                            if tpoint[1] < 1:
                                tpoint[1] = int(1)
                            if tpoint[1] > frameheight:
                                tpoint[1]=int(frameheight-1)
                        x1=int(tpoint[0])-trfrmsize
                        if x1<1:
                            x1=1
                        y1=int(tpoint[1])-trfrmsize
                        if y1<1:
                            y1=1
                        x2=int(tpoint[0])+trfrmsize
                        if x2>framewidth:
                            x2=int(framewidth-1)
                        y2=int(tpoint[1])+trfrmsize
                        if y2>frameheight:
                            y2=int(frameheight-1)
                        dumb1,frame,box= simple_tracker(tpoint,trfrmsize,frame)
                #Maunual Flight
                if (achannel[9]<350):#'user flight mode' switch F is OFF from Targeting condition
                    mode=mode-1
                    if mode<5:
                        mode=4
                        tpoint,frame,box= simple_tracker(tpoint,trfrmsize,frame)
                        #Autopilot Flight mode
                        if (achannel[8]>1200):#B switch On with F OFF
                            amode=amode+1
                            if amode>10:
                                amode=11
                                #place autopilot controls here (RstkHorz Roll A [0] / RstkVert Pitch E [1] / LstkVert Thrtl [2] / LstkHoz Yaw [3]
                                #positive quadrant error, cmd +pitch or +yaw to remove +error
                                xError1=tpoint[0]-centerXcol#RstkVert Pitch E [1]
                                yError1=tpoint[1]-centerYrow#RstkHorz Roll A [0]
                                if (minHorz<valhorz)&(valhorz<maxHorz):#remove integration windup if max command is reached
                                    xIntegral=xIntegral+xError1#assumes loop time same everytime, hence nothing multiplied to xError; actually we should multiply by actual loop time.
                                if (minVert<valvert)&(valvert<maxVert):#valhorz and valvert init at beginning as rstickhorz and rstickvert
                                    yIntegral=yIntegral+yError1
#                                xDer=xError1-xError2#assume loop time the same, just assess error 'rate' change
#                                yDer=yError1-yError2
#                                xError2=xError1
#                                yError2=yError1
                                if abs(xError1)<5:#remove integration noise near zero error by only using Kp near axis
                                    xIntegral=0
                                if abs(yError1)<5:
                                    yIntegral=0
                                if (xError1*xIntegral)<0:#remove integration unwinding effects if target crosses axis suddenly
                                    xIntegral=0#both the proportional error and integration windup should be of same sign
                                if (yError1*yIntegral)<0:
                                    yIntegral=0
                                valhorz=rstickhorz-((Kp*xError1)+(Ki*xIntegral)) #+(Kd*xDer)
                                if valhorz>maxHorz:
                                    valhorz=maxHorz
                                if valhorz<minHorz:
                                    valhorz=minHorz
                                achannel[0]=int(valhorz)
                                valvert=rstickvert-((Kp*yError1)+(Ki*yIntegral)) #+(Kd*yDer)
                                if valvert>maxVert:
                                    valvert=maxVert
                                if valvert<minVert:
                                    valvert=minVert
                                achannel[1]=int(valvert)
                        else:#User Flight Mode
                            amode=amode-1
                            if amode<5:
                                amode=4
            sendout=parse_serout(achannel,readin)#digichannel)
            ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
        else:
            print("WARNING No Reciever Data Acquired Nor Sent to Flight Controller")
        ser.reset_input_buffer()
        loopimshow=loopimshow+1
        if loopimshow>10:
            print("Show Image")
            if ret_val==1:
                if mode == 11 :#Targeting Mode
                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)#topleft to bottomright
                    cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,255,0),1)
                    cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,255,0),1)
                    cv2.putText(frame, "Targeting", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                if mode ==4 :#Flight mode
                    cv2.rectangle(frame,(int(box[0]), int(box[1])),(int(box[2]),int(box[3])),(0,0,255),2)#topleft to bottomright
                    cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,0,255),1)
                    cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,0,255),1)
                    cv2.circle(frame,(centerXcol,centerYrow),8,(255,0,0),3)
                    if amode>10:#User flight mode
                        cv2.putText(frame, "Auto Pilot", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    if amode<5:#Autopilot flight mode
                        cv2.putText(frame, "Controlled Flight", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                loopimshow=0
                cv2.imshow("Show", frame)# Display result
#                print("Width  ", trfrmsize)
            if cv2.waitKey(20) == 27:# esc to quit; this .waitKey consumes at least 22ms time
                stop_loop(frame,cam)
                break
        print(time.time()-start,' LOOP TIME')
        start=time.time()

if __name__ == '__main__':
    main()
#            a2channel=achannel #test quality of bytes going out on sbus
#            sendout=parse_serout(a2channel,readin)#digichannel)
#            parse_serin(sendout)#generates a new achannel
#            if (abs(achannel[0]-a2channel[0]))>5:
#                    print("PROBLEM achannel 0")
#            if (abs(achannel[1]-a2channel[1]))>5:
#                    print("PROBLEM achannel 1")
#            if (abs(achannel[2]-a2channel[2]))>5:
#                    print("PROBLEM achannel 2")
#            if (abs(achannel[3]-a2channel[3]))>5:
#                    print("PROBLEM achannel 3")
##            time.sleep(.005)#help with noise on serial output 
