import cv2
import time
import subprocess#for setting cam properties at shell level

def stop_loop(frame,cam):#break jumps us out of inner most loop and used in an if statement of the loop
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
#    cv2.waitKey(100)
    cam.release()
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()

framewidth = int(640)#int(1920)#int(640)
frameheight = int(480)#int(1080)#int(480)

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
cv2.resizeWindow('Show',framewidth,frameheight)#640,480)
#    cv2.namedWindow("Showbbox", cv2.WINDOW_NORMAL)#
#    cv2.namedWindow("ShowDevFrame", cv2.WINDOW_NORMAL)#
##    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
ret_val, frame = cam.read() #must init & warm-up cam, this frame usually takes .250sec
time.sleep(.25)

while 1:
    ret_val=0
    ret_val, frame = cam.read() #read frame from camera
    cv2.imshow("Show", frame)# Display result
    
    if cv2.waitKey(20) == 27:# esc to quit; this .waitKey consumes at least 22ms time
        stop_loop(frame,cam)
        break

