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

#python interpreter renames xyz.py program global __name__ to __main__
#if some other .py script uses thisscript.py as import thisscript.py then it's __name__ = actual name, not__main__

def show_webcam():#mirror=False):
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second
    cam.set(cv2.CAP_PROP_CONVERT_RGB,1)
    cam.set(cv2.CAP_PROP_FPS,60)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)#1280)#1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)#720)#1080)
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen
    start=time.time()
    ret_val, frame = cam.read() #warm-up frame usually takes .250sec
    count=0
    bbox = (220, 100, 90, 200)#init bounding box
    tracker_type='mse'
    if tracker_type == 'bst':
        tracker = cv2.TrackerBoosting_create() #0.470s
    if tracker_type == 'mil':
        tracker = cv2.TrackerMIL_create() #0.730s
    if tracker_type == 'kcf':
        tracker = cv2.TrackerKCF_create() #0.200s
    if tracker_type == 'tld':
        tracker = cv2.TrackerTLD_create() #0.720s
    if tracker_type == 'mfl':
        tracker = cv2.TrackerMedianFlow_create() #0.090s
    if tracker_type == 'mse':
        tracker = cv2.TrackerMOSSE_create() #0.040s
    if tracker_type == "crt":
        tracker = cv2.TrackerCSRT_create() #0.575s
    ok = tracker.init(frame, bbox)
    while True:
        start=time.time()
        ret_val, frame = cam.read()
        start=how_long(start,'cam.read took ')
        # 
        start=time.time()
        ok, bbox = tracker.update(frame)
        start=how_long(start,'tracker took ')
#            # --------- Steer Right Routine ----------
#            if cx >= 370:
#                RSteer = cx - 370
#                SteerRight = remap(RSteer, 0, 45, 1, 270)
#                print ("Turn Right: ", SteerRight)
#
#            # --------- On Track Routine ----------
#            if cx < 370 and cx > 270:
#                print "On Track"
#
#            # --------- Steer Left Routine ----------
#            if cx <= 270:
#                LSteer = 270 - cx
#                SteerLeft = remap(LSteer, 0, 45, 1, 270)
#                print ("Turn Left: ", SteerLeft)           
         # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            print("Tracking failure detected")
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        # Display result
        cv2.imshow("Show", frame)
        if cv2.waitKey(1) == 27:# esc to quit
            stop_loop(frame,cam)
            break  
        count += 1
        if count>1000:
            stop_loop(frame,cam)
            break

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

def main():
    show_webcam()#mirror=True)

if __name__ == '__main__':
    main()

