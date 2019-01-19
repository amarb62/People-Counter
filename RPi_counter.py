##People counter
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import Person
import time
from time import sleep
import os

cnt_up   = 0
cnt_down = 0
count_up = 0
count_down = 0
state =0
filename="Counter.csv"
w = 256
h = 192

def writecsv(cin,cout):
    global filename
    #open csv file in append mode
    filep = open(filename,'a')
    # create string data to write per image
    datastr = "IN-" + str(cin) + "\nOUT-" + str(cout)+ "\n\n"
    #write to csv
    filep.write(datastr)

############################################
#initialize a PiCam object
camera = PiCamera()
#setting up resolution of the PiCam to 240,180 pixels
camera.resolution = (w,h)
#set the framerate
camera.framerate=5
#create a RGB Array of PiCam storage type
rawCapture = PiRGBArray(camera,(w,h))
time.sleep(0.1)
############################################


frameArea = h*w
areaTH = frameArea/200
print 'Area Threshold', areaTH

#Lines coordinate for counting
line_up = int(1*(h/5))
line_down= int(4*(h/5))

up_limit = int(.5*(h/5))
down_limit = int(4.5*(h/5))

print "Red line y:",str(line_down)
print "Blue line y:", str(line_up)
line_down_color = (255,0,0)
line_up_color = (0,0,255)
pt1 =  [0, line_down];
pt2 =  [w, line_down];
pts_L1 = np.array([pt1,pt2], np.int32)
pts_L1 = pts_L1.reshape((-1,1,2))
pt3 =[0, line_up];
pt4 =[w, line_up];
pts_L2 = np.array([pt3,pt4], np.int32)
pts_L2 = pts_L2.reshape((-1,1,2))

pt5 =[0, up_limit];
pt6 =[w, up_limit];
pts_L3 = np.array([pt5,pt6], np.int32)
pts_L3 = pts_L3.reshape((-1,1,2))
pt7 =[0, down_limit];
pt8 =[w, down_limit];
pts_L4 = np.array([pt7,pt8], np.int32)
pts_L4 = pts_L4.reshape((-1,1,2))

#Background Substractor
fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = True)

#Structuring elements for morphographic filters
kernelOp = np.ones((3,3),np.uint8)
kernelOp2 = np.ones((5,5),np.uint8)
kernelCl = np.ones((11,11),np.uint8)

#Variables
font = cv2.FONT_HERSHEY_SIMPLEX
persons = []
rect_co = []
max_p_age = 1
pid = 1
val = []

#to capture video coninuously create a video object
for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
    ## Read the image
    frame = image.array
    
    #Apply background subtraction
    fgmask = fgmask2= fgbg.apply(frame)

    #Binarization to eliminate shadows
    try:
        ret,imBin= cv2.threshold(fgmask,200,255,cv2.THRESH_BINARY)
        ret,imBin2 = cv2.threshold(fgmask2,200,255,cv2.THRESH_BINARY)
        #Opening (erode->dilate) to remove noise.
        mask = cv2.morphologyEx(imBin, cv2.MORPH_OPEN, kernelOp)
        mask2 = cv2.morphologyEx(imBin2, cv2.MORPH_OPEN, kernelOp)
        #Closing (dilate -> erode) to join white regions.
        mask =  cv2.morphologyEx(mask , cv2.MORPH_CLOSE, kernelCl)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernelCl)
    except:
        print('EOF')
        print 'UP:',cnt_up+count_up
        print 'DOWN:',cnt_down+count_down
        break
    #################
    #   CONTOURS   #
    #################
    
    # RETR_EXTERNAL returns only extreme outer flags. All child contours are left behind.
    _, contours0, hierarchy = cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours0:
        rect = cv2.boundingRect(cnt)
        area = cv2.contourArea(cnt)
        if area > areaTH:
            #################
            #   TRACKING    #
            #################
            
            #Missing conditions for multipersons, outputs and screen entries
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            x,y,w,h = cv2.boundingRect(cnt)

            new = True
            if cy in range(up_limit,down_limit):
                for i in persons:
                    if abs(cx-i.getX()) <= w and abs(cy-i.getY()) <= h:
                        # the object is close to one that has already been detected before
                        new = False
                        i.updateCoords(cx,cy)   #update coordinates in the object and resets age
                        if i.going_UP(line_down,line_up) == True:
                            if w > 100:
                                count_up = w/60
                                print 
                            else:    
                                cnt_up += 1;
                                writecsv(cnt_up+count_up,cnt_down+count_down)
                            print "ID:",i.getId(),'crossed going up at',time.strftime("%c")
                        elif i.going_DOWN(line_down,line_up) == True:
                            if w > 100:
                                count_down = w/60
                            else:
                                cnt_down += 1;
                                writecsv(cnt_up+count_up,cnt_down+count_down)
                            print "ID:",i.getId(),'crossed going down at',time.strftime("%c")
                        break
                    if i.getState() == '1':
                        if i.getDir() == 'down' and i.getY() > down_limit:
                            i.setDone()
                        elif i.getDir() == 'up' and i.getY() < up_limit:
                            i.setDone()
                    if i.timedOut():
                        #get out of the people list
                        index = persons.index(i)
                        persons.pop(index)
                        del i     #free the memory of i
                if new == True:
                    p = Person.MyPerson(pid,cx,cy, max_p_age)
                    persons.append(p)
                    pid += 1
                    
            #################
            #   DRAWINGS     #
            #################
            cv2.circle(frame,(cx,cy), 5, (0,0,255), -1)
            img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)            
        
    ####################
    # DISPLAY ON FRAME #
    ####################
    str_up = 'IN: '+ str(cnt_up+count_up)
    str_down = 'OUT: '+ str(cnt_down+count_down)
    frame = cv2.polylines(frame,[pts_L1],False,line_down_color,thickness=2)
    frame = cv2.polylines(frame,[pts_L2],False,line_up_color,thickness=2)
    frame = cv2.polylines(frame,[pts_L3],False,(255,255,255),thickness=1)
    frame = cv2.polylines(frame,[pts_L4],False,(255,255,255),thickness=1)
    cv2.putText(frame, str_up ,(10,30),font,0.5,(0,0,255),1,cv2.LINE_AA)
    cv2.putText(frame, str_down ,(10,70),font,0.5,(255,0,0),1,cv2.LINE_AA)
    cv2.imshow('Live Tracking',frame)  
    rawCapture.truncate(0)
    #Press ESC to exit
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    
#################
#   CLOSING    #
#################
cv2.waitKey()
cv2.destroyAllWindows()
