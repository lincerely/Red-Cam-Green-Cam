#! usr/bin/env python
#
#   SM3610 Hardware Hacking
#   Red cam Green cam
#   7/12/2017
#   Tested on raspberry pi 3b with python 2 + opencv 2.4.x
#
#   Python package dependencies: imutils, cv2, gpiozero
#
#

from __future__ import print_function, division
from imutils.video import VideoStream
from gpiozero import AngularServo, LED, Buzzer, Button
import argparse
import imutils
from time import sleep, strftime
import cv2
import os
from random import randint
import numpy as np

#
#   bcolors - for printing color log
#


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

#
#   kill() - Called when user press the button
#


def kill():

    print(bcolors.HEADER + "[INFO] Killed" + bcolors.ENDC)

    global buttonPressed
    global v
    global sv
    global alarm

    if buttonPressed is True:
        return

    alarm.off()

    v = -50
    sv.angle = v
    sleep(0.5)
    buttonPressed = True

    sh.detach()
    sv.detach()

#
#   clamp() - helper function to keep servo angle in range
#


def clamp(x, maxX, minX):
    return max(min(x, maxX), minX)

#
#   /// Initialization ///
#


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("--display", dest="display", action="store_true", help="display Pi's view while running")
ap.add_argument("-a", "--min-area", dest="minA", type=int, default=2500, help="minimum area size")
ap.add_argument("-A", "--max-area", dest="maxA", type=int, default=275 * 300, help="maximum area size")
ap.add_argument("--webcam", dest="picam", action="store_false", help="use webcam (default Raspberry Pi cam)")
ap.add_argument("--record", dest="record", action="store_true", help="record processed frames into video")
ap.add_argument("-o","--out", dest="vname", type=str, default=strftime("%Y%m%d-%H%M%S")+".avi", help="output video path")

ap.set_defaults(display=False, picam=True, record=False)
args = vars(ap.parse_args())

display = args["display"]

print("[INFO] -- Settings -- ")
print("[INFO] Display =", display)
print("[INFO] minimum area size =", args["minA"])
print("[INFO] maximum area size =", args["maxA"])
print("[INFO] Record =", args["record"])
if args['record'] is True:
    print("[INFO] Output path =", args["vname"])


print("[INFO] Waiting for pi cam to warmup...")
vs = VideoStream(usePiCamera=args["picam"]).start()
sleep(2.0)


print("[INFO] Waking up servos...")

#
#   GPIO settings
#

sv = AngularServo(3, min_angle=-50, max_angle=50)
sh = AngularServo(2, min_angle=50, max_angle=-50)

alarm = LED(16)
alarm.off()


button = Button(18)
button.when_pressed = kill

frame = vs.read()

sleep(2.0)

#
#   Initial rhe FourCC, video writer, dimension of the frame
#
isRecord = args["record"]
if isRecord:
    fourcc = cv2.cv.FOURCC(*"MJPG")
    writer = None
    (h, w) = (None, None)
    zeros = None
    vname = args["vname"]
    fps = 2

#
#   Constant for setting
#

boundaryX = 150
boundaryY = 100
FW = 500    # Frame Width
FH = 375    # Frame Height
angleVarX = 0.1
angleVarY = 0.3

#
#   Global Vars
#
h = 0
v = 0

firstFrame = None   # firstFame for comparsion
lastFrame = None    # for making gif


buttonPressed = None
isMotionInBounded = False

#
#   /// Main Loop ///
#

print("[INFO] start detection...")
while True:

    # Player wins
    # Died sequence is playing, no need to get frame or compare
    if buttonPressed:
        alarm.off()
        #print(bcolors.HEADER+"[INFO] Double Killed"+bcolors.ENDC)
        v = clamp(v + 1, 50, -50)
        sv.angle = v
        #print("sv.angle = %f, v = %f" % (sv.angle, v))
        sleep(0.01)

        if sv.angle >= 50:  # it is totally died

            sh.detach()
            sv.detach()

            # -- Restart the game after 10 sec
            sleep(10)
            buttonPressed = False

        continue

    isMotion = False
    isMotionInBound = False

    # grab the next frame from the video stream
    org_frame = vs.read()

    # resize the frame, convert it to grayscale, and blur it
    org_frame = imutils.resize(org_frame, width=500)
    frame = org_frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # No frame to compare, continue to the next loop
    if firstFrame is None:
        sleep(0.25)
        firstFrame = gray
        continue

    # compute the absolute difference between the current frame and
    # first frame
    frameDelta = cv2.absdiff(firstFrame, gray)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

    # dilate the thresholded image to fill in holes, then find contours
    # on thresholded image
    thresh = cv2.dilate(thresh, None, iterations=2)
    (cnts, _) = cv2.findContours(thresh.copy(),
                                 cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # the center of the contour rect
    centerx = None
    centery = None

    # use for finding max area
    maxArea = 0
    maxC = None

    # loop over the contours, select the largest area
    for c in cnts:
        area = cv2.contourArea(c)
        # if the contour is too small, ignore it
        if area < args["minA"] or area > args["maxA"]:
            continue
        elif maxC is None:
            maxC = c
            maxArea = area
            continue
        elif area > maxArea:
            maxC = c
            maxArea = area
            continue

    if not maxC is None:
        # compute the bounding box for the contour, draw it on the frame,
        # and update the text
        (x, y, w, h2) = cv2.boundingRect(maxC)

        centerx = x + w / 2
        centery = y + h2 / 2

        #print("[INFO] Center: %f, %f" % (centerx,centery) )

        isMotion = True

        if centerx > boundaryX and centerx < 500 - boundaryX and centery > boundaryY and centery < 375 - boundaryY:
            isMotionInBound = True
            # Draw the detected motion rect
            cv2.rectangle(frame, (x, y), (x + w, y + h2), (0, 0, 255), 1)
        else:
            # Draw the detected motion rect
            cv2.rectangle(frame, (x, y), (x + w, y + h2), (0, 255, 0), 1)

    #   Draw the boundary
    cv2.rectangle(frame, (boundaryX, boundaryY),
                  (FW - boundaryX, FH - boundaryY), (255, 0, 0), 1)


    #  Blur Detection
    
    centerImg = org_frame[boundaryY:FH - boundaryY, boundaryX:FW-boundaryX]
    centerBlur = cv2.Laplacian(centerImg, cv2.CV_64F).var()
    frameBlur = cv2.Laplacian(org_frame, cv2.CV_64F).var()
    
    
    	
    

    #
    #   Decision Making - response to input
    #

    if frameBlur > centerBlur:	# CASE 4: center is blurry, instant death
    	print(bcolors.FAIL + "[INFO] center blur and GAME OVER" + bcolors.ENDC)
    	
        alarm.on()
        sleep(3)
        alarm.off()
        
        # reset game
        firstFrame = None
        isMotionInBounded = False

    elif isMotionInBound:  # CASE 3

        if not isMotionInBounded:   # CASE 3A: last frame is not in bound
            print(bcolors.WARNING +
                  "[INFO] Motion in bound...maybe" + bcolors.ENDC)
            # Allow the player to stay and prevent error
            alarm.off()
            firstFrame = None
            isMotionInBounded = True
        else:   # CASE 3B: confirm it is a kill
            print(bcolors.FAIL +
                  "[INFO] Motion in bound and GAME OVER" + bcolors.ENDC)

            alarm.on()
            sleep(3)
            alarm.off()

            # reset game
            firstFrame = None
            isMotionInBounded = False

    elif isMotion:  # CASE 2: Motion detected but not inside the rect

        isMotionInBounded = False

        alarm.off()

        # move towards the motion center

        dh = (centerx - (FW / 2)) * angleVarX  # (50/(FW/2))
        dv = (centery - (FH / 2)) * angleVarY  # (50/(FH/2))

        h = clamp(h + dh, 50, -50)
        v = clamp(v + dv, 40, -10)

        sh.angle = h
        sv.angle = v

        print("%s[INFO] Motion detected, move to (%f, %f), d = (%f, %f)%s" % (
            bcolors.OKBLUE, h, v, dh, dv, bcolors.ENDC))

        firstFrame = None

    else:  # CASE 1: no motion --> random move
        if h > 0:
            randomX = (randint(0, 2500) - 1875) * angleVarX
            randomY = (randint(0, 2500) - 1875) * angleVarY
        else:
            randomX = (randint(0, 2500) - 625) * angleVarX
            randomY = (randint(0, 2500) - 625) * angleVarY

        h = clamp(h + randomX, 50, -50)
        v = clamp(v + randomY, 20, -20)

        sh.angle = h
        sv.angle = v

        print("%s[INFO] No motion detected, randomly move to (%f, %f)%s" %
              (bcolors.OKGREEN, h, v, bcolors.ENDC))
        firstFrame = None
        isMotionInBounded = False
        alarm.off()

    # Sleep to allow the servo to move
    sleep(0.15)
    sh.detach()
    sv.detach()

    if display:
        cv2.imshow("Shybot's view", frame)
        #cv2.imshow("Change", frameDelta)
        #cv2.imshow("Thresh", thresh)

        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key was pressed, break from the loop
        if key == ord("q"):
            break
        elif key == ord("r"):
            firstFrame = None

    if isRecord:
        # check if the writer is None
        if writer is None:
            # store the image dimensions, initialzie the video writer,
            # and construct the zeros array
            (h, w) = frame.shape[:2]
            writer = cv2.VideoWriter(vname, fourcc, fps,
                (w, h), True)
            zeros = np.zeros((h, w), dtype="uint8")
     
        # write the output frame to file
        writer.write(frame)

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()

if isRecord:
    print("[INFO] Saving output... ")
    writer.release()
