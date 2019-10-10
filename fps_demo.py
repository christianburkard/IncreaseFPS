# USAGE
# python fps_demo.py
# python fps_demo.py --display 1

# import the necessary packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import cv2
from collections import deque
import numpy as np
import time


numframesut = 100
umframesth = 1000

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
    help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=1,
    help="Whether or not frames should be displayed")
args = vars(ap.parse_args())


# allow the camera to warmup
time.sleep(0.5)
buffer = 120
# define the lower and upper boundaries of the "black"
# ball in the HSV color space, then initialize the
# list of tracked points
#blackLower = (29, 86, 6)
#blackUpper = (64, 255, 25)
blackUpper = (20, 20, 20)
blackLower = (0, 0, 0)
pts = deque(maxlen=buffer)
# allow the camera or video file to warm up
time.sleep(2.0)
tracker = None
writer = None
confdef = 0.4
fps = FPS().start()

# grab a pointer to the video stream and initialize the FPS counter
#print("[INFO] sampling frames from webcam...")
#stream = cv2.VideoCapture(0)
fps = FPS().start()

## loop over some framesr
#while fps._numFrames < numframesut:
#    # grab the frame from the stream and resize it to have a maximum
#    # width of 400 pixels
#    (grabbed, frame) = stream.read()
#    frame = imutils.resize(frame, width=400)
#
#    # check to see if the frame should be displayed to our screen
#
#    cv2.imshow("Frame", frame)
#    cv2.waitKey(1) & 0xFF
#
#    # update the FPS counter
#    fps.update()
#
## stop the timer and display FPS information
#fps.stop()
#print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
#print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
#
## do a bit of cleanup
#stream.release()
#cv2.destroyAllWindows()
#
## created a *threaded *video stream, allow the camera senor to warmup,
## and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=0).start()
fps = FPS().start()

coordArrayX = np.array([])
coordArrayY = np.array([])

# loop over some frames...this time using the threaded stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=400)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, blackLower, blackUpper)
    mask2 = cv2.erode(mask1, None, iterations=1)
    mask3 = cv2.dilate(mask2, None, iterations=1)

    # find contours in the mask and initialize the current
    # (x, y) center of the object
    cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None



# only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        PixCoordX = (center[0])
        PixCoordY = (center[1])
        print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                pixcoordinate
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)





        pts.appendleft(center)



# loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
#        thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
#        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

#    with open("H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv", "a") as log:
#        log.write("{0},{1}\n".format(PixCoordX, PixCoordY,))

    coordArrayX = np.append(coordArrayX,abs(PixCoordX))
    coordArrayY = np.append(coordArrayY,abs(PixCoordY))


    # show the frame
#    cv2.imshow("Frame", frame)
#    cv2.imshow("blurred", blurred)
#    cv2.imshow("hsv", hsv)
    # check to see if the frame should be displayed to our screen

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("r"):
        break
    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()