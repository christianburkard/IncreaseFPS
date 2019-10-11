# USAGE
# python picamera_fps_demo.py
# python picamera_fps_demo.py --display 1

# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
    help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
    help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

numframes = 1000

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))
stream = camera.capture_continuous(rawCapture, format="bgr",
    use_video_port=True)

# allow the camera to warmup and start the FPS counter
print("[INFO] sampling frames from `picamera` module...")
time.sleep(2.0)
fps = FPS().start()

# loop over some frames
for (i, f) in enumerate(stream):
    # grab the frame from the stream and resize it to have a maximum
    # width of 400 pixels
    frame = f.array
    frame = imutils.resize(frame, width=400)

    # check to see if the fraif args["display"] > 0:
    cv2.imshow("Frame", frame)
    cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame and update
    # the FPS counter
    rawCapture.truncate(0)
    fps.update()

    # check to see if the desired number of frames have been reached
    if i == numframes/10:
        break

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)

buffer = 120
# define the lower and upper boundaries of the "black"
# ball in the HSV color space, then initialize the
# list of tracked points
#blackLower = (29, 86, 6)
#blackUpper = (64, 255, 25)
blackUpper = (60, 60, 60)
blackLower = (0, 0, 0)
pts = deque(maxlen=buffer)
# allow the camera or video file to warm up
time.sleep(1.0)
tracker = None
writer = None
confdef = 0.4
fps = FPS().start()

# loop over some frames...this time using the threaded stream
while fps._numFrames < numframes:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels


    frame = vs.read()
    try:
        frame.shape
        print("checked for shape".format(img.shape))
    except AttributeError:
        print("shape not found")
        #code to move to next frame


    frame = imutils.resize(frame, width = 400)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, blackLower, blackUpper)
    mask2 = cv2.erode(mask1, None, iterations=1)
    ##                mask3 = cv2.dilate(mask2, None, iterations=1)


    #Apply median filter
    #            mask3 = cv2.dilate(mask2, None, iterations=1)
    mask4 = cv2.medianBlur(mask2,5)


    # find contours in the mask and initialize the current
    # (x, y) center of the object
    cnts = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
        if int(M["m00"]) == 0:
            center = (0, 0)
        else:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        PixCoordX = (center[0])
        PixCoordY = (center[1])
        print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
    #                pixcoordinate

        # write data to arduino
    #                    writeDataArduino()
        # only proceed if the radius meets a minimum size
        if radius > 0.5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    # update the points queue
    try:
        print("Contour radius: {:.2f}".format(radius))
        PixRadius = radius
    #                    serRasp = serial.Serial('/dev/ttyUSB0',57600)
    #                    serRasp.write(PixCoordX)
    #                    serRasp.write(PixCoordY)
        pts.appendleft(center)
    except: print("No radius detected ...")
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

    coordArrayX = np.append(coordArrayX,abs(PixCoordX))
    coordArrayY = np.append(coordArrayY,abs(PixCoordY))
    radiusArray = np.append(radiusArray,abs(PixRadius))


    # show the frame
    #                cv2.imshow("mask1", mask1)
    #                cv2.imshow("erode", mask2)
    cv2.imshow("contour", frame)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("r"):
        print("Exiting... ")
        break
    fps.update()

    fps.stop()








    # check to see if the frame should be displayed to our screen

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()