# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.exposure_mode = 'off' 
keypointsLast = []
#setup simple blob dectection 
params = cv2.SimpleBlobDetector_Params()

# Set Thresholds
params.minThreshold = 40
params.maxThreshold = 255

#Filter by Color 
params.filterByColor = False
params.blobColor = 255

#Filter by Areaq
params.filterByArea = True
params.minArea = 10
params.maxArea = 50


#Filter by Circularity
params.filterByCircularity = False
params.minConvexity = 0.1

#Fliter by Inertia?
params.filterByInertia = True

ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else : 
    detector = cv2.SimpleBlobDetector_create(params)


# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    
    imgBGR = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
    
    imgIRBW = cv2.inRange(imgHSV, np.array([0,100,230]), np.array([360,255,255]))

    contours, hierarchy = cv2.findContours(imgIRBW, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0,255,0), 3)
    
    keypoints = detector.detect(imgIRBW)
    if keypoints == []:
        cv2.drawKeypoints(image, keypointsLast, image , color=(255,0,0), flags=0)
    else:
        keypointsLast = keypoints
        cv2.drawKeypoints(image, keypoints, image , color=(255,0,0), flags=0)
    
    
    print(keypoints)
    
    # show the frame
    cv2.imshow("Frame", image)
    cv2.imshow("IR frame", imgIRBW)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
        