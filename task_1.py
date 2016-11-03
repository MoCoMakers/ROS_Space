#import wireframe_material
import wx, os, sys
import numpy as np
import cv2,time


cad=[283,202,51]
#blank_image = np.zeros((720,1280,3), np.uint8)
tuples=[]     
xt = 0
yt = 0
xs = 1.0
ys = 1.0 
refPt = []
cropping = False
#cropped = False
xxx = np.uint8(255)
yyy = np.uint8(255)

sock = 'socket'
def send(sock,num):
    print sock, num

def webcam(device, win_name):
    
    global blank_image, xs, ys, xt, yt
    buf ="                                                             "
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 30;
    params.maxThreshold = 300;
    
    # Change color
    params.filterByColor = 1
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = 1
    params.minArea = 200
     
    # Filter by Circularity
    params.filterByCircularity = 0
    params.minCircularity = 0.01
     
    # Filter by Convexity
    params.filterByConvexity = 1
    params.minConvexity = 0.7
     
    # Filter by Inertia
    params.filterByInertia = 1
    params.minInertiaRatio = 0.5
     
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)

    cap2 = cv2.VideoCapture(device)
    win2 = cv2.cv.NamedWindow("Laser", cv2.cv.CV_WINDOW_AUTOSIZE)

    ret, frame2 = cap2.read()
    ret = cap2.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)
    cnt = 0
    while(cap2.isOpened()):
        ret, img = cap2.read()

        cv2.imshow('RGB',img)
    
        im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)              

        detector = cv2.SimpleBlobDetector(params)
         
        # Detect blobs.
        keypoints = detector.detect(im)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if keypoints != [] and cnt>2:
            #print 'keypoints= ',keypoints
            print 'pointx = ', int(keypoints[0].pt[0]),int(keypoints[0].pt[1])
            print 'color = ',img[keypoints[0].pt[0]][keypoints[0].pt[1]]
        cv2.imshow("Keypoints", im_with_keypoints)
        cnt +=1
        
        key = cv2.waitKey()
        if key == 27:
            break
                                 
    cap2.release()
    cv2.destroyAllWindows() 
          

webcam(0,'RGB')
