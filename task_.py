#import wireframe_material
import os, sys
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
    params.filterByColor = 0
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
    while(cap2.isOpened()):
        ret, img = cap2.read()

        cv2.imshow('RGB',img)
    
        im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)              

        detector = cv2.SimpleBlobDetector(params)
         
        # Detect blobs.
        keypoints = detector.detect(im)
         
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        
        if keypoints != []:
            keypoint_pos = keypoints[0].pt

            print 'len keypoints - ', len(keypoints)

            if keypoint_pos[0]<481 and keypoint_pos[1]<681:
                #print "I am here"
                print 'keypoints= ',str(keypoint_pos)
                rgb = img[keypoint_pos[0]][keypoint_pos[1]]
                #rbg[0] = rgb[0].replace(' ',',')
                rgb2 = (int(rgb[0]),int(rgb[1]),int(rgb[2]))

                print 'rgb = ', rgb2
                
                im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), rgb2, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #else:
                #im_with_keypoints = cv2#.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                #if 120 < rgb2[0] > 130 and
                # rgb =  (124, 124, 117) orange
                cv2.imshow("Keypoints", im_with_keypoints)

        #time.sleep(.1)
        key = cv2.waitKey()
        key=0    
        if key == 27:
            break
                                 
    cap2.release()
    cv2.destroyAllWindows() 
          

webcam(0,'RGB')
