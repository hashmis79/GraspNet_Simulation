#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

threshold1 = 0
threshold2 = 0
H1 = 0
H2 = 0
S1 = 0
S2 = 0
V1 = 0
V2 = 0

def show_image(): 
    image = cv2.imread('/home/cair/test_ws/src/manipulator1/src/test_pic15.png') 

    # image = cv2.resize(image,(300,300)) 
    global H1,H2,V1,V2,S1,S2
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    min_green = np.array([H1,S1,V1]) 
    max_green = np.array([H2,S2,V2]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(image,image, mask= mask_g) 

    cv2.imshow('Green',res_g) 

    cv2.imshow('Original',image) 
    # cv2.waitKey(0) 
    # cv2.destroyAllWindows()

def detection(threshold1,threshold2):
    im = cv2.imread("/home/cair/test_ws/src/manipulator1/src/test_pic15.png")
    # print(threshold1,threshold2)
    # Set up the detector with default parameters.
    # detector = cv2.SimpleBlobDetector_create()
    # # Detect blobs.
    # keypoints = detector.detect(im)
    # # Draw detected blobs as red circles.
    # # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    # red = (250,0,0)
    # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]))
    # for i in keypoints :
    #   im[i] = (255,0,0)
    # # Show keypoints
    # print(im.width)
    img_blur = cv2.GaussianBlur(im, (3,3), 0)
    # show_image()
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV) 
    min_green = np.array([0,0,0]) 
    max_green = np.array([255,102,255]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(im,im, mask= mask_g) 
    res_g = cv2.cvtColor(res_g, cv2.COLOR_HSV2BGR)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(im, threshold1, threshold2, cv2.THRESH_BINARY)
    drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    edges = cv2.Canny(image=im, threshold1=threshold1, threshold2=threshold2)
    contours, hierarchy= cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    # cv2.drawContours(image=im, contours=contours, contourIdx=0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    rectangle = []
    minArea = 100000
    maxArea = 200
    print(len(contours))
    for count in contours:
        area = cv2.contourArea(count)
        print(area)
        color = (0,0,0)
        epsilon = 0.01 * cv2.arcLength(count, True)
        approximations = cv2.approxPolyDP(count, epsilon, True)
        if maxArea< area < minArea : 
            # cv2.drawContours(edges, [approximations], 0, (0), 3)
            (x,y),(width,height),angle_of_rotation = cv2.minAreaRect(count)
            # print(x,y,width,height, angle_of_rotation)
            rect =cv2.minAreaRect(count)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print(box)
            boundRect = cv2.boundingRect(box)
            cv2.rectangle(im, (int(boundRect[0]), int(boundRect[1])),(int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 2)
    cv2.imshow("Keypoints",im)
    # try:
    #     x,y,width,height,angle_of_rotation = cv2.minAreaRect(edges)
    # except(Exception):
    #     print(Exception)

    # print(x,y,width,height,angle_of_rotation)
    # print(keypoints)
    # cv2.imshow("Keypoints", im)
    # cv2.waitKey(0)

def on_change1(val):
    global threshold1,threshold2
    threshold1 = val
    detection(threshold1,threshold2)
    

def on_change2(val):
    global threshold1,threshold2
    threshold2 = val
    detection(threshold1,threshold2)

# def on_change3(val):
#     global H1
#     H1 = val
#     show_image()
# def on_change4(val):
#     global H2
#     H2 = val
#     show_image()
# def on_change5(val):
#     global V1
#     V1 = val
#     show_image()
# def on_change6(val):
#     global V2
#     V2 = val
#     show_image()
# def on_change7(val):
#     global S1
#     S1 = val
#     show_image()
# def on_change8(val):
#     global S2
#     S2 = val
#     show_image()



im = cv2.imread("/home/cair/test_ws/src/manipulator1/src/test_pic6.png")
windowName = 'image'
 
cv2.imshow(windowName, im)
cv2.createTrackbar('threshold1', windowName, 0, 255, on_change1)
cv2.createTrackbar('threshold2', windowName, 0, 255, on_change2)
# cv2.createTrackbar('H1', windowName, 0, 255, on_change3)
# cv2.createTrackbar('H2', windowName, 0, 255, on_change4)
# cv2.createTrackbar('V1', windowName, 0, 255, on_change5)
# cv2.createTrackbar('V2', windowName, 0, 255, on_change6)
# cv2.createTrackbar('S1', windowName, 0, 255, on_change7)
# cv2.createTrackbar('S2', windowName, 0, 255, on_change8)
cv2.waitKey(0)
cv2.destroyAllWindows()