#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy,tf2_ros, geometry_msgs.msg # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import *
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

threshVal = 0
x = 0
y = 0
w = 0
h = 0
box_height = 0
Average_depth = 0
camera = [0.035,0,0.18]
camera_mono = [0.0635,0,-0.001]
X_Coordinate = 0
Y_Coordinate = 0
Z_Coordinate = 0
Orientation = 0
camera_wrt_origin = np.eye(4,4)
function_control = 0.0

def detect_coordinate(x, y, w, h, Average_depth,hfov_rad):
  # global Average_depth, x, y, w, h
  # img_width: pixel image width
  img_width = 640
  img_height = 480
  # h_fov_rad: Horizontal Field of View of Camera in Radians
  # focal_length: constant focal length of camera
  focal_length = (img_width/2)/math.tan(hfov_rad/2)

  # for (x, y, w, h) in logo:
  #   cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
    # centre_x_pixel= (2*x+w)/2.0
    # centre_y_pixel= (2*y+h)/2.0
  centre_x_pixel = x
  centre_y_pixel = y
  err_x_m = ((img_width/2)-centre_x_pixel)*(Average_depth)/focal_length
  err_y_m = ((img_height/2)-centre_y_pixel)*(Average_depth)/focal_length
  err_h = (h)*(Average_depth)/focal_length
  err_w = (w)*(Average_depth)/focal_length
  return(err_x_m,err_y_m, err_h, err_w)

# ------------------------------Callback for Stereo Camera---------------------------------------

def callback_state(data):
  global function_control
  function_control = data.data
  if data.data == 1.0:
    rospy.Subscriber('/camera2/color/image_raw', Image, callback)
  elif data.data == 2.0:
    rospy.Subscriber('/camera1/image_raw', Image, callback_mono)
  else:
    exit

def callback(data):
  global function_control
  if function_control == 1.0:
    try:
      box_coordinate = rospy.Publisher("/box/Coordinate", Float64MultiArray, queue_size =1)
    except(Exception):
      print(Exception)
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    global threshVal,x,y,w,h, camera, box_height
    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")
    
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    new_frame = current_frame
    # print(current_frame)
    # img_blur = cv2.GaussianBlur(current_frame, (3,3), 0)
    # im = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
    # ret,thresh = cv2.threshold(im, 85, 255, cv2.THRESH_BINARY)
    # drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV) 
    min_green = np.array([0,0,0]) 
    max_green = np.array([255,255,102]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(current_frame,current_frame, mask= mask_g)
    edges = cv2.Canny(image=res_g, threshold1=85, threshold2=255)
    contours, hierarchy= cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    # cv2.drawContours(image=im, contours=contours, contourIdx=0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    rectangle = []
    minArea = 1
    # print(len(contours))
    for count in contours:
        area = cv2.contourArea(count)
        color = (0,0,0)
        epsilon = 0.01 * cv2.arcLength(count, True)
        approximations = cv2.approxPolyDP(count, epsilon, True)
        if area > minArea : 
            # cv2.drawContours(im, [approximations], 0, (0), 3)
            (x,y),(w,h),angle_of_rotation = cv2.minAreaRect(count)
            # print(x,y,w,h, angle_of_rotation)
            # print(area)
            rect =cv2.minAreaRect(count)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print(box)
            boundRect = cv2.boundingRect(box)
            cv2.rectangle(new_frame, (int(boundRect[0]), int(boundRect[1])),(int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 1)
    
    # cv2.imshow("key",new_frame)
    # cv2.waitKey(0)
    rospy.Subscriber('/camera2/depth/image_raw', Image, callback_depth)
    global Average_depth
    depth = Average_depth
    Y_Coordinate, Z_Coordinate, box_height, box_width = detect_coordinate(x, y, w, h, depth, 1.047198) 
    X_Coordinate = Average_depth + 0.04
    print(X_Coordinate,Z_Coordinate,box_height, box_width) 
    camera_coordinate = np.eye(4,4)
    camera_coordinate[0:3,3] = camera

    box_wrt_camera = np.eye(4,4)
    box_wrt_camera[0:3,3] = [Average_depth+0.04,-Y_Coordinate,Z_Coordinate]

    box_wrt_origin = np.matmul(camera_coordinate,box_wrt_camera)
    box = np.eye(6,1)
    box[0:3,0] = box_wrt_origin[0:3,3]
    box[3:6,0] = [0,0,0]
    data = Float64MultiArray()
    data.data = box
    try:
      box_coordinate.publish(data)
      # print(box_wrt_origin) 
    except(Exception):
      print(Exception)
    # cv2.imshow("Keypoints",im)
    # cv2.waitKey(1)


# ------------------------------Callback for Mono Camera---------------------------------------

def callback_mono(data):
  global function_control
  if function_control == 2.0:
    try:
      box_coordinate = rospy.Publisher("/box/Coordinate_Mono", Float64MultiArray, queue_size =1)
    except(Exception):
      print(Exception)
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    global threshVal, x, y, w, h, camera, box_height, X_Coordinate, Y_Coordinate, Z_Coordinate, camera_wrt_origin
    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")
    
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    # print(current_frame)
    # img_blur = cv2.GaussianBlur(current_frame, (7,7), 0)
    # im = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
    # ret,thresh = cv2.threshold(im, 170, 255, cv2.THRESH_BINARY)
    # drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV) 
    min_green = np.array([0,130,0]) 
    max_green = np.array([255,255,255]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(current_frame,current_frame, mask= mask_g)
    edges = cv2.Canny(image=res_g, threshold1=85, threshold2=255)
    contours, hierarchy= cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    # cv2.drawContours(image=im, contours=contours, contourIdx=0, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    rectangle = []
    maxArea = 20000.0
    minArea = 2000.0
    Orientation_z = 0
    # print("cont",len(contours))
    # print("In Mono")
    for count in contours:
        area = cv2.contourArea(count)
        # print(area)
        color = (255,0,0)
        epsilon = 0.01 * cv2.arcLength(count, True)
        approximations = cv2.approxPolyDP(count, epsilon, True)
        if minArea < area < maxArea : 
            # cv2.drawContours(im, [approximations], 0, (0), 3)
            (x,y),(w,h),Orientation = cv2.minAreaRect(count)
            # print(x,y,w,h, Orientation)
            rect =cv2.minAreaRect(count)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            Orientation_z = Orientation*math.pi/180
            # print(box)    
            boundRect = cv2.boundingRect(box)
            cv2.rectangle(current_frame, (int(x), int(y)),(int(x+1), int(y+1)), color, 2)
    # cv2.imshow("rect",im)
    # rospy.Subscriber('/camera2/depth/image_raw', Image, callback_depth)
    depth = 0.4 - 0.0635 - box_height/2
    X_Coordinate, Y_Coordinate, box_length, box_width = detect_coordinate(x, y, w, h, depth,1.3962634) 
    # print(Y_Coordinate,X_Coordinate, box_length, box_width, Orientation_z) 
    Z_Coordinate = depth + box_height/2
    # camera_coordinate = np.eye(4,4)
    # camera_coordinate[0:3,3] = camera
    box_wrt_camera = np.eye(4,4)
    box_wrt_camera[0:3,3] = [Z_Coordinate,X_Coordinate,Y_Coordinate]
    rotation_abt_zy = np.matmul([[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]],[[math.cos(Orientation_z),-math.sin(Orientation_z),0,0],[math.sin(Orientation_z),math.cos(Orientation_z),0,0],[0,0,1,0],[0,0,0,1]])
    box_wrt_camera = np.matmul(box_wrt_camera,rotation_abt_zy)
    # print(box_wrt_camera)
    # box_wrt_camera[0:3,0:3] = np.matmul([[math.cos(Orientation), (-math.sin(Orientation)), 0],[math.sin(Orientation), math.cos(Orientation), 0],[0 , 0, 1]],[[0,0,-1],[0,1,0],[1,0,0]])  camera_wrt_endeff = np.eye(4,4)
    # camera_wrt_endeff[0:3,3] = [] 
    # Box -> Camera -> Link5 -> End Effector -> Origin
    # box_wrt_Link5 = 
    # box_wrt_endeff =
    # print(camera_wrt_origin)
    box_wrt_origin = np.matmul(camera_wrt_origin,box_wrt_camera)
    # print(box_wrt_camera)
    Rotation_matrix = R.from_dcm(box_wrt_origin[0:3,0:3])
    z,y,x = Rotation_matrix.as_euler('zyx', degrees = False)
    # print(z,y,x)
    box = np.eye(9,1)
    box[0:3,0] = box_wrt_origin[0:3,3]
    box[3:6,0] = [x,y,-z]
    box[6:9,0] = [box_length,box_width,box_height]
    # print(box_wrt_origin)
    data = Float64MultiArray()
    data.data = box
    # try:
    # print(box)
    box_coordinate.publish(data)
    #   # print(box_wrt_origin) 
    # except(Exception):
    #   print(Exception)
    # cv2.imshow("Keypoints",im)
    # cv2.waitKey(1)

# ------------------------------Callback for Calculating depth values---------------------------------------

def callback_depth(data):
    # for i in range(int(480)):
    #     for j in range(int(640)):
    #       print(i)
  global function_control
  if function_control == 1.0 :
    try:
      br = CvBridge()
      global Average_depth, x, y, w, h
      current_frame = br.imgmsg_to_cv2(data)
      # cv2.imshow("Depth_map", current_frame)
      current_frame = np.nan_to_num(current_frame)
        
        # print(len(frame))
      box = np.eye(int(h),int(w))
      for i in range(int(h)):
        for j in range(int(w)):
          frame = current_frame[int(y-(h/2)+i)][int(x-(w/2)+j)]
          if frame == 0.0:
            box[i][j] = 100
          else:
            box[i][j] = frame
      # print(np.min(box))
      # box = current_frame[int(y-(h/2)):int(y+(h/2))][int(x-(w/2)):int(x+(w/2))]
      # print(sum)
      # Average_depth = sum/(w*h)
      if 0.0 < np.min(box) < 1.0:
        Average_depth = np.min(box)
      # print(current_frame)
      # print("Sum",sum)
      # print("Depth :", Average_depth)
      # print(box)
    except(Exception):
      exit


# ------------------------------Initial function---------------------------------------

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  global camera_wrt_origin
  rospy.init_node('video_sub_py', anonymous=True)
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer) 
  rate = rospy.Rate(10.0)
  # Node is subscribing to the video_frames topic
  # rospy.Subscriber('/camera2/color/image_raw', Image, callback)
  rospy.Subscriber('/camera/state', Float64, callback_state)
  # try:
  # trans = tfBuffer.lookup_transform('world', 'Base', rospy.Time())
  # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  # print(trans)
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform('Base', 'camera_link', rospy.Time())
      r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
      z,y,x =r.as_euler('zyx', degrees=True)
      rotationzyx = np.eye(4,4)
      rotationzyx[0:3,0:3] = r.as_dcm()
      for i in range(4):
        for j in  range(4):
          rotationzyx[i][j] = round(rotationzyx[i][j],1)
      # rotationyx = np.matmul([[math.cos(y),0,math.sin(y),0],[0,1,0,0],[-math.sin(y),0,math.cos(y),0],[0,0,0,1]],[[1,0,0,0],[0,math.cos(x),-math.sin(x),0],[0,math.sin(x),math.cos(x),0],[0,0,0,1]])
      # rotationzyx = np.matmul([[math.cos(z),-math.sin(z),0,0],[math.sin(z),math.cos(z),0,0],[0,0,1,0],[0,0,0,1]],rotationyx)
      camera_wrt_origin = np.matmul([[1,0,0,trans.transform.translation.x],[0,1,0,trans.transform.translation.y],[0,0,1,trans.transform.translation.z],[0,0,0,1]],rotationzyx)
      # print(camera_wrt_origin)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rate.sleep()
      continue
  # rospy.Subscriber('/camera2/color/image_raw', Image, callback)
  
  # rospy.Subscriber('/camera1/color/image_raw', Image, callback_mono)
  # rospy.Subscriber('/camera2/depth/image_raw', Image, callback_depth)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()