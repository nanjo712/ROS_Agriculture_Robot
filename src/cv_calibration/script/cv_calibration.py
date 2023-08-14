#!/usr/bin/python3.8
# -*- coding: utf-8 -*-

import YOLOv8_Detect.openvino_yolo as myDetect
import rospy
import rospkg
import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
from std_msgs import Int8

state = 0

def FSMStateCallback(msg):
    global state
    state = msg.data

rospy.init_node("point_publisher", anonymous=True)
point_pub=rospy.Publisher("/vase_point",PointStamped,queue_size=10)
state_sub=rospy.Subscriber("/FSM_state",Int8,)
rate=rospy.Rate(10)

model_path=rospkg.RosPack().get_path("cv_calibration")+'/script/YOLOv8_Detect/best_openvino_model/'

ov_model = YOLO(model_path,task="detect")
cap = cv2.VideoCapture(0)
img_size = (640, 480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_size[1])
cap.set(cv2.CAP_PROP_FPS, 30)

cnt=0

warm_up = 10

while warm_up != 0 :
    ret, frame = cap.read()
    frame = myDetect.undistort(myDetect.g200_params, frame)
    result = ov_model(frame,verbose=False)
    warm_up-=1

while not rospy.is_shutdown():
    if state != 2 :
        continue
    ret, frame = cap.read()
    frame = myDetect.undistort(myDetect.g200_params, frame)
    result = ov_model(frame,verbose=False)
    result = result[0]
    cv2.imshow("detect",result.plot())
    if len(result)>0 and result.boxes.conf[0]>0.5:
        point_msg=PointStamped()
        point_msg.header.stamp=rospy.Time.now()
        point_msg.header.seq=cnt
        cnt+=1
        point_msg.header.frame_id="camera_link"
        point_msg.point.x=result.boxes.xywhn[0][0]
        point_msg.point.y=result.boxes.xywhn[0][1]
        point_msg.point.z=0
        point_pub.publish(point_msg)
    #rate.sleep()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

rospy.spin()
cap.release()
cv2.destroyAllWindows()