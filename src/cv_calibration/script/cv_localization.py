#!/usr/bin/python3.8
# -*- coding: utf-8 -*-

import YOLOv8_Detect.openvino_yolo as myDetect
import rospy
import rospkg
import cv2
import numpy as np
from geometry_msgs.msg import Point32
from ultralytics import YOLO

rospy.init_node("point_publisher", anonymous=True)
point_pub=rospy.Publisher("/vase_point",Point32,queue_size=10)
rate=rospy.Rate(10)

model_path=rospkg.RosPack().get_path("cv_calibration")+'/script/YOLOv8_Detect/best_openvino_model/'

ov_model = YOLO(model_path,task="detect")
cap = cv2.VideoCapture(0)
img_size = (640, 480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_size[1])
cap.set(cv2.CAP_PROP_FPS, 30)


while not rospy.is_shutdown():
    ret, frame = cap.read()
    frame = myDetect.undistort(myDetect.g200_params, frame)
    result = ov_model(frame)
    result = result[0]
    cv2.imshow("detect",result.plot())
    if len(result)>0 and result.boxes.conf[0]>0.5:
        point_msg=Point32()
        point_msg.x=result.boxes.xywhn[0][0]
        point_msg.y=result.boxes.xywhn[0][1]
        point_msg.z=0
        point_pub.publish(point_msg)
    # rate.sleep()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()