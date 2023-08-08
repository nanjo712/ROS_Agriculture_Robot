#!/usr/bin/python3.8
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import PointStamped

cap = cv2.VideoCapture(0)
hsv_low = np.array([0, 127, 212])
hsv_high = np.array([7, 193, 255])

rospy.init_node("point_publisher", anonymous=True)
point_pub=rospy.Publisher("/point",PointStamped,queue_size=10)
rate=rospy.Rate(10)
cnt=0


while not rospy.is_shutdown():
    ret, frames = cap.read()
    mask = cv2.cvtColor(frames, cv2.COLOR_BGR2HSV)  # BGR转HSV
    mask = cv2.inRange(mask, hsv_low, hsv_high)  # 通过HSV的高低阈值，提取图像部分区域
    # kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])  # x坐标
        cy = int(M["m01"] / M["m00"])  # y坐标
        print("Object found at: ({}, {})".format(cx, cy))
        center = (cx, cy)  # 假设(cx, cy)是你要标记的坐标
        msg=PointStamped()
        msg.header.frame_id,msg.header.seq,msg.header.stamp="camera0",cnt,rospy.Time.now()
        msg.point.x,msg.point.y,msg.point.z=cx,cy,0
        cnt+=1
        cv2.circle(frames, center, 5, (0,0,0), -1)  # -1表示将标记点填充为实心圆
    cv2.imshow('mask', mask)
    cv2.imshow("BGR", frames)  # 显示图片
    rate.sleep()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()