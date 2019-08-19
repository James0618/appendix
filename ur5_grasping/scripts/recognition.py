#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Recognition:
    def __init__(self):
        rospy.init_node('image_recongition', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.can_pub = rospy.Publisher('/recognition/can', Image)
        self.location_pub = rospy.Publisher('/recognition/location', Pose)
        self.bridge = CvBridge()
        self.pose_goal = Pose()
        self.pose_goal.orientation.x = 0
        self.pose_goal.orientation.y = 0.707
        self.pose_goal.orientation.x = 0
        self.pose_goal.orientation.w = 0.707
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            
        # convert bgr image to hsv and find the can by threshold
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        low_green_can = np.array([10, 200, 10])
        upper_green_can = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, low_green_can, upper_green_can)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        # find the center of the can
        imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 100, 255, 0)
        image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        img_contour = cv2.drawContours(cv_image, contours, 0, (0,0,255), 2)
        
        # compute the location of coke_can
        try:
            cnt = contours[0]
            M = cv2.moments(cnt)

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            [hx, hy] = cv_image.shape[0:2]
            y = 0.1 * (800.0 / 90.0) * (hx / 2 - cx) / float(hx)       # the real scale
            x = 0.1 * (800.0 / 90.0) * (hy / 2 - cy) / float(hy) + 0.3 # the real scale
            self.pose_goal.position.x = x
            self.pose_goal.position.y = y
            self.pose_goal.position.z = 0.125
            
            self.location_pub.publish(self.pose_goal)
        
        except (ZeroDivisionError, IndexError) as e:
            pass
                   
        try:
            self.can_pub.publish(self.bridge.cv2_to_imgmsg(img_contour, 'bgr8'))
        except CvBridgeError as e:
            print(e)
            

if __name__ == '__main__':
    rec = Recognition()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down!')

