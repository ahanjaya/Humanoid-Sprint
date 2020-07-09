#!/usr/bin/env python3

import os
import cv2
import yaml
import rospy
import rospkg
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class Marker:
    def __init__(self):
        rospy.init_node('marathon_marker_cv')
        rospy.loginfo("[Marker] Martahon Vision - Running")

        self.python2 = False
        if self.python2:
            self.bridge = CvBridge()

        self.frame_w    = rospy.get_param("/usb_cam/image_width")
        self.frame_h    = rospy.get_param("/usb_cam/image_height")
        self.source_img = np.zeros((self.frame_w, self.frame_h, 3), np.uint8)
        self.debug      = False
        self.arrow      = String()

        self.pi_4 = np.pi * 4

        # Subscriber
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # Publisher
        self.arrow_state_pub = rospy.Publisher("/marathon/marker/arrow", String, queue_size=1)

    def img_callback(self, msg):
        if self.python2:
            try:
                self.source_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.loginfo(e)
        else:
            dt  = np.dtype(np.uint8)
            dt  = dt.newbyteorder('>')
            arr = np.frombuffer(msg.data, dtype=dt)

            arr = np.reshape(arr, (self.frame_h, self.frame_w ,3))
            self.source_img = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def kill_node(self):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("[Marker] Shutdown Time...")

    def run(self):

        while not rospy.is_shutdown():
            rgb_img    = self.source_img.copy()
            blur_img   = cv2.GaussianBlur(rgb_img, (5,5), 0)
            thresh_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
            edge_img   = cv2.Canny(thresh_img, 100, 200)
            self.arrow = "None"
            
            # contour
            # _, contours, _ = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # _, contours, _ = cv2.findContours(edge_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours, _ = cv2.findContours(edge_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # cv2.drawContours(rgb_img, contours, -1, (255, 0, 0), -1)
                sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
                # cnt = max(line_contours, key=cv2.contourArea)
                
                for cnt in sorted_contours:
                    cnt = cv2.convexHull(cnt)

                    if len(cnt) < 5:
                        continue
                    # filter contour area
                    cntr_area = cv2.contourArea(cnt)
                    # print(cntr_area)
                    if cntr_area <= 4000 or cntr_area >= 10000:
                        continue

                    # filter circular
                    arclen = cv2.arcLength(cnt , True)
                    # print('arc len: {}'.format(arclen))
                    if arclen >= 400 or arclen <= 300:
                        continue

                    circularity = (self.pi_4 * cntr_area) / (arclen * arclen)
                    # print(circularity)
                    if circularity > 0.3:
                        ellipse = cv2.fitEllipse(cnt)
                        angle   = round(ellipse[2], 2)
                        cv2.ellipse(rgb_img, ellipse, (0,255,0), 2)
                        ex     = int(ellipse[0][0])
                        ey     = int(ellipse[0][1])
                        label1 = "area: {} arclen:{:.1f}".    format(cntr_area, arclen)
                        label2 = "angle: {} circular: {:.1f}".format(angle, circularity)
                        cv2.putText(rgb_img, label1, (ex, ey),    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                        cv2.putText(rgb_img, label2, (ex, ey+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

                        # # arrow decision
                        if angle >= 150.0 or angle <= 20.0:
                            self.arrow = "straight"
                        elif angle >= 85.0 and angle <= 130.0:
                            self.arrow = "left"
                        elif angle >= 40.0 and angle < 85.0:
                            self.arrow = "right"

                        # print('angle: {} \t  arrow: {}'.format(angle, self.arrow))
                        break
            
            # publishing
            cv2.putText(rgb_img, self.arrow, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.arrow_state_pub.publish(self.arrow)
            
            k = cv2.waitKey(1)
            cv2.imshow('arrow', rgb_img)
            # cv2.imshow('edge', edge_img)
            
            if k == 27 or k == ord('q'):
                rospy.loginfo('exit')
                break

        rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    marker = Marker()
    marker.run()