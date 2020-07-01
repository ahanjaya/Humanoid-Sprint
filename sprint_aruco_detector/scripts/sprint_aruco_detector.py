#!/usr/bin/env python

import cv2
import rospy
import numpy as np

from cv2 import aruco
from sensor_msgs.msg import Image
from robot_msgs.msg import ArucoData 
from cv_bridge import CvBridge, CvBridgeError

class Vision:
    def __init__(self):
        rospy.init_node('sprint_aruco')
        rospy.loginfo("[Vision] Sprint Aruco - Running")

        self.bridge = CvBridge()

        self.frame_w      = rospy.get_param("/usb_cam/image_width")
        self.frame_h      = rospy.get_param("/usb_cam/image_height")
        self.source_img   = np.zeros((self.frame_w, self.frame_h, 3), np.uint8)
        self.aruco_dict   = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_data   = ArucoData()
        self.debug        = False
        
        # Subscriber
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # Publisher
        self.aruco_pos_pub = rospy.Publisher("/sprint/marker/position", ArucoData, queue_size=1)
        self.aruco_img_pub = rospy.Publisher("/sprint/marker/image",    Image,     queue_size=1)

    def img_callback(self, msg):
        try:
            self.source_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

    def kill_node(self):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("[Vision] Shutdown Time...") 

    def run(self):
        
        while not rospy.is_shutdown():
            rgb_img    = self.source_img.copy()
            blur_img   = cv2.GaussianBlur(rgb_img, (5,5), 0)
            gray_img   = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_params)
            cx, cy, size = -1, -1, -1

            if len(corners) > 0 and ids[0,0] == 1:
                M = cv2.moments(corners[0])
                cx   = int(M["m10"] / M["m00"])
                cy   = int(M["m01"] / M["m00"])
                size = int(M["m00"])
                
                if self.debug:
                    rospy.loginfo("[Vision] ID : {}, Pos: {},{}".format(ids[0,0], cx, cy))

                cv2.circle(rgb_img, (cx,cy), 5, (0, 0, 255), -1)
                aruco.drawDetectedMarkers(rgb_img, corners, ids)

            # arcuo coordinates
            self.aruco_data.x = cx
            self.aruco_data.y = cy
            self.aruco_data.size = size

            # publishing aruco result
            self.aruco_pos_pub.publish(self.aruco_data)
            self.aruco_img_pub.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

            k = cv2.waitKey(1)
            # cv2.imshow('image', rgb_img)
            # if k == 27 or k == ord('q'):
            #     break
        
        rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    vision = Vision()
    vision.run()