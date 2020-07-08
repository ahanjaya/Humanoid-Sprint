#!/usr/bin/env python

import os
import cv2
import yaml
import rospy
import rospkg
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

class Line:
    def __init__(self):
        rospy.init_node('srospy.loginfo_aruco')
        rospy.loginfo("[Line] Martahon Vision - Running")

        # Config File
        self.rospack    = rospkg.RosPack()
        self.cfg_file   = self.rospack.get_path("marathon_vision") + "/config/color.yaml"
        
        self.bridge     = CvBridge()
        self.frame_w    = rospy.get_param("/usb_cam/image_width")
        self.frame_h    = rospy.get_param("/usb_cam/image_height")
        self.source_img = np.zeros((self.frame_w, self.frame_h, 3), np.uint8)
        
        # Color Config
        self.b_lower, self.b_upper, self.b_util = None, None, None
        self.y_lower, self.y_upper, self.y_util = None, None, None
        self.blue_conf, self.yellow_conf = False, False
        self.line_data = Pose2D()
        self.debug     = False
        self.kernel    = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))

        # Subscriber
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # Publisher
        self.line_pos_pub = rospy.Publisher("/marathon/line/position", Pose2D, queue_size=1)
        

    def img_callback(self, msg):
        try:
            self.source_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

    def update_parameter(self, x):
        pass
    
    def create_trackbar(self):
        cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('H_Max', 'Control',255,255, self.update_parameter)
        cv2.createTrackbar('H_Min', 'Control',0,  255, self.update_parameter)
        cv2.createTrackbar('S_Max', 'Control',255,255, self.update_parameter)
        cv2.createTrackbar('S_Min', 'Control',0,  255, self.update_parameter)
        cv2.createTrackbar('V_Max', 'Control',255,255, self.update_parameter)
        cv2.createTrackbar('V_Min', 'Control',0,  255, self.update_parameter)
        cv2.createTrackbar('Erode', 'Control',0,  20,  self.update_parameter)
        cv2.createTrackbar('Dilate','Control',0,  20,  self.update_parameter)
        cv2.createTrackbar('Size',  'Control',8000, 30000, self.update_parameter)

    def set_trackbar(self, color):
        if color == "Blue":
            lower, upper, utils = self.b_lower, self.b_upper, self.b_util
        elif color == "Yellow":
            lower, upper, utils = self.y_lower, self.y_upper, self.y_util
        
        cv2.setTrackbarPos('H_Max', 'Control', upper[0])
        cv2.setTrackbarPos('H_Min', 'Control', lower[0])
        cv2.setTrackbarPos('S_Max', 'Control', upper[1])
        cv2.setTrackbarPos('S_Min', 'Control', lower[1])
        cv2.setTrackbarPos('V_Max', 'Control', upper[2])
        cv2.setTrackbarPos('V_Min', 'Control', lower[2])
        cv2.setTrackbarPos('Erode', 'Control', utils[0])
        cv2.setTrackbarPos('Dilate','Control', utils[1])
        cv2.setTrackbarPos('Size',  'Control', utils[2])

    def get_trackbar(self):
        h_max  = cv2.getTrackbarPos("H_Max", "Control")
        h_min  = cv2.getTrackbarPos("H_Min", "Control")
        s_max  = cv2.getTrackbarPos("S_Max", "Control")
        s_min  = cv2.getTrackbarPos("S_Min", "Control")
        v_max  = cv2.getTrackbarPos("V_Max", "Control")
        v_min  = cv2.getTrackbarPos("V_Min", "Control")
        erode  = cv2.getTrackbarPos("Erode", "Control")
        dilate = cv2.getTrackbarPos("Dilate", "Control")
        size   = cv2.getTrackbarPos("Size", "Control")
        
        lower = np.array([h_min, s_min,  v_min])
        upper = np.array([h_max, s_max,  v_max])
        utils = np.array([erode, dilate, size])
        return lower, upper, utils

    def update_config(self, default=True, config=None, color=None):
        if default:
            lower = np.array([ 0, 0, 0 ])
            upper = np.array([ 255, 255, 255 ])
            utils = np.array([ 0, 0, 1000 ])
        else:
            lower = np.array([ config[color]['H_Min'], config[color]['S_Min'],  config[color]['V_Min'] ])
            upper = np.array([ config[color]['H_Max'], config[color]['S_Max'],  config[color]['V_Max'] ])
            utils = np.array([ config[color]['Erode'], config[color]['Dilate'], config[color]['Size']  ])

        return lower, upper, utils

    def load_config(self):

        if os.path.exists(self.cfg_file):
            with open(self.cfg_file,'r') as f:
                config = yaml.safe_load(f)
            self.b_lower, self.b_upper, self.b_util = self.update_config(False, config, 'Blue')
            self.y_lower, self.y_upper, self.y_util = self.update_config(False, config, 'Yellow')
            rospy.loginfo("[Line] Load initial config")
        
        else:
            self.b_lower, self.b_upper, self.b_util = self.update_config()
            self.y_lower, self.y_upper, self.y_util = self.update_config()

            rospy.logwarn("[Line] File {} is not exists".format(self.cfg_file))

    def create_color(self, lower, upper, utils):
        color = { 'H_Min': int(lower[0]), 'S_Min':  int(lower[1]), 'V_Min': int(lower[2]),
                  'H_Max': int(upper[0]), 'S_Max':  int(upper[1]), 'V_Max': int(upper[2]),
                  'Erode': int(utils[0]), 'Dilate': int(utils[1]),  'Size': int(utils[2]) }
        return color
    
    def save_config(self):
        blue   = self.create_color(self.b_lower, self.b_upper, self.b_util)
        yellow = self.create_color(self.y_lower, self.y_upper, self.y_util)
        config = { 'Blue': blue, 'Yellow': yellow}
        with open(self.cfg_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        rospy.loginfo('[Line] Save Colors Config: {}'.format(self.cfg_file))

    def refresh(self, color):
        self.blue_conf   = False 
        self.yellow_conf = False

        cv2.destroyAllWindows()
        self.create_trackbar()
        self.set_trackbar(color)

    def kill_node(self):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("[Line] Shutdown Time...")

    def run(self):

        self.load_config()
        
        while not rospy.is_shutdown():
            rgb_img   = self.source_img.copy()
            hsv_img   = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
            
            # color filtering
            blue_mask   = cv2.inRange(hsv_img,  self.b_lower, self.b_upper)
            blue_mask   = cv2.erode (blue_mask, self.kernel, iterations=self.b_util[0])
            blue_mask   = cv2.dilate(blue_mask, self.kernel, iterations=self.b_util[1])

            yellow_mask = cv2.inRange(hsv_img, self.y_lower, self.y_upper)
            yellow_mask = cv2.erode (yellow_mask, self.kernel, iterations=self.y_util[0])
            yellow_mask = cv2.dilate(yellow_mask, self.kernel, iterations=self.y_util[1])

            # or binary
            line_mask = cv2.bitwise_or(blue_mask, yellow_mask)

            # contour
            _, line_contours, _ = cv2.findContours(line_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cx, cy = -1, -1

            if len(line_contours) > 0:
                # cnt = max(line_contours, key=cv2.contourArea)

                sorted_line_contours = sorted(line_contours, key=cv2.contourArea, reverse=True)
                for cnt in sorted_line_contours:
                    cntr_area = cv2.contourArea(cnt)
                    # print(cntr_area)

                    # filter size
                    if cntr_area >= self.b_util[2] : #and cntr_area <= self.y_util[2]:
                        box_x, box_y, box_w, box_h = cv2.boundingRect(cnt)
                        # filter rectangle, by judging height > width
                        # if box_h > box_w:
                        cx = box_x + box_w / 2
                        cy = box_y + box_h / 2
                        cv2.circle(rgb_img, (cx, cy), 5, (255, 0, 0), -1)
                        cv2.rectangle(rgb_img, (box_x, box_y), (box_x + box_w, box_y + box_h), (0,255,0), 2)
                        break

            # color config
            if self.blue_conf:
                self.b_lower, self.b_upper, self.b_util = self.get_trackbar()
                cv2.imshow('blue', blue_mask)
                
            elif self.yellow_conf:
                self.y_lower, self.y_upper, self.y_util = self.get_trackbar()
                cv2.imshow('yellow', yellow_mask)

            # arcuo coordinates
            self.line_data.x = cx
            self.line_data.y = cy

            # publishing line result
            self.line_pos_pub.publish(self.line_data)

            k = cv2.waitKey(1)
            # cv2.imshow('line', rgb_img)
            
            if k == 27 or k == ord('q'):
                rospy.loginfo('exit')
                break

            elif k == ord('b'):
                rospy.loginfo('[Line] Blue colors enter .. ')
                self.refresh('Blue')
                self.blue_conf = True

            elif k == ord('y'):
                rospy.loginfo('[Line] Yellow colors enter .. ')
                self.refresh('Yellow')
                self.yellow_conf = True

            elif k == ord('x'):
                self.blue_conf = self.yellow_conf = False
                cv2.destroyAllWindows()

            elif k == ord('s'):
                self.save_config()

            elif k == ord('l'):
                self.load_config()

        
        rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    line = Line()
    line.run()