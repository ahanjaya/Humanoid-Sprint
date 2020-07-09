#!/usr/bin/env python3

import re
import cv2
import PIL
import rospy
import rospkg
import collections
import numpy as np
import tflite_runtime.interpreter as tflite

from std_msgs.msg import String
from sensor_msgs.msg import Image
from marathon_vision import common

class BBox(collections.namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])):
    """Bounding box.
    Represents a rectangle which sides are either vertical or horizontal, parallel
    to the x or y axis.
    """
    __slots__ = ()

class MobileNet:
    def __init__(self):
        rospy.init_node('marathon_marker_ssd')
        rospy.loginfo("[Marker] Martahon Vision - Running")

        self.object     = collections.namedtuple('Object', ['id', 'score', 'bbox'])
        self.top_k      = 3   # number of categories with highest score to display
        self.threshold  = 0.6 # confident level

        # Config File
        self.rospack    = rospkg.RosPack()
        self.labels     = self.rospack.get_path("marathon_vision") + "/data/coco_labels.txt"
        # self.labels     = self.rospack.get_path("marathon_vision") + "/data/arrow_labels.txt"
        self.model      = self.rospack.get_path("marathon_vision") + "/data/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite"
        
        self.frame_w    = rospy.get_param("/usb_cam/image_width")
        self.frame_h    = rospy.get_param("/usb_cam/image_height")
        self.source_img = np.zeros((self.frame_w, self.frame_h, 3), np.uint8)
        self.debug      = False
        self.arrow      = String()

        # Subscriber
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # Publisher
        self.arrow_state_pub = rospy.Publisher("/marathon/marker/arrow", String, queue_size=1)

    def img_callback(self, msg):
        dt  = np.dtype(np.uint8)
        dt  = dt.newbyteorder('>')
        arr = np.frombuffer(msg.data, dtype=dt)

        arr = np.reshape(arr, (self.frame_h, self.frame_w ,3))
        self.source_img = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def load_labels(self, path):
        p = re.compile(r'\s*(\d+)(.+)')
        with open(path, 'r', encoding='utf-8') as f:
            lines = (p.match(line).groups() for line in f.readlines())
            return {int(num): text.strip() for num, text in lines}

    def get_output(self, interpreter, score_threshold, top_k, image_scale=1.0):
        """Returns list of detected objects."""
        boxes     = common.output_tensor(interpreter, 0)
        class_ids = common.output_tensor(interpreter, 1)
        scores    = common.output_tensor(interpreter, 2)
        count     = int(common.output_tensor(interpreter, 3))

        def make(i):
            ymin, xmin, ymax, xmax = boxes[i]
            return self.object(
                id    = int(class_ids[i]),
                score = scores[i],
                bbox  = BBox(xmin=np.maximum(0.0, xmin), ymin=np.maximum(0.0, ymin),
                             xmax=np.minimum(1.0, xmax), ymax=np.minimum(1.0, ymax)))

        return [make(i) for i in range(top_k) if scores[i] >= score_threshold]

    def kill_node(self):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("[Marker] Shutdown Time...")

    def run(self):
        labels      = self.load_labels(self.labels)
        interpreter = common.make_interpreter(self.model)
        interpreter.allocate_tensors()

        while not rospy.is_shutdown():
            bgr_img = self.source_img.copy()
            rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
            pil_img = PIL.Image.fromarray(rgb_img)

            common.set_input(interpreter, pil_img)
            interpreter.invoke()
            objs = self.get_output(interpreter, score_threshold=self.threshold, top_k=self.top_k)

            self.arrow = "None"
            for obj in objs:
                x0, y0, x1, y1 = list(obj.bbox)
                x0, y0, x1, y1 = int(x0*self.frame_w), int(y0*self.frame_h), int(x1*self.frame_w), int(y1*self.frame_h)
                percent  = int(100 * obj.score)

                obj_length = x1 - x0
                obj_width  = y1 - y0
                obj_area   = obj_length * obj_width
                rospy.loginfo("[Marker] Object Area: {}".format(obj_area))

                if obj_area <= 50000:
                    self.arrow = labels.get(obj.id, obj.id)
                    label      = '{}% {}'.format(percent, self.arrow)
                    cv2.rectangle(bgr_img, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    cv2.putText(bgr_img, label, (x0, y0+30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                    break

            # publishing
            self.arrow_state_pub.publish(self.arrow)
            
            k = cv2.waitKey(1)
            cv2.imshow('arrow', bgr_img)
            
            if k == 27 or k == ord('q'):
                rospy.loginfo('exit')
                break

        rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    ssd_v2 = MobileNet()
    ssd_v2.run()