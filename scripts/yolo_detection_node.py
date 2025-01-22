#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        self.model = YOLO(rospy.get_param("~model_path"))
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('detected_objects', Detection2DArray, queue_size=1)
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        
        detections = Detection2DArray()
        detections.header = msg.header
        
        for result in results:
            for box in result.boxes:
                detection = Detection2D()
                detection.bbox = BoundingBox2D()
                detection.bbox.center.x = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
                detection.bbox.center.y = (box.xyxy[0][1] + box.xyxy[0][3]) / 2
                detection.bbox.size_x = box.xyxy[0][2] - box.xyxy[0][0]
                detection.bbox.size_y = box.xyxy[0][3] - box.xyxy[0][1]
                detection.results.score = box.conf[0]
                detection.results.class_id = box.cls[0]
                detections.detections.append(detection)
        
        self.pub.publish(detections)

if __name__ == '__main__':
    rospy.init_node('yolo_detector')
    detector = YOLODetector()
    rospy.spin()