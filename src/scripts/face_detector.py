#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pal_detection_msgs.msg import FaceDetections, FaceDetection
import cv2
from facenet_pytorch import MTCNN
import torch

class FaceDetector:
    def __init__(self):
        rospy.init_node('cnn_face_detector', anonymous=True)
        
        self.verbose_publishing = rospy.get_param('~verbose_publishing', True)
        self.bridge = CvBridge()

        # Initialize MTCNN
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.mtcnn = MTCNN(keep_all=True, device=self.device)
        
        rospy.loginfo("Creating face detector")

        self.image_sub = rospy.Subscriber('image', Image, self.image_callback)
        rospy.loginfo(f"Subscribing to image topic: {rospy.resolve_name('image')}")
        
        self.face_pub = rospy.Publisher('faces', FaceDetections, queue_size=1)
        self.debug_pub = rospy.Publisher('debug', Image, queue_size=1)
        self.face_coordinates_pub = rospy.Publisher('face_coordinates', FaceDetection, queue_size=1)

        rospy.loginfo("Spinning to serve callbacks ...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        rospy.loginfo("Image received")
        self.detect_faces(cv_image)

    def detect_faces(self, img):
        # Convert to RGB
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Detect faces
        boxes, _ = self.mtcnn.detect(rgb_img)

        if boxes is None:
            boxes = []

        rospy.loginfo(f"Detected {len(boxes)} faces")
        self.publish_detections(boxes)
        self.publish_debug_image(img, boxes)
        self.publish_face_coordinates(boxes)

    def publish_detections(self, boxes):
        detections_msg = FaceDetections()
        detections_msg.header.stamp = rospy.Time.now()
        detections_msg.header.frame_id = "camera_frame"

        for box in boxes:
            detection = FaceDetection()
            detection.x, detection.y, detection.width, detection.height = int(box[0]), int(box[1]), int(box[2]-box[0]), int(box[3]-box[1])
            detection.eyesLocated = False
            detection.leftEyeX = 0
            detection.leftEyeY = 0
            detection.rightEyeX = 0
            detection.rightEyeY = 0
            detection.name = ""
            detection.confidence = 0
            detections_msg.faces.append(detection)

        self.face_pub.publish(detections_msg)

    def publish_face_coordinates(self, boxes):
        for box in boxes:
            detection = FaceDetection()
            detection.x, detection.y, detection.width, detection.height = int(box[0]), int(box[1]), int(box[2]-box[0]), int(box[3]-box[1])
            self.face_coordinates_pub.publish(detection)

    def publish_debug_image(self, img, boxes):
        for box in boxes:
            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)

        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.debug_pub.publish(debug_img_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    try:
        fd = FaceDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
