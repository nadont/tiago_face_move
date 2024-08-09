#!/usr/bin/env python

import rospy
from pal_detection_msgs.msg import FaceDetections
from std_msgs.msg import Int32

class FaceCount:
    def __init__(self):
        rospy.init_node('face_count', anonymous=True)
        
        self.face_count_pub = rospy.Publisher('face_count', Int32, queue_size=1)
        self.face_sub = rospy.Subscriber('faces', FaceDetections, self.face_callback)
        
        rospy.loginfo("FaceCount node initialized")
        rospy.loginfo("Subscribing to faces topic")

    def face_callback(self, msg):
        face_count = len(msg.faces)
        rospy.loginfo(f"Detected {face_count} faces")
        
        self.face_count_pub.publish(face_count)

if __name__ == '__main__':
    try:
        fc = FaceCount()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
