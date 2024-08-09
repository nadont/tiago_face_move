#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from pal_detection_msgs.msg import FaceDetection

class FaceFollower:
    def __init__(self):
        rospy.init_node('face_follower')

        # Subscribe to the face_coordinates topic
        self.face_sub = rospy.Subscriber('face_coordinates', FaceDetection, self.face_callback)
        # Initialize the action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Flag to check if we have already processed a face detection
        self.face_detected = False

    def face_callback(self, face):
        # Process only if a face has not been detected yet
        if not self.face_detected:
            rospy.loginfo(f"Face detected at x: {face.x}, y: {face.y}, width: {face.width}, height: {face.height}")

            # Convert from image coordinates to world coordinates

            #conversion_factor = 0.02  # meters per pixel (tune based on actual tests)
            conversion_factor = 0.002
            x = face.x * conversion_factor  # Convert from pixels to meters
            y = face.y * conversion_factor  # Convert from pixels to meters

            # Move to the detected face position
            self.move_to_goal(x, y, 0)

            # Move back to the origin (0, 0, 0)
            self.move_to_goal(0, 0, 0)

            # Set the flag to True to avoid further face detections
            self.face_detected = True

            # After reaching the origin, send a shutdown signal to end the program
            rospy.signal_shutdown("Face detected and returned to origin. Exiting...")

    def move_to_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.logerr("Goal failed or was preempted!")

if __name__ == '__main__':
    try:
        FaceFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
