#!/usr/bin/env python3

import rospy
import actionlib
import math
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        rospy.sleep(3)

        self.velocity_publisher = rospy.Publisher('/tab_vel', Twist, queue_size=10)
        self.sonar_subscriber = rospy.Subscriber('/sonar_base', Range, self.sona_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.face_count_sub = rospy.Subscriber('face_count', Int32, self.face_count_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.obstacle_detected = False
        self.follower_detection = False
        self.range_threshold = 0.5  # meters
        self.face_count = 0
        self.last_scan_time = rospy.Time.now()
        self.sonar_ranges = {
            "base_sonar_01_link": float('inf'),
            "base_sonar_02_link": float('inf'),
            "base_sonar_03_link": float('inf')
        }

        rospy.loginfo("MoveRobot node initialized")

    def scan_callback(self, scan):
        self.detect_obstacle(scan)

    def sona_callback(self, scan):
        self.detect_follower(scan)

    def face_count_callback(self, msg):
        self.face_count = msg.data
        rospy.loginfo(f"Face count received: {self.face_count}")
        
        if self.face_count > 0:
            #welcome_text = f"Hello, welcome to the University of Portsmouth, I will be navigating {self.face_count} of you here in this room."
            welcome_text = "Hello"
            self.speak(welcome_text)
            self.execute()
        else:
            rospy.loginfo("No faces detected, not moving the robot.")
            self.speak("No faces detected.")
            rospy.signal_shutdown("No faces detected, stopping the node.")

    def detect_obstacle(self, scan):
        if any(distance < self.range_threshold for distance in scan.ranges[191:382] if not math.isinf(distance)):
            self.obstacle_detected = True
            #rospy.loginfo("Obstacle detected within range threshold")
            self.obstacle_list = []
            for distance in scan.ranges[191:382]:
                if distance < self.range_threshold and not math.isinf(distance):
                    self.obstacle_list.append(distance)
                    #rospy.loginfo(f'Obstacle at distance: {distance}')
            #rospy.loginfo(f'Obstacle at distance: {obstacle_list}')
        else:
            self.obstacle_detected = False

    def detect_follower(self, scan):
        if scan.header.frame_id in self.sonar_ranges:
            self.sonar_ranges[scan.header.frame_id] = scan.range
        
        if any(range < 1.0 for range in self.sonar_ranges.values()):
            self.follower_detection = True
        else:
            self.follower_detection = False
            #rospy.loginfo(f"Sonar ranges: {self.sonar_ranges}")
            #rospy.loginfo(f"Follower detection: {self.follower_detection}")
 
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_publisher.publish(stop_msg)
        rospy.loginfo("Robot stopped due to obstacle detection")

    def move_forward(self, velocity, distance):
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        start_time = rospy.Time.now().to_sec()
        current_distance = 0
        
        while current_distance < distance:
            if self.obstacle_detected:
                self.stop_robot()
                rospy.loginfo("Movement interrupted by obstacle detection")
                obs_text = "Obstacle detected, please move out the way"
                self.speak(obs_text)
                rospy.sleep(3)
                start_time = rospy.Time.now().to_sec() - (current_distance/abs(velocity))
                if self.obstacle_detected:
                    obs_text1 = "Aborting navigation due to obstacle"
                    self.speak(obs_text1)
                    rospy.loginfo(f'Obstacle at distance: {self.obstacle_list}')
                    rospy.signal_shutdown("Movement complete, stopping the node.")
            elif not self.follower_detection:
                self.stop_robot()
                rospy.loginfo("Follower not detected")
                obs_text1 = "Follower not detected, please move closer to me"
                self.speak(obs_text1)
                rospy.sleep(3)
                start_time = rospy.Time.now().to_sec() - (current_distance/abs(velocity))
                if not self.follower_detection:
                    obs_text2 = "Aborting due to no follower"
                    self.speak(obs_text2)
                    rospy.signal_shutdown("Movement complete, stopping the node.")
            else:
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                current_time = rospy.Time.now().to_sec()
                current_distance = (current_time - start_time) * abs(velocity)
                rospy.loginfo(f"Current distance: {current_distance} meters")
        
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def move_backwards(self, velocity, distance):
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        start_time = rospy.Time.now().to_sec()
        current_distance = 0
        
        while current_distance < distance:
            if self.obstacle_detected:
                self.stop_robot()
                rospy.loginfo("Movement interrupted by obstacle detection")
                obs_text = "Obstacle detected, please move out the way"
                self.speak(obs_text)
                rospy.sleep(3)
                start_time = rospy.Time.now().to_sec() - (current_distance/abs(velocity))
                if self.obstacle_detected:
                    obs_text1 = "Aborting navigation due to obstacle"
                    self.speak(obs_text1)
                    rospy.loginfo(f'Obstacle at distance: {self.obstacle_list}')
                    rospy.signal_shutdown("Movement complete, stopping the node.") 
            else:
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                current_time = rospy.Time.now().to_sec()
                current_distance = (current_time - start_time) * abs(velocity)
                rospy.loginfo(f"Current distance: {current_distance} meters")
        
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def turn_robot(self, angular_velocity, angle):
        vel_msg = Twist()
        vel_msg.angular.z = angular_velocity

        start_time = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < angle:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()
            current_angle = (current_time - start_time) * abs(angular_velocity)
            rospy.loginfo(f"Current angle: {current_angle} radians")

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def speak(self, text):
        client = actionlib.SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()

        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = 'en_GB'

        rospy.loginfo(f"Sending TTS goal: {text}")
        client.send_goal(goal)
        client.wait_for_result()

        rospy.loginfo("Speech completed")

    def execute(self):
        try:
            rospy.loginfo("Executing move with obstacle detection and turning")
            self.turn_robot(-0.5, 2*1.58)
            rospy.sleep(1)
            rospy.loginfo("Robot is moving forward for 1 meter")
            rest_text2 = "Moving forward"
            self.speak(rest_text2)
            self.move_forward(0.2, 1.0)

            rospy.loginfo("Robot is waiting for 3 seconds")
            rospy.sleep(3)
            rest_text3 = "Waiting 3 seconds"
            self.speak(rest_text3)
            self.turn_robot(0.5, 2*1.58)

            rospy.loginfo("Robot is moving backward for 1 meter")
            rest_text4 = "Moving back"
            self.speak(rest_text4)
            self.move_backwards(0.2, 1.0)
            

            #rest_text = "Thank you for today, hope to see you here again."
            rest_text = "Thank you"
            self.speak(rest_text)

            rospy.signal_shutdown("Movement complete, stopping the node.")
        
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    robot_mover = MoveRobot()
    rospy.spin()

