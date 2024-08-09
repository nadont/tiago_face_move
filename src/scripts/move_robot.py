#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/key_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/sonar_base', Range, self.scan_callback)
        self.face_count_sub = rospy.Subscriber('face_count', Int32, self.face_count_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.obstacle_detected = False
        self.range_threshold = 1.0  # meters
        self.face_count = 0
        self.scan_period = rospy.Duration(3)  # 3 seconds scan period
        self.last_scan_time = rospy.Time.now()
        
        rospy.loginfo("MoveRobot node initialized")

    def scan_callback(self, scan):
        current_time = rospy.Time.now()
        if current_time - self.last_scan_time >= self.scan_period:
            self.last_scan_time = current_time
            self.detect_obstacle(scan)

    def face_count_callback(self, msg):
        self.face_count = msg.data
        rospy.loginfo(f"Face count received: {self.face_count}")
        
        if self.face_count > 0:
            self.execute()
        else:
            rospy.loginfo("No faces detected, not moving the robot.")
            rospy.signal_shutdown("No faces detected, stopping the node.")

    def detect_obstacle(self, scan):
        if scan.range < self.range_threshold:
            self.obstacle_detected = True
            rospy.loginfo(f"Obstacle detected within range threshold: {scan.range} meters")
        else:
            self.obstacle_detected = False
            rospy.loginfo(f"No obstacles detected within range threshold: {scan.range} meters")

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_publisher.publish(stop_msg)
        rospy.loginfo("Robot stopped due to obstacle detection")

    def move_with_obstacle_detection(self, velocity, distance):
        vel_msg = Twist()
        vel_msg.linear.x = velocity

        start_time = rospy.Time.now().to_sec()
        current_distance = 0
        initial_position = velocity

        while current_distance < distance:
            if self.obstacle_detected:
                self.stop_robot()
                rospy.loginfo("Movement interrupted by obstacle detection")
                
                # Wait for 3 seconds
                rospy.sleep(3)
                
                # Check if obstacle is still there
                if self.obstacle_detected:
                    rospy.loginfo("Obstacle still detected, returning to original position")
                    
                    # Move back to original position
                    vel_msg.linear.x = -velocity
                    while current_distance > 0:
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()
                        current_time = rospy.Time.now().to_sec()
                        current_distance -= (current_time - start_time) * abs(velocity)
                        rospy.loginfo(f"Returning to original position, current distance: {current_distance} meters")
                    
                    vel_msg.linear.x = 0
                    self.velocity_publisher.publish(vel_msg)  # Stop the robot
                    return
                
                else:
                    rospy.loginfo("Obstacle cleared, continuing movement")

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()
            current_distance = (current_time - start_time) * abs(velocity)
            rospy.loginfo(f"Current distance: {current_distance} meters")

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)  # Stop the robot

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
        self.velocity_publisher.publish(vel_msg)  # Stop the robot

    def execute(self):
        try:
            rospy.loginfo("Executing move with obstacle detection and turning")

            # Turn right 90 degrees (π/2 radians)
            rospy.loginfo("Turning right 90 degrees")
            self.turn_robot(0.5, 1.5708)  # Turn right with an angular velocity of 0.5 rad/s for π/2 radians

            # Move forward for 3 meters
            rospy.loginfo("Robot is moving forward for 3 meters")
            self.move_with_obstacle_detection(0.5, 3)  # Move forward with a velocity of 0.5 m/s for 3 meters

            # Wait for 3 seconds
            rospy.loginfo("Robot is waiting for 3 seconds")
            rospy.sleep(3)  # Wait for 3 seconds

            # Move backward for 3 meters
            rospy.loginfo("Robot is moving backward for 3 meters")
            self.move_with_obstacle_detection(-0.5, 3)  # Move backward with a velocity of -0.5 m/s for 3 meters
            
            # Turn back to the original orientation
            rospy.loginfo("Turning back to original orientation")
            self.turn_robot(-0.5, 1.5708)  # Turn left with an angular velocity of -0.5 rad/s for π/2 radians
            

            rospy.signal_shutdown("Movement complete, stopping the node.")
        
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    robot_mover = MoveRobot()
    rospy.spin()
