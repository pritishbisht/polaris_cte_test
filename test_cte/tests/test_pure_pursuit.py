#!/usr/bin/env python3

import rospy
import unittest
import rostest
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import GetModelState
import time

class PurePursuitTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_pure_pursuit_node', anonymous=True)
        self.ackermann_cmd_received = False
        self.cross_track_errors = []
        self.ackermann_sub = rospy.Subscriber('/gem/ackermann_cmd', AckermannDrive, self.ackermann_callback)

        # Wait for Gazebo service to be ready
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def ackermann_callback(self, msg):
        # Confirm receipt of Ackermann commands
        self.ackermann_cmd_received = True
        self.received_steering_angle = msg.steering_angle

    def test_ackermann_commands(self):
        """Test if the node publishes AckermannDrive commands"""
        # Give some time for commands to be published
        timeout = time.time() + 5.0  # 5 seconds timeout
        while not self.ackermann_cmd_received and time.time() < timeout:
            rospy.sleep(0.1)
        self.assertTrue(self.ackermann_cmd_received, "No AckermannDrive commands were received")

    def test_cross_track_error(self):
        """Test if the cross-track error is within acceptable bounds."""
        start_time = time.time()
        duration = rospy.get_param('~duration', 10)  # Duration as set in launch file

        while time.time() - start_time < duration:
            model_state = self.get_model_state(model_name='gem')
            x, y = model_state.pose.position.x, model_state.pose.position.y
            yaw = model_state.pose.orientation.z
            # Calculate cross-track error based on current pose vs. path (simplified)
            expected_y = x * 0.5  # Simplified expected y, adjust as necessary
            error = abs(y - expected_y)
            self.cross_track_errors.append(error)
            rospy.sleep(0.1)

        # Check if cross-track errors were within acceptable bounds
        max_error = max(self.cross_track_errors)
        self.assertLessEqual(max_error, 1.0, "Cross-track error exceeded 1.0 meters")

if __name__ == '__main__':
    rostest.rosrun('test_cte', 'test_pure_pursuit', PurePursuitTest)

