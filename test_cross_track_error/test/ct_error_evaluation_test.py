#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import unittest

class CrossTrackErrorTest(unittest.TestCase):
    def setUp(self):
        # Initialize the ROS node for testing
        rospy.init_node('cross_track_error_test', anonymous=True)

        # Define acceptable error threshold and test duration from parameters
        self.error_threshold = rospy.get_param("~error_threshold", 1.0)
        self.test_duration = rospy.get_param("~duration", 30)
        
        self.ct_errors = []  # Store cross-track errors

        # Subscribe to cross-track error topic
        rospy.Subscriber('/pure_pursuit/cross_track_error', Float32, self.ct_error_callback)

    def ct_error_callback(self, msg):
        if rospy.is_shutdown():
            return  # Skip processing if shutdown is in progress
        # Append the received error value to the list
        self.ct_errors.append(abs(msg.data))

    def test_cross_track_error_within_threshold(self):
        # Wait for the test duration
        rospy.sleep(self.test_duration)

        # Log and evaluate cross-track errors
        rospy.loginfo("Evaluating cross-track errors")
        all_within_threshold = all(error < self.error_threshold for error in self.ct_errors)

        # Log the result and use assertions to indicate pass/fail
        if all_within_threshold:
            rospy.loginfo("Test Passed: All cross-track errors were within ±%.1f meters" % self.error_threshold)
        else:
            rospy.loginfo("Test Failed: Some cross-track errors exceeded ±%.1f meters" % self.error_threshold)
        
        # Assert the condition for `unittest` to recognize pass/fail
        self.assertTrue(all_within_threshold, "Some cross-track errors exceeded the threshold")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_cross_track_error', 'cross_track_error_test', CrossTrackErrorTest)