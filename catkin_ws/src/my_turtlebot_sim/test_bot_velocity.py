#!/usr/bin/env python

import unittest
import rospy
from gazebo_msgs.msg import ModelStates

class TestBotVelocity(unittest.TestCase):
    """
    Test case to verify the velocity of a simulated robot.

    The robot's velocity is measured using the /gazebo/model_states topic.
    The test checks whether the measured velocity matches the expected value.
    """

    def test_velocity(self):
        """
        Test the robot's velocity.

        This function initializes a ROS node, subscribes to the model states topic,
        waits for data, and checks if the measured velocity matches the expected value.
        """
        rospy.init_node('test_velocity', log_level=rospy.DEBUG)

        # Subscribe to the model states topic
        self.robot_velocity = None
        self.subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.velocity_callback)

        # Wait for data (adjust timeout as needed)
        rospy.sleep(5)

        # Check if the measured velocity matches the expected value
        expected_velocity = 0.9  # Meters per second (m/s)
        self.assertAlmostEqual(self.robot_velocity, expected_velocity, delta=0.1)

    def velocity_callback(self, data):
        """
        Callback function to record the robot's velocity from model states.

        Args:
            data (ModelStates): The model states message containing robot information.
        """
        # Assuming 'turtlebot3_waffle' is the robot we're interested in
        try:
            idx = data.name.index('turtlebot3_waffle')
            self.robot_velocity = data.twist[idx].linear.x
        except ValueError:
            rospy.logwarn("Robot 'turtlebot3_waffle' not found in Gazebo.")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_turtlebot_sim', 'test_bot_velocity', TestBotVelocity)
