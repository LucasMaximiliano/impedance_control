"""Integration tests for (de-)spawing of impedance_controller ROS 2 node."""

import os
import time
import unittest

import launch
import launch_ros
import launch_testing
import rclpy
import std_msgs.msg


def generate_test_description():
    """Generate the launch description for integration tests."""
    impedance_controller_node = launch_ros.actions.Node(
        package='impedance_controller',
        namespace='',
        executable='impedance_controller_node',
        name='impedance_controller',
    )
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                impedance_controller_node,
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {"impedance_controller_node": impedance_controller_node},
    )

class TestImpedanceController(unittest.TestCase):
    """Active integration tests while the node is running."""
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_impedance_controller')

    def tearDown(self):
        self.node.destroy_node()

    def test_logs_spawning(self, proc_output):
        """Check whether logging properly."""
        proc_output.assertWaitFor(
            'Constructing... Combined Controller Node',
            timeout=5, stream='stderr')

@launch_testing.post_shutdown_test()
class TestImpedanceControllerShutdown(unittest.TestCase):
    """Post-shutdown integration tests."""
    def test_logs_despawning(self, proc_output, impedance_controller_node):
        """Check whether logging properly."""
        launch_testing.asserts.assertInStderr(
            proc_output,
            'Destructing... Combined Controller Node',
            process=impedance_controller_node
        )

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)