"""Integration tests for publishing torque output of the impedance_controller ROS 2 node."""

import os
import time
import unittest

import launch
import launch_ros
import launch_testing
import rclpy
import std_msgs.msg
import std_srvs.srv

SERVICE_TOPIC = "/impedance_controller/combined_control_enabled"
OUTPUT_TOPIC = "/command/set_torque_nm"
TEST_TIMEOUT_SEC = 10.0
SPIN_TIMEOUT_SEC = 0.05
MIN_MSGS = 5

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

    def test_publishes_torque(self, proc_output):
        """Check whether torque messages published."""
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.Float32MultiArray, OUTPUT_TOPIC,
            lambda msg: msgs_rx.append(msg), 100)
        srv = self.node.create_client(
                std_srvs.srv.SetBool, SERVICE_TOPIC)
        try:
            # Request combined control enabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Combined control enabled"
            # Listen to the torque topic for TEST_TIMEOUT_SEC seconds
            end_time = time.time() + TEST_TIMEOUT_SEC
            while time.time() < end_time and len(msgs_rx) < MIN_MSGS:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=SPIN_TIMEOUT_SEC)
            # There should have been at least MIN_MSGS messages received
            assert len(msgs_rx) >= MIN_MSGS
        finally:
            self.node.destroy_subscription(sub)