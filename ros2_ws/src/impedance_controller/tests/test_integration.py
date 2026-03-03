import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
import std_msgs.msg

def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='impedance_controller',
                    namespace='',
                    executable='impedance_controller_node',
                    name='impedance_controller',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )

# Active tests i.e. those performed while the nodes under test are running
class TestImpedanceController(unittest.TestCase):
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

    def test_publishes_clamped_torque(self, proc_output):
        """Check whether torque messages get clamped"""
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.Float32MultiArray, '/command/set_torque_nm',
            lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the torque topic for 10 s
            end_time = time.time() + 10
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # All received torques should be within a safe range
            # TODO: Update the range below based on the actual limits of the system
            assert all(abs(t) <= 120 for msg in msgs_rx for t in msg.data)
        finally:
            self.node.destroy_subscription(sub)

    def test_publishes_zero_torque(self, proc_output):
        """Check whether torque messages get zeroed out"""
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.Float32MultiArray, '/command/set_torque_nm',
            lambda msg: msgs_rx.append(msg), 100)
        pub = self.node.create_publisher(
            std_msgs.msg.Float32MultiArray, '/torque_nm', 10)
        try:
            # Listen to the torque topic for 10 s
            end_time = time.time() + 10
            while time.time() < end_time:
                # publish stimulus for NaN torques
                pub.publish(std_msgs.msg.Float32MultiArray(data=[float('nan')]*8))
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # All received torques should be zero
            assert all(t == 0 for msg in msgs_rx for t in msg.data)
        finally:
            self.node.destroy_subscription(sub)

    def test_publishes_torque(self, proc_output):
        """Check whether torque messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.Float32MultiArray, '/command/set_torque_nm',
            lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the torque topic for 10 s
            end_time = time.time() + 10
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 100 messages received
            assert len(msgs_rx) > 100
        finally:
            self.node.destroy_subscription(sub)

    def test_logs_spawning(self, proc_output):
        """Check whether logging properly"""
        proc_output.assertWaitFor(
            'Constructing... Impedance Controller Node',
            timeout=5, stream='stderr')

    def test_logs_despawning(self, proc_output):
        """Check whether logging properly"""
        proc_output.assertWaitFor(
            'Destructing... Impedance Controller Node',
            timeout=5, stream='stderr')

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestImpedanceControllerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)