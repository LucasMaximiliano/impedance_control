"""Integration tests for clamping the torque output of the impedance_controller ROS 2 node."""

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
CLAMP_THRESHOLD_NM = 10.0
TEST_TIMEOUT_SEC = 5.0
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

    def test_publishes_clamped_torque(self, proc_output):
        """Check whether torque messages get clamped."""
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.Float32MultiArray, OUTPUT_TOPIC,
            lambda msg: msgs_rx.append(msg), 100)
        srv = self.node.create_client(
                std_srvs.srv.SetBool, SERVICE_TOPIC)
        try:
            srv.wait_for_service(timeout_sec=TEST_TIMEOUT_SEC)
            # Request combined control enabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None
            assert future.result().success
            assert future.result().message == "Combined control enabled"
            # Listen to the torque topic for TEST_TIMEOUT_SEC seconds
            end_time = time.time() + TEST_TIMEOUT_SEC
            while time.time() < end_time and len(msgs_rx) < MIN_MSGS:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=SPIN_TIMEOUT_SEC)
            # All received torques should be within a safe range
            assert all(abs(t) <= CLAMP_THRESHOLD_NM for msg in msgs_rx for t in msg.data)
        finally:
            self.node.destroy_subscription(sub)