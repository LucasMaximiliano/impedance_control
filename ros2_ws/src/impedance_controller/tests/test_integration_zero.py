"""Integration tests for zeroing out the torque output given NaN input of the impedance_controller ROS 2 node."""

import time
import unittest

import launch
import launch_ros
import launch_testing
import rclpy
import std_msgs.msg
import std_srvs.srv

OUTPUT_TOPIC = "/command/set_torque_nm"
INPUT_TOPIC = "/senso_joint/torque_nm"
SERVICE_TOPIC_0 = "/impedance_controller/combined_control_enabled"
SERVICE_TOPIC_1 = "/impedance_controller/gravity_compensation_enabled"
SERVICE_TOPIC_2 = "/impedance_controller/impedance_control_enabled"
CLAMP_THRESHOLD_NM = 30.0
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

    def test_publishes_zero_torque(self, proc_output):
        """Check whether torque messages get zeroed out."""
        msgs_rx = []
        pub = self.node.create_publisher(
            std_msgs.msg.Float32MultiArray, INPUT_TOPIC, 100)
        srv0 = self.node.create_client(
            std_srvs.srv.SetBool, SERVICE_TOPIC_0)
        srv1 = self.node.create_client(
            std_srvs.srv.SetBool, SERVICE_TOPIC_1)
        srv2 = self.node.create_client(
            std_srvs.srv.SetBool, SERVICE_TOPIC_2)
        try:
            srv0.wait_for_service(timeout_sec=TEST_TIMEOUT_SEC)
            srv1.wait_for_service(timeout_sec=TEST_TIMEOUT_SEC)
            srv2.wait_for_service(timeout_sec=TEST_TIMEOUT_SEC)
            # request combined control enabled
            future = srv0.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None
            assert future.result().success
            # request gravity compensation disabled
            future = srv1.call_async(std_srvs.srv.SetBool.Request(data=False))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None
            assert future.result().success
            # request impedance control enabled
            future = srv2.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None
            assert future.result().success
            sub = self.node.create_subscription(
                std_msgs.msg.Float32MultiArray, OUTPUT_TOPIC,
                lambda msg: msgs_rx.append(msg), 100)
            # Listen to the torque topic for TEST_TIMEOUT_SEC seconds while publishing NaN torques
            end_time = time.time() + TEST_TIMEOUT_SEC
            while time.time() < end_time and len(msgs_rx) < MIN_MSGS:
                # publish stimulus for NaN torques
                pub.publish(std_msgs.msg.Float32MultiArray(data=[float('nan')]*8))
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=SPIN_TIMEOUT_SEC)
            # All received torques should be zero
            assert all(t == 0 for msg in msgs_rx for t in msg.data)
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
            self.node.destroy_client(srv0)
            self.node.destroy_client(srv1)
            self.node.destroy_client(srv2)