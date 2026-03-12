"""Integration tests for dynamic parameter reconfiguration of the impedance_controller ROS 2 node using services."""

import os
import time
import unittest

import launch
import launch_ros
import launch_testing
import rclpy
import std_srvs.srv
import impedance_controller_interfaces.srv

SERVICE_TOPIC_1 = "/impedance_controller/gravity_compensation_enabled"
SERVICE_TOPIC_2 = "/impedance_controller/impedance_control_enabled"
SERVICE_TOPIC_3 = "/impedance_controller/set_inertia_gain"
SERVICE_TOPIC_4 = "/impedance_controller/set_damping_gain"
SERVICE_TOPIC_5 = "/impedance_controller/set_stiffness_gain"
SERVICE_TOPIC_6 = "/impedance_controller/set_torque_gain"
TEST_TIMEOUT_SEC = 2.0

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

    def test_logs_gravity_enable_update(self, proc_output):
        """Check whether gravity compensation enabled is correctly set."""
        try:
            srv = self.node.create_client(
                std_srvs.srv.SetBool, SERVICE_TOPIC_1)
            # request gravity compensation enabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Gravity compensation enabled"
            # request gravity compensation disabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=False))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Gravity compensation disabled"
        finally:
            self.node.destroy_client(srv)
    
    def test_logs_impedance_enable_update(self, proc_output):
        """Check whether impedance controller enabled is correctly set."""
        try:
            srv = self.node.create_client(
                std_srvs.srv.SetBool, SERVICE_TOPIC_2)
            # request impedance control enabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Impedance control enabled"
            # request impedance control disabled
            future = srv.call_async(std_srvs.srv.SetBool.Request(data=False))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Impedance control disabled"
        finally:
            self.node.destroy_client(srv)

    def test_logs_inertia_matrix_update(self, proc_output):
        """Check whether inertia gain is correctly set."""
        try:
            srv = self.node.create_client(
                impedance_controller_interfaces.srv.SetGain, SERVICE_TOPIC_3)
            # request inertia gain update
            future = srv.call_async(impedance_controller_interfaces.srv.SetGain.Request(gain=5.0))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Inertia gain updated to 5.000000"
        finally:
            self.node.destroy_client(srv)

    def test_logs_damping_matrix_update(self, proc_output):
        """Check whether damping gain is correctly set."""
        try:
            srv = self.node.create_client(
                impedance_controller_interfaces.srv.SetGain, SERVICE_TOPIC_4)
            # request damping gain update
            future = srv.call_async(impedance_controller_interfaces.srv.SetGain.Request(gain=5.0))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Damping gain updated to 5.000000"
        finally:
            self.node.destroy_client(srv)

    def test_logs_stiffness_matrix_update(self, proc_output):
        """Check whether stiffness gain is correctly set."""
        try:
            srv = self.node.create_client(
                impedance_controller_interfaces.srv.SetGain, SERVICE_TOPIC_5)
            # request stiffness gain update
            future = srv.call_async(impedance_controller_interfaces.srv.SetGain.Request(gain=5.0))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Stiffness gain updated to 5.000000"
        finally:
            self.node.destroy_client(srv)

    def test_logs_torque_gain_update(self, proc_output):
        """Check whether torque gain is correctly set."""
        try:
            srv = self.node.create_client(
                impedance_controller_interfaces.srv.SetGain, SERVICE_TOPIC_6)
            # request torque gain update
            future = srv.call_async(impedance_controller_interfaces.srv.SetGain.Request(gain=0.5))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TEST_TIMEOUT_SEC)
            assert future.result() is not None and future.result().success and future.result().message == "Torque gain updated to 0.500000"
        finally:
            self.node.destroy_client(srv)