"""dummy_planner_node.py

This script implements a dummy planner for testing of the impedance controller. It also
spins up a ROS2 node containing the dummy planner.

The dummy planner generates a trajectory with a trapezoidal velocity profile from the
measured current configuration of the finger to the desired final configuration. The trajectory
is sampled at the control loop frequency and published as position and velocity references to
the impedance controller, as long as the planner is enabled.

IMPORTANT: Make sure that the constants defined below are consistent with the parameters defined
in the impedance controller, otherwise the trajectory generation will not work as expected!
"""

import rclpy
import numpy as np
from roboticstoolbox import trapezoidal
from roboticstoolbox import mtraj
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from impedance_controller_interfaces.srv import SetFloat64MultiArray

# Constants (please verify these values before testing)
CONTROL_LOOP_DURATION_MS = 10
MAX_JOINT_VELOCITY_RAD_PER_SEC = 2.0
MAX_JOINT_ACCELERATION_RAD_PER_SEC_SQ = 0.4
QUEUE_SIZE = 10
SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC = {
    "name": '/senso_joint/position_rad',
    "msg_type": Float64MultiArray
}
SET_DESIRED_FINAL_POSITION_SERVICE    = {
    "name": '/impedance_controller/set_final_position_rad',
    "msg_type": SetFloat64MultiArray
}
SET_PLANNER_ENABLED_SERVICE = {
    "name": '/impedance_controller/set_planner_enabled',
    "msg_type": SetBool
}
PUBLISH_DESIRED_POSITION_TOPIC = {
    "name": '/impedance_controller/set_position_rad',
    "msg_type": Float64MultiArray
}
PUBLISH_DESIRED_VELOCITY_TOPIC = {
    "name": '/impedance_controller/set_velocity_rad_per_sec',
    "msg_type": Float32MultiArray
}

class DummyPlannerNode(Node):
    """
    This class implements a dummy planner for testing of the impedance controller.

    The dummy planner generates a trajectory with a trapezoidal velocity profile. It
    also has a ROS2 interface to interface with the rest of the system.
    
    Attributes
    ----------
    planner_enabled_ : bool
        a flag indicating whether the planner is enabled or not.
    trajectory_ready_ : bool
        a flag indicating whether the trajectory has been calculated and is ready for execution.
    trajectory_index_ : int
        an integer index for iterating through the trajectory during execution.
    timer_ : rclpy.timer.Timer
        a ROS2 timer for executing the control loop at the specified frequency.
    measured_initial_position_subscription_ : rclpy.subscription.Subscription
        a ROS2 subscription for receiving the measured initial position from the encoders.
    desired_final_position_service_ : rclpy.service.Service
        a ROS2 service for receiving the desired final position and calculating the trajectory.
    planner_enabled_service_ : rclpy.service.Service
        a ROS2 service for enabling or disabling the planner.
    desired_position_publisher_ : rclpy.publisher.Publisher
        a ROS2 publisher for publishing the desired position reference at the control loop frequency.
    desired_velocity_publisher_ : rclpy.publisher.Publisher
        a ROS2 publisher for publishing the desired velocity reference at the control loop frequency.
    measured_initial_position_ : numpy.ndarray
        a list storing the measured initial position from the encoders.
    desired_final_position_ : numpy.ndarray
        a list storing the desired final position.
    trajectory_ : numpy.ndarray
        a list storing the calculated trajectory.

    Methods
    -------
    __init__(self)
        Initializes the ROS2 node, sets up subscribers, publishers, and services, and initializes variables.
    measured_initial_position_callback(self, msg)
        Callback function for receiving and storing the measured initial position from the encoders.
    desired_final_position_callback(self, request, response)
        Callback function for receiving the desired final position, calculating the trajectory, and storing
        it for execution.
    planner_enabled_callback(self, request, response)
        Callback function for enabling or disabling the planner.
    timer_callback(self)
        Callback function for publishing the desired position and velocity references at the control loop
        frequency when the planner is enabled and the trajectory has already been calculated.
    """
    def __init__(self):
        """Initializes the ROS2 node, sets up subscribers, publishers, and services, and initializes
        variables."""
        super().__init__('dummy_planner_node')
        self.get_logger().info('Dummy Planner Node has been started.')
        # Variables
        self.planner_enabled_ = False
        self.trajectory_ready_ = False
        self.trajectory_index_ = 0
        # Timer
        self.timer_ = self.create_timer(CONTROL_LOOP_DURATION_MS / 1000.0, self.timer_callback)
        # Subscriber
        self.measured_initial_position_subscription_ = self.create_subscription(
            SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC["msg_type"],
            SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC["name"],
            self.measured_initial_position_callback,
            QUEUE_SIZE
        )
        # Services
        self.desired_final_position_service_ = self.create_service(
            SET_DESIRED_FINAL_POSITION_SERVICE["msg_type"],
            SET_DESIRED_FINAL_POSITION_SERVICE["name"],
            self.desired_final_position_callback
        )
        self.planner_enabled_service_ = self.create_service(
            SET_PLANNER_ENABLED_SERVICE["msg_type"],
            SET_PLANNER_ENABLED_SERVICE["name"],
            self.planner_enabled_callback
        )
        # Publishers
        self.desired_position_publisher_ = self.create_publisher(
            PUBLISH_DESIRED_POSITION_TOPIC["msg_type"],
            PUBLISH_DESIRED_POSITION_TOPIC["name"],
            QUEUE_SIZE
        )
        self.desired_velocity_publisher_ = self.create_publisher(
            PUBLISH_DESIRED_VELOCITY_TOPIC["msg_type"],
            PUBLISH_DESIRED_VELOCITY_TOPIC["name"],
            QUEUE_SIZE
        )

    def measured_initial_position_callback(self, msg):
        """Callback function for receiving and storing the measured initial position from the encoders.
        
        Turns the received message from `SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC` into a numpy array
        and stores it in the class variable `measured_initial_position_`.

        Parameters
        ----------
        msg : std_msgs.msg.Float64MultiArray
            the message containing the measured initial position from the encoders.
        """
        self.measured_initial_position_ = np.array(msg.data)

    def desired_final_position_callback(self, request, response):
        """Callback function for receiving the desired final position, calculating the trajectory, and
        storing it for execution.

        Turns the received desired final position from the `SET_DESIRED_FINAL_POSITION_SERVICE` request into
        a numpy array and stores it in the class variable `desired_final_position_`. If the measured initial
        position has already been received, it calculates the trajectory with a trapezoidal velocity profile
        from the measured initial position to the desired final position, samples the trajectory at the
        control loop frequency, and stores it in the class variable `trajectory_`.

        Parameters
        ----------
        request : impedance_controller_interfaces.srv.SetFloat64.Request
            the request containing the desired final position.
        response : impedance_controller_interfaces.srv.SetFloat64.Response
            the response to be sent back after processing the request.

        Returns
        -------
        impedance_controller_interfaces.srv.SetFloat64.Response
            the response containing a success flag and a message indicating the result of the operation.
        """
        self.desired_final_position_ = np.array(request.data)
        if hasattr(self, 'measured_initial_position_'):
            self.get_logger().info('Received desired final position: ' + str(self.desired_final_position_) +
                ' rad. Calculating trajectory from measured initial position: ' + str(self.measured_initial_position_) +
                ' rad.')
            # Generate trajectory with trapezoidal velocity profile (sampled at the control loop frequency)
            trajectory_duration_sec = ( (MAX_JOINT_ACCELERATION_RAD_PER_SEC_SQ + MAX_JOINT_VELOCITY_RAD_PER_SEC**2) /
                (MAX_JOINT_ACCELERATION_RAD_PER_SEC_SQ * MAX_JOINT_VELOCITY_RAD_PER_SEC) )
            time_array = np.arange(0, trajectory_duration_sec, CONTROL_LOOP_DURATION_MS / 1000.0)
            self.trajectory_ = mtraj(trapezoidal, self.measured_initial_position_, self.desired_final_position_, time_array)
            self.trajectory_ready_ = True
            # Write response
            response.message = 'Trajectory calculated successfully.'
            response.success = True
            self.get_logger().info(response.message)
        else:
            # Write response
            response.message = 'Warning: Measured initial position not received.'
            response.success = False
            self.get_logger().warn(response.message)
        return response

    def planner_enabled_callback(self, request, response):
        """Callback function for enabling or disabling the planner.

        Turns the received boolean from `SET_PLANNER_ENABLED_SERVICE` into a flag
        for enabling or disabling the planner and stores it in the class variable
        `planner_enabled_`.

        Parameters
        ----------
        request : std_srvs.srv.SetBool.Request
            the request containing the boolean for enabling or disabling the
            planner.
        response : std_srvs.srv.SetBool.Response
            the response to be sent back after processing the request.

        Returns
        -------
        std_srvs.srv.SetBool.Response
            the response containing a success flag and a message indicating the
            result of the operation.
        """
        if request.data:
            response.message = 'Planner enabled.'
            self.planner_enabled_ = True
        else:
            response.message = 'Planner disabled.'
            self.planner_enabled_ = False
        self.get_logger().info(response.message)
        response.success = True
        return response

    def timer_callback(self):
        """Callback function for publishing the desired position and velocity
        references to the impedance controller.

        If the planner is enabled and the trajectory has already been calculated,
        it formats the current position and velocity references from the trajectory
        into ROS2 messages and publishes them to `PUBLISH_DESIRED_POSITION_TOPIC` and
        `PUBLISH_DESIRED_VELOCITY_TOPIC` at the control loop frequency. It also iterates
        through the trajectory by increasing the index for the next references. If the
        end of the trajectory is reached, it logs that the trajectory execution is
        completed, disables the planner, resets the trajectory ready flag, and resets
        the trajectory index for the next execution.
        """
        if self.planner_enabled_ and self.trajectory_ready_:
            desired_position_msg = Float64MultiArray()
            desired_velocity_msg = Float32MultiArray()
            # Format ROS messages
            desired_position_msg.data = self.trajectory_.q[self.trajectory_index_, :].tolist()
            desired_velocity_msg.data = self.trajectory_.qd[self.trajectory_index_, :].tolist()
            # Publish ROS messages and increment trajectory index for next run
            self.desired_position_publisher_.publish(desired_position_msg)
            self.desired_velocity_publisher_.publish(desired_velocity_msg)
            self.trajectory_index_ += 1
            # Check if end of trajectory is reached and reset variables if so
            if self.trajectory_index_ >= self.trajectory_.__len__():
                self.get_logger().info('Trajectory execution completed.')
                self.planner_enabled_ = False
                self.trajectory_ready_ = False
                self.trajectory_index_ = 0

def main():
    rclpy.init()
    node = DummyPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
