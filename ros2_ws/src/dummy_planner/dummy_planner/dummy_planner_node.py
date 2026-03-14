import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

# Topic names and queue size
QUEUE_SIZE = 10

SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC = {
    "name": 'senso_joint/position_rad',
    "msg_type": Float64MultiArray
}

SUBSCRIBE_DESIRED_FINAL_POSITION_TOPIC    = {
    "name": 'impedance_controller/set_final_position_rad',
    "msg_type": Float64MultiArray
}

PUBLISH_DESIRED_POSITION_TOPIC = {
    "name": 'impedance_controller/set_position_rad',
    "msg_type": Float64MultiArray
}

PUBLISH_DESIRED_VELOCITY_TOPIC = {
    "name": 'impedance_controller/set_velocity_rad_per_sec',
    "msg_type": Float32MultiArray
}

class DummyPlannerNode(Node):
    def __init__(self):
        super().__init__('dummy_planner_node')
        self.get_logger().info('Dummy Planner Node has been started.')
        # Subscribers
        self.measured_initial_position_subscription_ = self.create_subscription(
            SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC["msg_type"],
            SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC["name"],
            self.measured_initial_position_callback,
            QUEUE_SIZE
        )
        self.desired_final_position_subscription_ = self.create_subscription(
            SUBSCRIBE_DESIRED_FINAL_POSITION_TOPIC["msg_type"],
            SUBSCRIBE_DESIRED_FINAL_POSITION_TOPIC["name"],
            self.desired_final_position_callback,
            QUEUE_SIZE
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
        # Process the measured initial position
        pass

    def desired_final_position_callback(self, msg):
        # Process the desired final position
        pass

def main():
    print('Hi from dummy_planner.')
    rclpy.init()
    node = DummyPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
