import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

# Topic names and queue size
QUEUE_SIZE = 10

SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC = '/senso_joint/position_rad'
SUBSCRIBE_DESIRED_FINAL_POSITION_TOPIC    = 'blablabla' # TODO: Check with @Zara how to interface w/GUI

class DummyPlannerNode(Node):
    def __init__(self):
        super().__init__('dummy_planner_node')
        self.get_logger().info('Dummy Planner Node has been started.')
        # Subscribers
        self.measured_initial_position_subscription_ = self.create_subscription(
            Float64MultiArray,
            SUBSCRIBE_MEASURED_INITIAL_POSITION_TOPIC,
            self.measured_initial_position_callback,
            QUEUE_SIZE
        )
        self.desired_final_position_subscription_ = self.create_subscription(
            Float64MultiArray,
            SUBSCRIBE_DESIRED_FINAL_POSITION_TOPIC,
            self.desired_final_position_callback,
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


if __name__ == '__main__':
    main()
