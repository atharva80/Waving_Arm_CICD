import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class ArmWavingNode(Node):
    def __init__(self):
        super().__init__('arm_waving_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.get_logger().info('Arm Waving Node Started. Publishing to /joint_states.')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_to_arm_1']

        angle = 1.5 * math.sin(time.time() * 0.5)
        msg.position = [angle]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmWavingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()