import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PlanterNode(Node):
    def __init__(self):
        super().__init__('nova_plant_movement')

        self.pub = self.create_publisher(
            Float64MultiArray,
            'planar_goal',
            10
        )

        # order of poses
        self.poses = [
            # 0: Rest pose, gripper closed
            [0.0, 0.0, 0.30, 0.0, 0.0],

            # 1: Above seed position
            [0.2, 0.0, 0.18, 0.0, 0.0],

            # 2: Down to seed position
            [0.2, 0.0, 0.10, 0.0, 0.0],

            # 3: Open gripper at seed position
            [0.2, 0.0, 0.10, 0.0, 1.0],

            # 4: Lift seed
            [0.2, 0.0, 0.20, 0.0, 1.0],

            # 5: Back to rest
            [0.0, 0.0, 0.30, 0.0, 1.0],

            # 6: Rest pose
            [0.0, 0.0, 0.30, 0.0, 1.0],
        ]

        self.index = 0
        self.step_time = 1.0
        self.timer = self.create_timer(self.step_time, self.timer_cb)

        self.get_logger().info('Planter node started, looping seeding routine.')

    def timer_cb(self):
        pose = self.poses[self.index]
        
        msg = Float64MultiArray()
        msg.data = pose
        self.pub.publish(msg)

        self.get_logger().info(
            f"Step {self.index}: x={pose[0]:.3f}, y={pose[1]:.3f}, "
            f"z={pose[2]:.3f}, phi={pose[3]:.2f}, g={pose[4]:.2f}"
        )

        # loop
        self.index = (self.index + 1) % len(self.poses)

def main(args=None):
    rclpy.init(args=args)
    node = PlanterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()