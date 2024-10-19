import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
import math


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.iterations_left = 10  # Number of times to repeat the route
        self.waypoints = []  # Store waypoints

    def send_goal(self):
        if self.iterations_left > 0:
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = self.waypoints

            self._action_client.wait_for_server()

            # Send the goal
            self.get_logger().info(f'Sending waypoints, {self.iterations_left} iterations left...')
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback}')  # Print feedback to inspect its structure

        # If the robot reaches the final waypoint, decrement the iteration count and repeat
        if feedback.feedback.number_of_poses_remaining == 0:
            self.iterations_left -= 1
            if self.iterations_left > 0:
                self.get_logger().info(f'Repeating route, {self.iterations_left} iterations left...')
                self.send_goal()
            else:
                self.get_logger().info('Finished all iterations.')

    def create_waypoint(self, x, y, theta):
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.orientation.z = math.sin(theta / 2.0)
        waypoint.pose.orientation.w = math.cos(theta / 2.0)
        return waypoint


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    # Define a list of waypoints (x, y, theta in radians)
    node.waypoints = [
        node.create_waypoint(1.0, 1.0, 0.0),
        node.create_waypoint(2.0, 2.0, math.pi / 2),
        node.create_waypoint(3.0, 3.0, math.pi),
    ]

    # Send the goal for the first time
    node.send_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
