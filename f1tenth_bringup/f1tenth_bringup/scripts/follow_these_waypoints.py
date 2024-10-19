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

    def send_goal(self, waypoints):
        goal_msg = NavigateThroughPoses.Goal()

        # Add the list of waypoints to the goal message
        goal_msg.poses = waypoints

        self._action_client.wait_for_server()

        # Send the goal
        self.get_logger().info('Sending waypoints...')
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback}')  # Print entire feedback to inspect its structure


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
    waypoints = [
        node.create_waypoint(1.0, 1.0, 0.0),
        node.create_waypoint(2.0, 2.0, math.pi / 2),
        node.create_waypoint(3.0, 3.0, math.pi),
    ]

    node.send_goal(waypoints)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
