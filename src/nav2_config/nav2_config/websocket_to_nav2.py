import rclpy
from rclpy.node import Node
import asyncio
import json
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

import websockets


class WebSocketNav2Bridge(Node):

    def __init__(self):
        super().__init__('websocket_nav2_bridge')

        # WebSocket server
        self.server_url = "ws://rs44owcoo04408gk80goks8g.76.13.143.124.sslip.io"
        self.username = "admin"
        self.password = "123456"

        self.loop = asyncio.new_event_loop()
        self.websocket = None

        # NAV2 Action Client
        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # Run websocket in separate thread
        t = threading.Thread(target=self.start_ws_client, daemon=True)
        t.start()

        self.get_logger().info("Bridge Started (Direct X,Y → Nav2)")

    def start_ws_client(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_client())

    async def ws_client(self):
        while True:
            try:
                async with websockets.connect(self.server_url) as websocket:
                    self.websocket = websocket

                    # Auth
                    await websocket.send(json.dumps({
                        "type": "auth",
                        "data": {
                            "username": self.username,
                            "password": self.password
                        }
                    }))

                    auth_resp = await websocket.recv()
                    auth = json.loads(auth_resp)

                    if auth.get("type") != "auth_ok":
                        self.get_logger().error("Auth failed")
                        return

                    self.get_logger().info("Authenticated Successfully")

                    # Listen for messages
                    async for message in websocket:
                        data = json.loads(message)
                        msg_type = data.get("type")

                        if msg_type == "set_route":
                            route = data["data"]["route"]
                            self.handle_waypoints(route)

            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
                time.sleep(3)

    def handle_waypoints(self, route):
        self.get_logger().info(f"Received {len(route)} waypoints from GUI")

        poses = []

        for i, wp in enumerate(route):
            try:
                raw_x = float(wp["pose"]["position"]["x"])
                raw_y = float(wp["pose"]["position"]["y"])

                self.get_logger().info(
                    f"WP {i}: Direct Map Coordinates ({raw_x:.2f}, {raw_y:.2f})"
                )

                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()

                pose.pose.position.x = raw_x
                pose.pose.position.y = raw_y
                pose.pose.position.z = 0.0

                pose.pose.orientation.w = 1.0

                poses.append(pose)

            except KeyError as e:
                self.get_logger().error(f"Invalid waypoint format: missing {e}")

        if poses:
            self.send_to_nav2(poses)

    def send_to_nav2(self, poses):
        self.get_logger().info("Waiting for Nav2 Action Server...")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f"Sending {len(poses)} waypoints to Nav2...")

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted by Nav2")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Navigation finished successfully")


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketNav2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
