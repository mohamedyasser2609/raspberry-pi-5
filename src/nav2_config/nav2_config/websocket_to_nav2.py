import rclpy
from rclpy.node import Node
import asyncio
import json
import threading
import time
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

import websockets


class WebSocketNav2Bridge(Node):

    def __init__(self):
        super().__init__('websocket_nav2_bridge')

        self.server_url = "ws://rs44owcoo04408gk80goks8g.76.13.143.124.sslip.io"
        self.username = "admin"
        self.password = "123456"

        self.loop = asyncio.new_event_loop()
        self.websocket = None

        # ---------------- NAV2 ----------------
        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # GPS ORIGIN (أول نقطة تعتبر zero)
        self.origin_lat = None
        self.origin_lng = None

        t = threading.Thread(target=self.start_ws_client, daemon=True)
        t.start()

        self.get_logger().info("Bridge Started (GPS → MAP → Nav2)")

    # =====================================================
    def start_ws_client(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_client())

    # =====================================================
    def gps_to_local(self, lat, lng):

        # set origin first time
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lng = lng
            self.get_logger().info("GPS origin set")

        # simple flat earth approx
        dx = (lng - self.origin_lng) * 111000 * math.cos(math.radians(lat))
        dy = (lat - self.origin_lat) * 111000

        return dx, dy

    # =====================================================
    async def ws_client(self):

        while True:
            try:
                async with websockets.connect(self.server_url) as websocket:
                    self.websocket = websocket

                    # AUTH
                    await websocket.send(json.dumps({
                        "type": "auth",
                        "data": {
                            "username": self.username,
                            "password": self.password
                        }
                    }))

                    auth = json.loads(await websocket.recv())

                    if auth.get("type") != "auth_ok":
                        self.get_logger().error("Auth failed")
                        return

                    self.get_logger().info("Authenticated")

                    # LOOP
                    async for message in websocket:

                        data = json.loads(message)
                        msg_type = data.get("type")

                        # =================================================
                        # ROUTE → NAV2 (GPS CONVERTED)
                        # =================================================
                        if msg_type == "set_route":

                            route = data["data"]["route"]

                            self.handle_waypoints(route)

                        # =================================================
                        # STATE (optional forward)
                        # =================================================
                        elif msg_type == "state":
                            pass

            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
                time.sleep(3)

    # =====================================================
    def handle_waypoints(self, route):

        self.get_logger().info(f"Route received: {len(route)} points")

        poses = []

        for i, wp in enumerate(route):

            lat = float(wp["pose"]["position"]["x"])
            lng = float(wp["pose"]["position"]["y"])

            # GPS → LOCAL MAP
            x, y = self.gps_to_local(lat, lng)

            self.get_logger().info(f"WP {i}: GPS({lat},{lng}) → MAP({x:.2f},{y:.2f})")

            pose = PoseStamped()
            pose.header.frame_id = "map"

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            poses.append(pose)

        self.send_to_nav2(poses)

    # =====================================================
    def send_to_nav2(self, poses):

        self.get_logger().info("Waiting for Nav2...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 not available")
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info(f"Sending {len(poses)} poses to Nav2")

        self._action_client.send_goal_async(goal)


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = WebSocketNav2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
