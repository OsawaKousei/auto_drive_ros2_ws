import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node


class PathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/path', 10)
        self.waypoints = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # 定期的にパスをパブリッシュするスレッド
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        self.get_logger().info('パスパブリッシャーを初期化しました')

    def update_path(self, waypoints: list) -> None:
        """
        経路点リストを更新し、パスメッセージを再構築します
        """
        self.waypoints = waypoints
        self.create_path_message()
        self.get_logger().info(f'パスを更新しました（経路点数: {len(waypoints)}）')

    def create_path_message(self) -> None:
        """
        経路点リストからPathメッセージを作成します
        waypoints: [(x, y, yaw), ...]
        """
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses = []

        for x, y, yaw in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Z座標は常に0

            # Yaw角からクォータニオンに変換
            q = self.euler_to_quaternion(0.0, 0.0, yaw)
            pose.pose.orientation = q

            self.path_msg.poses.append(pose)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        オイラー角（ロール、ピッチ、ヨー）をクォータニオンに変換
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q

    def publish_loop(self) -> None:
        """
        定期的にパスをパブリッシュするループ
        """
        while rclpy.ok():
            if len(self.waypoints) > 0:
                # タイムスタンプを更新
                self.path_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(self.path_msg)
            time.sleep(0.1)  # 10Hzでパブリッシュ

    def destroy_node(self) -> None:
        """
        ノードの終了時に呼ばれる
        """
        self.get_logger().info('パスパブリッシャーを終了します')
        super().destroy_node()
