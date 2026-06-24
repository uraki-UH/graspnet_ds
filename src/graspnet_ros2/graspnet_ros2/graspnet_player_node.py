import os
import struct

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs_py.point_cloud2 as pc2


class GraspNetPlayerNode(Node):
    def __init__(self):
        super().__init__('graspnet_player')

        self.declare_parameter('root', '/data/graspnet')
        self.declare_parameter('scene_id', 0)
        self.declare_parameter('camera', 'realsense')
        self.declare_parameter('ann_id', 0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('point_step', 2)

        self.root = self.get_parameter('root').value
        self.scene_id = int(self.get_parameter('scene_id').value)
        self.camera = self.get_parameter('camera').value
        self.ann_id = int(self.get_parameter('ann_id').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.point_step = int(self.get_parameter('point_step').value)

        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/camera/color/image_raw',
            10,
        )
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            10,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera/color/camera_info',
            10,
        )
        self.points_pub = self.create_publisher(
            PointCloud2,
            '/camera/camera/depth/color/points',
            10,
        )

        period = 1.0 / max(self.publish_rate, 0.001)
        self.timer = self.create_timer(period, self.publish_frame)

        self.get_logger().info(
            'GraspNet player started: '
            f'root={self.root}, '
            f'scene_id={self.scene_id}, '
            f'camera={self.camera}, '
            f'ann_id={self.ann_id}, '
            f'point_step={self.point_step}'
        )

    def get_paths(self):
        base = os.path.join(
            self.root,
            'train1',
            f'scene_{self.scene_id:04d}',
            self.camera,
        )

        rgb_path = os.path.join(base, 'rgb', f'{self.ann_id:04d}.png')
        depth_path = os.path.join(base, 'depth', f'{self.ann_id:04d}.png')
        camk_path = os.path.join(base, 'camK.npy')

        return rgb_path, depth_path, camk_path

    def load_frame(self):
        rgb_path, depth_path, camk_path = self.get_paths()

        if not os.path.exists(rgb_path):
            raise FileNotFoundError(f'RGB file not found: {rgb_path}')
        if not os.path.exists(depth_path):
            raise FileNotFoundError(f'Depth file not found: {depth_path}')
        if not os.path.exists(camk_path):
            raise FileNotFoundError(f'camK file not found: {camk_path}')

        rgb_bgr = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        if rgb_bgr is None:
            raise RuntimeError(f'Failed to read RGB image: {rgb_path}')

        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)

        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            raise RuntimeError(f'Failed to read depth image: {depth_path}')

        if len(depth.shape) != 2:
            raise RuntimeError(f'Depth image must be single channel: {depth_path}')

        camK = np.load(camk_path)

        if camK.shape != (3, 3):
            raise RuntimeError(f'camK shape must be 3x3, got {camK.shape}')

        return rgb, depth, camK

    def make_camera_info(self, header, width, height, camK):
        msg = CameraInfo()
        msg.header = header
        msg.width = width
        msg.height = height

        fx = float(camK[0, 0])
        fy = float(camK[1, 1])
        cx = float(camK[0, 2])
        cy = float(camK[1, 2])

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return msg

    def make_pointcloud2(self, header, rgb, depth, camK):
        fx = float(camK[0, 0])
        fy = float(camK[1, 1])
        cx = float(camK[0, 2])
        cy = float(camK[1, 2])

        height, width = depth.shape
        points = []

        step = max(1, self.point_step)

        for v in range(0, height, step):
            for u in range(0, width, step):
                z_mm = int(depth[v, u])

                if z_mm == 0:
                    continue

                z = z_mm / 1000.0
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                r = int(rgb[v, u, 0])
                g = int(rgb[v, u, 1])
                b = int(rgb[v, u, 2])

                rgb_uint32 = (r << 16) | (g << 8) | b
                rgb_float = struct.unpack('f', struct.pack('I', rgb_uint32))[0]

                points.append((x, y, z, rgb_float))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        return pc2.create_cloud(header, fields, points)

    def publish_frame(self):
        try:
            rgb, depth, camK = self.load_frame()
        except Exception as e:
            self.get_logger().error(str(e))
            return

        now = self.get_clock().now().to_msg()

        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        height, width = depth.shape

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header = header

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header = header

        camera_info_msg = self.make_camera_info(header, width, height, camK)
        points_msg = self.make_pointcloud2(header, rgb, depth, camK)

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)
        self.points_pub.publish(points_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()