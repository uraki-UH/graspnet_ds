import os
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


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class GraspNetPlayerNode(Node):
    def __init__(self):
        super().__init__('graspnet_player')

        self.declare_parameter('root', '/data/graspnet')
        self.declare_parameter('split', 'train_1')
        self.declare_parameter('scene_id', 0)
        self.declare_parameter('camera', 'realsense')
        self.declare_parameter('ann_id', 0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pointcloud_frame_id', 'graspnet_table')
        self.declare_parameter('use_camera_pose', True)
        self.declare_parameter('invert_camera_pose', False)
        self.declare_parameter('use_table_frame', True)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('point_step', 2)
        self.declare_parameter('loop', True)

        self.root = self.get_parameter('root').value
        self.split = self.get_parameter('split').value
        self.scene_id = int(self.get_parameter('scene_id').value)
        self.camera = self.get_parameter('camera').value
        self.ann_id = int(self.get_parameter('ann_id').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.pointcloud_frame_id = self.get_parameter('pointcloud_frame_id').value
        self.use_camera_pose = as_bool(self.get_parameter('use_camera_pose').value)
        self.invert_camera_pose = as_bool(self.get_parameter('invert_camera_pose').value)
        self.use_table_frame = as_bool(self.get_parameter('use_table_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.point_step = int(self.get_parameter('point_step').value)
        self.loop = as_bool(self.get_parameter('loop').value)

        self.bridge = CvBridge()
        self.camK = None
        self.camera_poses = None
        self.cam0_wrt_table = None
        self.frame_ids = self.collect_frame_ids()
        if not self.frame_ids:
            raise FileNotFoundError(
                f'No frames found for scene {self.scene_id:04d} under {self.root}'
            )

        if self.ann_id in self.frame_ids:
            self.current_index = self.frame_ids.index(self.ann_id)
        else:
            self.current_index = 0

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
            f'split={self.split}, '
            f'scene_id={self.scene_id}, '
            f'camera={self.camera}, '
            f'ann_id={self.ann_id}, '
            f'frame_count={len(self.frame_ids)}, '
            f'use_camera_pose={self.use_camera_pose}, '
            f'invert_camera_pose={self.invert_camera_pose}, '
            f'use_table_frame={self.use_table_frame}, '
            f'point_step={self.point_step}'
        )

    def get_split_dir(self):
        split_candidates = [self.split]

        if self.split == 'train_1':
            split_candidates.append('train1')
        elif self.split == 'train1':
            split_candidates.append('train_1')

        split_dir = None
        for candidate in split_candidates:
            candidate_dir = os.path.join(self.root, candidate)
            if os.path.isdir(candidate_dir):
                split_dir = candidate_dir
                break

        if split_dir is None:
            raise FileNotFoundError(
                'Split directory not found under root: '
                f'{self.root} (tried {split_candidates})'
            )

        return split_dir

    def collect_frame_ids(self):
        split_dir = self.get_split_dir()
        base = os.path.join(
            split_dir,
            f'scene_{self.scene_id:04d}',
            self.camera,
        )

        rgb_dir = os.path.join(base, 'rgb')
        depth_dir = os.path.join(base, 'depth')
        camk_path = os.path.join(base, 'camK.npy')
        poses_path = os.path.join(base, 'camera_poses.npy')
        cam0_wrt_table_path = os.path.join(base, 'cam0_wrt_table.npy')

        if not os.path.exists(camk_path):
            raise FileNotFoundError(f'camK file not found: {camk_path}')
        if not os.path.exists(poses_path):
            raise FileNotFoundError(f'camera_poses file not found: {poses_path}')
        if not os.path.exists(cam0_wrt_table_path):
            raise FileNotFoundError(f'cam0_wrt_table file not found: {cam0_wrt_table_path}')

        if not os.path.isdir(rgb_dir):
            raise FileNotFoundError(f'RGB directory not found: {rgb_dir}')
        if not os.path.isdir(depth_dir):
            raise FileNotFoundError(f'Depth directory not found: {depth_dir}')

        rgb_ids = {
            int(os.path.splitext(name)[0])
            for name in os.listdir(rgb_dir)
            if name.endswith('.png')
        }
        depth_ids = {
            int(os.path.splitext(name)[0])
            for name in os.listdir(depth_dir)
            if name.endswith('.png')
        }

        frame_ids = sorted(rgb_ids & depth_ids)
        self.base_dir = base
        self.camk_path = camk_path
        self.poses_path = poses_path
        self.cam0_wrt_table_path = cam0_wrt_table_path
        return frame_ids

    def get_paths(self, ann_id):
        rgb_path = os.path.join(self.base_dir, 'rgb', f'{ann_id:04d}.png')
        depth_path = os.path.join(self.base_dir, 'depth', f'{ann_id:04d}.png')
        return rgb_path, depth_path, self.camk_path

    def load_frame(self, ann_id):
        rgb_path, depth_path, camk_path = self.get_paths(ann_id)

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

        if self.camK is None:
            camK = np.load(camk_path)
            if camK.shape != (3, 3):
                raise RuntimeError(f'camK shape must be 3x3, got {camK.shape}')
            self.camK = camK

        if self.camera_poses is None:
            camera_poses = np.load(self.poses_path)
            if len(camera_poses.shape) != 3 or camera_poses.shape[1:] != (4, 4):
                raise RuntimeError(
                    f'camera_poses shape must be (N, 4, 4), got {camera_poses.shape}'
                )
            self.camera_poses = camera_poses

        if self.cam0_wrt_table is None:
            cam0_wrt_table = np.load(self.cam0_wrt_table_path)
            if cam0_wrt_table.shape != (4, 4):
                raise RuntimeError(
                    f'cam0_wrt_table shape must be (4, 4), got {cam0_wrt_table.shape}'
                )
            self.cam0_wrt_table = cam0_wrt_table

        return rgb, depth, self.camK

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

    def make_pointcloud2(self, header, rgb, depth, camK, camera_pose=None):
        fx = float(camK[0, 0])
        fy = float(camK[1, 1])
        cx = float(camK[0, 2])
        cy = float(camK[1, 2])

        height, width = depth.shape
        step = max(1, self.point_step)

        us = np.arange(0, width, step, dtype=np.float32)
        vs = np.arange(0, height, step, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        z_mm = depth[::step, ::step].astype(np.float32)
        valid = z_mm > 0
        if not np.any(valid):
            return pc2.create_cloud(header, [], [])

        z = z_mm[valid] / 1000.0
        u = uu[valid]
        v = vv[valid]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        if camera_pose is not None:
            ones = np.ones_like(z)
            cam_points = np.stack([x, y, z, ones], axis=1)
            world_points = (camera_pose @ cam_points.T).T
            x = world_points[:, 0]
            y = world_points[:, 1]
            z = world_points[:, 2]

        rgb_sample = rgb[::step, ::step]
        rgb_valid = rgb_sample[valid]
        r = rgb_valid[:, 0].astype(np.uint32)
        g = rgb_valid[:, 1].astype(np.uint32)
        b = rgb_valid[:, 2].astype(np.uint32)

        rgb_uint32 = (r << 16) | (g << 8) | b
        rgb_float = rgb_uint32.view(np.float32)

        points = list(zip(x.tolist(), y.tolist(), z.tolist(), rgb_float.tolist()))

        frame_id = self.pointcloud_frame_id if camera_pose is not None else self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header.frame_id = frame_id
        return pc2.create_cloud(header, fields, points)

    def publish_frame(self):
        ann_id = self.frame_ids[self.current_index]
        try:
            rgb, depth, camK = self.load_frame(ann_id)
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
        camera_pose = None
        if self.use_camera_pose:
            camera_pose = self.camera_poses[ann_id]
            if self.invert_camera_pose:
                camera_pose = np.linalg.inv(camera_pose)
            if self.use_table_frame:
                camera_pose = self.cam0_wrt_table @ camera_pose
        points_msg = self.make_pointcloud2(header, rgb, depth, camK, camera_pose)

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)
        self.points_pub.publish(points_msg)

        self.current_index += 1
        if self.current_index >= len(self.frame_ids):
            if self.loop:
                self.current_index = 0
            else:
                self.get_logger().info('Reached last frame, stopping playback')
                self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
