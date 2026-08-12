from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'root',
            default_value='/data/graspnet',
            description='GraspNet dataset root directory',
        ),
        DeclareLaunchArgument(
            'split',
            default_value='train_1',
            description='Dataset split directory, for example train_1 or train1',
        ),
        DeclareLaunchArgument(
            'scene_id',
            default_value='0',
            description='Scene index to play',
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='realsense',
            description='Camera directory name',
        ),
        DeclareLaunchArgument(
            'ann_id',
            default_value='0',
            description='Annotation/frame index to publish',
        ),
        DeclareLaunchArgument(
            'start',
            default_value='-1',
            description='First frame id in playback range, or -1 for full scene',
        ),
        DeclareLaunchArgument(
            'end',
            default_value='-1',
            description='Last frame id in playback range, or -1 for full scene',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_color_optical_frame',
            description='ROS frame_id',
        ),
        DeclareLaunchArgument(
            'pointcloud_frame_id',
            default_value='graspnet_table',
            description='Frame id for published point cloud',
        ),
        DeclareLaunchArgument(
            'use_camera_pose',
            default_value='true',
            description='Whether to transform points with per-frame camera poses',
        ),
        DeclareLaunchArgument(
            'invert_camera_pose',
            default_value='false',
            description='Whether to invert the per-frame camera pose before applying it',
        ),
        DeclareLaunchArgument(
            'use_table_frame',
            default_value='true',
            description='Whether to compose the per-frame pose with cam0_wrt_table',
        ),
        DeclareLaunchArgument(
            'hz',
            default_value='1.0',
            description='Publish frequency in Hz',
        ),
        DeclareLaunchArgument(
            'point_step',
            default_value='2',
            description='Stride for point cloud sampling',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Whether to loop back to the first frame after the last one',
        ),
        DeclareLaunchArgument(
            'player_package',
            default_value='graspnet_ros2_cpp',
            description='ROS package that provides the player executable',
        ),
        DeclareLaunchArgument(
            'player_executable',
            default_value='graspnet_player_cpp',
            description='Player executable name',
        ),
        DeclareLaunchArgument(
            'publish_rgb',
            default_value='false',
            description='Whether to publish /camera/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'publish_depth',
            default_value='false',
            description='Whether to publish /camera/camera/aligned_depth_to_color/image_raw',
        ),
        DeclareLaunchArgument(
            'publish_camera_info',
            default_value='false',
            description='Whether to publish /camera/camera/color/camera_info',
        ),
        Node(
            package=LaunchConfiguration('player_package'),
            executable=LaunchConfiguration('player_executable'),
            name='graspnet_player',
            output='screen',
            parameters=[{
                'root': LaunchConfiguration('root'),
                'split': LaunchConfiguration('split'),
                'scene_id': LaunchConfiguration('scene_id'),
                'camera': LaunchConfiguration('camera'),
                'ann_id': LaunchConfiguration('ann_id'),
                'start': LaunchConfiguration('start'),
                'end': LaunchConfiguration('end'),
                'frame_id': LaunchConfiguration('frame_id'),
                'pointcloud_frame_id': LaunchConfiguration('pointcloud_frame_id'),
                'use_camera_pose': LaunchConfiguration('use_camera_pose'),
                'invert_camera_pose': LaunchConfiguration('invert_camera_pose'),
                'use_table_frame': LaunchConfiguration('use_table_frame'),
                'hz': LaunchConfiguration('hz'),
                'point_step': LaunchConfiguration('point_step'),
                'loop': LaunchConfiguration('loop'),
                'publish_rgb': LaunchConfiguration('publish_rgb'),
                'publish_depth': LaunchConfiguration('publish_depth'),
                'publish_camera_info': LaunchConfiguration('publish_camera_info'),
            }],
        ),
    ])
