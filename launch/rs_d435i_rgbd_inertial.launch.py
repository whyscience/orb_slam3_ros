import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    orb_slam3_ros_share_dir = get_package_share_directory('orb_slam3_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='orb_slam3_ros',
            executable='ros_rgbd_inertial',
            name='orb_slam3',
            output='screen',
            remappings=[
                ('/camera/rgb/image_raw', '/camera/color/image_raw'),
                ('/camera/depth_registered/image_raw', '/camera/aligned_depth_to_color/image_raw'),
                ('/imu/data', '/camera/imu')
            ],
            parameters=[
                {'voc_file': os.path.join(orb_slam3_ros_share_dir, 'orb_slam3', 'Vocabulary', 'ORBvoc.txt')},
                {'settings_file': os.path.join(orb_slam3_ros_share_dir, 'config', 'RGB-D-Inertial',
                                               'RealSense_D435i.yaml')},
                {'world_frame_id': 'world'},
                {'cam_frame_id': 'camera'},
                {'imu_frame_id': 'imu'},
                {'enable_pangolin': True},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(orb_slam3_ros_share_dir, 'config', 'orb_slam3_rgbd.rviz')]
        ),

        # Node(
        #     package='hector_trajectory_server',
        #     executable='hector_trajectory_server',
        #     name='trajectory_server_orb_slam3',
        #     namespace='orb_slam3_ros',
        #     output='screen',
        #     parameters=[
        #         {'/target_frame_name': '/world'},
        #         {'/source_frame_name': '/imu'},
        #         {'/trajectory_update_rate': 20.0},
        #         {'/trajectory_publish_rate': 20.0}
        #     ]
        # )
    ])
