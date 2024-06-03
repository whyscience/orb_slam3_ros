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
            executable='ros_stereo_inertial',
            name='orb_slam3',
            output='screen',
            remappings=[
                ('/camera/left/image_raw', '/cam0/image_raw'),
                ('/camera/right/image_raw', '/cam1/image_raw'),
                ('/imu/data', '/imu0')
            ],
            parameters=[
                {'voc_file': os.path.join(orb_slam3_ros_share_dir, 'orb_slam3', 'Vocabulary', 'ORBvoc.txt.bin')},
                {'settings_file': os.path.join(orb_slam3_ros_share_dir, 'config', 'Stereo-Inertial', 'TUM-VI.yaml')},
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
            arguments=['-d', os.path.join(orb_slam3_ros_share_dir, 'config', 'orb_slam3_with_imu.rviz')]
        ),

        # Node(
        #     package='hector_trajectory_server',
        #     executable='hector_trajectory_server',
        #     name='trajectory_server_orb_slam3',
        #     namespace='orb_slam3_ros',
        #     output='screen',
        #     parameters=[
        #         {'target_frame_name': '/world'},
        #         {'source_frame_name': '/imu'},
        #         {'trajectory_update_rate': 20.0},
        #         {'trajectory_publish_rate': 20.0}
        #     ]
        # )
    ])
