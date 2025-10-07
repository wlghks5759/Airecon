import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():




    return LaunchDescription([
        
        SetParameter(name='use_sim_time', value=True),
        
        # Gazebo to ROS2 bridge - 실제 존재하는 토픽들
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/world/world_demo/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                 '--ros-args', '--log-level', 'warn'],
            output='screen',
            name='gz_bridge'
        ),
        
        #RGB
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='rgb_camera_1_tf',
            arguments=['--x', '0', '--y', '0.15', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_1/camera_link/IMX214']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='rgb_camera_2_tf',
            arguments=['--x', '0.12', '--y', '0', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_2/camera_link/IMX214']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='rgb_camera_3_tf',
            arguments=['--x', '0', '--y', '-0.15', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-3.1416',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_3/camera_link/IMX214']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='rgb_camera_4_tf',
            arguments=['--x', '-0.12', '--y', '0', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '1.5708',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_4/camera_link/IMX214']
        ),


        #DEPTH
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='depth_camera_1_tf',
            arguments=['--x', '0', '--y', '0.15', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_1/camera_link/StereoOV7251']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='depth_camera_2_tf',
            arguments=['--x', '0.12', '--y', '0', '--z', '0',
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_2/camera_link/StereoOV7251']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='depth_camera_3_tf',
            arguments=['--x', '0', '--y', '-0.15', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-3.1416',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_3/camera_link/StereoOV7251']
        ),

        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='depth_camera_4_tf',
            arguments=['--x', '-0.12', '--y', '0', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '1.5708',
                    '--frame-id', 'base_link', '--child-frame-id', 'x500_depth_0/oakd_lite_camera_4/camera_link/StereoOV7251']
        ),


        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_imu_tf',
        #     arguments=['--x', '0', '--y', '0', '--z', '0.242',
        #                '--roll', '0', '--pitch', '0', '--yaw', '0',
        #                '--frame-id', 'base_link',
        #                '--child-frame-id', 'x500_depth_0/base_link/imu_sensor']
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0.242',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom',
                       '--child-frame-id', 'base_link']
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom_tf',
        #     arguments=['--x', '0', '--y', '0', '--z', '0',
        #                '--roll', '0', '--pitch', '0', '--yaw', '0',
        #                '--frame-id', 'map',
        #                '--child-frame-id', 'odom']
        # ),



        Node(
            package='rtabmap_sync', executable='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True, # 근사 동기화 사용
                'queue_size': 10,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_1/link/camera_link/sensor/StereoOV7251/depth_image'),
                ('rgbd_image', 'rgbd_image0'),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True, # 근사 동기화 사용
                'queue_size': 10,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_2/link/camera_link/sensor/StereoOV7251/depth_image'),
                ('rgbd_image', 'rgbd_image1'),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True, # 근사 동기화 사용
                'queue_size': 10,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_3/link/camera_link/sensor/StereoOV7251/depth_image'),
                ('rgbd_image', 'rgbd_image2'),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True, # 근사 동기화 사용
                'queue_size': 10,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/world/world_demo/model/x500_depth_0/model/oakd_lite_camera_4/link/camera_link/sensor/StereoOV7251/depth_image'),
                ('rgbd_image', 'rgbd_image3'),
            ]
        ),

        # Node(
        #     package='rtabmap_sync', executable='rgbdx_sync',
        #     output='screen',
        #     parameters=[{
        #         'approx_sync': True,
        #         'queue_size': 10,
        #         'rate': 10.0,
        #         'rgbd_cameras': 4,
        #     }],
        #     remappings=[
        #         ('rgbd_image0', 'rgbd_image0'),
        #         ('rgbd_image1', 'rgbd_image1'),
        #         ('rgbd_image2', 'rgbd_image2'),
        #         ('rgbd_image3', 'rgbd_image3'),
        #     ]
        # ),



        Node(
            package='rtabmap_odom', executable='rgbd_odometry', 
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_rgbd': True,
                # 'rgbd_cameras': 3,


            }],
            remappings=[
                # ('rgbd_image0', 'rgbd_image0'),
                ('rgbd_image', 'rgbd_image1'),
                # ('rgbd_image2', 'rgbd_image2'),
                ('odom', '/odom'),
                ('imu', '/world/world_demo/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),

            ],
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', 
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                # 'rgbd_cameras': 4,
                'rgbd_cameras': 4,
                'subscribe_odom_info': True,
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'database_path': '~/database.db'

            }],
            remappings=[
                ('rgbd_image0', 'rgbd_image0'),
                ('rgbd_image1', 'rgbd_image1'),
                ('rgbd_image2', 'rgbd_image2'),
                ('rgbd_image3', 'rgbd_image3'),
                # ('rgbd_image', 'rgbd_image1'),
                ('odom', '/odom'),
                ('imu', '/world/world_demo/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),

            ],
        ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', 
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_odom_info': True,
                'subscribe_imu': True,
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True, 
                # 'rgbd_cameras': 4,
                'rgbd_cameras': 4,
                

            }],
            remappings=[
                ('rgbd_image0', 'rgbd_image0'),
                ('rgbd_image1', 'rgbd_image1'),
                ('rgbd_image2', 'rgbd_image2'),
                ('rgbd_image3', 'rgbd_image3'),
                # ('rgbd_image', 'rgbd_image1'),
                ('odom', '/odom'),
                ('odom_info', '/odom_info'),

            ],
        ),
    ])





