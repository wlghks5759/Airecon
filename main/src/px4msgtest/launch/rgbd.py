from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration



def generate_launch_description():


    return LaunchDescription([
        
        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'base_link', '--child-frame-id', 'oak-d-base-frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical1_tf',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'odom', '--child-frame-id', 'base_link']
        ),


        # Node(
        #     package='tf2_ros', 
        #     executable='static_transform_publisher',
        #     name='points_camera_tf',
        #     arguments=['--x', '0.10', '--y', '0.0', '--z', '0.0', 
        #             '--roll', str(-(pi/2)), '--pitch', '0', '--yaw', str(-(pi/2)),
        #             '--frame-id', 'base_link', '--child-frame-id', 'oak_right_camera_optical_frame']
        # ),



        Node(
            package='rtabmap_odom', executable='rgbd_odometry', 
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,    
                'approx_sync': False,
                'approx_sync_max_interval': 0.005,

            }],
            remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('imu', '/mavros/imu/data'),
            ],),



        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', 
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': False,
                'approx_sync_max_interval': 0.005,
                'subscribe_depth': True,
                'subscribe_rgb': True,

                


            }],
            remappings=[
                ('rgb/image', '/oak/rgb/image_raw'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/oak/stereo/image_raw'),
                ('odom', '/odom'),
                ('imu', '/mavros/imu/data'),

            ],
            ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', 
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                # 'subscribe_scan_cloud': True,
                'subscribe_odom_info': True,
                'frame_id': 'base_link',
                'approx_sync_max_interval': 0.01,

            }],
            remappings=[
                ('rgb/image', '/oak/rgb/image_raw'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/oak/stereo/image_raw'),
                ('odom', '/odom'),
                ('odom_info','/odom_info'),
                ('imu', '/mavros/imu/data'),
                #('scan_cloud', '/oak/points')

            ],
            ),
    ])




