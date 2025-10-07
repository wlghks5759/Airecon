from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ✅ 드론1용 Aruco Detector
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            namespace='px4_1',
            name='aruco_detector_drone1',
            parameters=[{
                'aruco_id': 0,
                'dictionary': 2,
                'marker_size': 0.5
            }],
            remappings=[
                (
                    '/world/world_demo/model/x500_gimbal_0/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/image',
                    '/world/world_demo/model/x500_gimbal_1/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/image'
                ),
                (
                    '/world/world_demo/model/x500_gimbal_0/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/camera_info',
                    '/world/world_demo/model/x500_gimbal_1/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/camera_info'
                )
            ]
        ),

        # ✅ 드론2용 Aruco Detector
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            namespace='px4_2',
            name='aruco_detector_drone2',
            parameters=[{
                'aruco_id': 1,
                'dictionary': 2,
                'marker_size': 0.5
            }],
            remappings=[
                (
                    '/world/world_demo/model/x500_gimbal_0/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/image',
                    '/world/world_demo/model/x500_gimbal_2/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/image'
                ),
                (
                    '/world/world_demo/model/x500_gimbal_0/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/camera_info',
                    '/world/world_demo/model/x500_gimbal_2/model/oakd_lite_camera_5/link/camera_link/sensor/IMX214/camera_info'
                )
            ]
        )

    ])
