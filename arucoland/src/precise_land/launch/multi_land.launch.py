from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ✅ 드론 1의 정밀 착륙 노드
        Node(
            package='precise_land',        # 패키지 이름이 precise_land 라고 가정
            executable='precise_land',     # 실행 파일 이름이 precise_land 라고 가정
            namespace='px4_1',
            name='precise_land_node1',
            parameters=[{
                'land_detected_topic': '/px4_1/fmu/out/vehicle_land_detected'
            }],
        ),

        # ✅ 드론 2의 정밀 착륙 노드
        Node(
            package='precise_land',        # 패키지 이름이 precise_land 라고 가정
            executable='precise_land',     # 실행 파일 이름이 precise_land 라고 가정
            namespace='px4_2',
            name='precise_land_node2',
            parameters=[{
                'land_detected_topic': '/px4_2/fmu/out/vehicle_land_detected'
            }],
        )
    ])
