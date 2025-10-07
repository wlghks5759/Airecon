import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleLandDetected

import numpy as np
from scipy.spatial.transform import Rotation, RotationSpline
from scipy.ndimage import gaussian_filter1d
from functools import partial

class MultiDroneMissionNode(Node):
    """
    두 대의 드론을 순차적으로 제어하는 미션 지휘관 노드.
    1번 드론 웨이포인트 비행 -> 1번 드론 착륙 감지 -> 2번 드론 웨이포인트 비행 -> 종료
    """

    def __init__(self):
        super().__init__('multi_drone_mission_node')

        # --- 미션 상태 관리 ---
        self.mission_state = "START"
        self.active_drone_id = 1
        self.offboard_setpoint_counter = 0
        self.precise_land_launched = False

        # --- 파라미터 ---
        self.takeoff_altitude = -5.0
        self.waypoint_tolerance = 0.5
        self.waypoints = []

        # --- 드론별 상태 및 ROS 인터페이스 관리 ---
        self.drone_ids = [1, 2]
        self.drone_states = {}
        self.traj_pubs = {}
        self.mode_pubs = {}
        self.cmd_pubs = {}
        self.pos_subs = {}
        self.land_subs = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for drone_id in self.drone_ids:
            self.drone_states[drone_id] = {
                'local_position': VehicleLocalPosition(),
                'is_landed': True,
                'waypoint_index': 0,
            }

            px4_ns = f"/px4_{drone_id}"
            self.mode_pubs[drone_id] = self.create_publisher(OffboardControlMode, f'{px4_ns}/fmu/in/offboard_control_mode', 10)
            self.traj_pubs[drone_id] = self.create_publisher(TrajectorySetpoint, f'{px4_ns}/fmu/in/trajectory_setpoint', 10)
            self.cmd_pubs[drone_id] = self.create_publisher(VehicleCommand, f'{px4_ns}/fmu/in/vehicle_command', 10)
            
            self.pos_subs[drone_id] = self.create_subscription(
                VehicleLocalPosition, f'{px4_ns}/fmu/out/vehicle_local_position',
                partial(self.vehicle_local_position_callback, drone_id=drone_id), qos_profile
            )
            self.land_subs[drone_id] = self.create_subscription(
                VehicleLandDetected, f'{px4_ns}/fmu/out/vehicle_land_detected',
                partial(self.vehicle_land_detected_callback, drone_id=drone_id), qos_profile
            )

        self.generate_waypoints()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("다중 드론 미션 지휘관 노드가 시작되었습니다.")

    # --- 콜백 함수들 ---
    def vehicle_local_position_callback(self, msg, drone_id):
        self.drone_states[drone_id]['local_position'] = msg

    def vehicle_land_detected_callback(self, msg, drone_id):
        if self.drone_states[drone_id]['is_landed'] != msg.landed:
            self.get_logger().info(f"드론 {drone_id} 착륙 상태 변경: {msg.landed}")
        self.drone_states[drone_id]['is_landed'] = msg.landed

    # --- 메인 상태 머신 ---
    def timer_callback(self):
        # =============== 1번 드론 임무 ===============
        if self.mission_state == "START":
            self.get_logger().info("미션 시작! 1번 드론 이륙 준비.")
            self.mission_state = "DRONE_1_SETUP_OFFBOARD"
            self.offboard_setpoint_counter = 0

        elif self.mission_state == "DRONE_1_SETUP_OFFBOARD":
            if self.offboard_setpoint_counter < 20:
                self.publish_offboard_control_mode(1)
                setpoint = TrajectorySetpoint()
                setpoint.position[0], setpoint.position[1], setpoint.position[2] = 0.0, 0.0, self.takeoff_altitude
                setpoint.yaw = 0.0  # Yaw 고정
                self.publish_trajectory_setpoint(1, setpoint)
                
                if self.offboard_setpoint_counter == 10:
                    self.get_logger().info("1번 드론 Offboard 모드 전환 및 ARM 시도.")
                    self.publish_vehicle_command(
                        1, 
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                        param1=1.0,
                        param2=6.0
                    )
                    self.arm(1)
                
                self.offboard_setpoint_counter += 1
            else:
                self.get_logger().info("Offboard 모드 설정 완료. 이륙 시작.")
                self.mission_state = "DRONE_1_TAKEOFF"

        elif self.mission_state == "DRONE_1_TAKEOFF":
            self.publish_offboard_control_mode(1)
            setpoint = TrajectorySetpoint()
            setpoint.position[0], setpoint.position[1], setpoint.position[2] = 0.0, 0.0, self.takeoff_altitude
            self.publish_trajectory_setpoint(1, setpoint)
            
            current_pos = self.drone_states[1]['local_position']
            current_z = current_pos.z
            current_xy_err = np.linalg.norm([current_pos.x, current_pos.y]) # 수평 위치 오차 계산

            # [수정/확인]: Z 축 도달 및 XY 위치 안정성 확인
            is_at_altitude = abs(current_z - self.takeoff_altitude) < abs(self.takeoff_altitude * 0.05)
            is_xy_stable = current_xy_err < 0.2 # 예: 20cm 이내

            if is_at_altitude and is_xy_stable:
                self.get_logger().info("1번 드론 이륙 완료. 웨이포인트 비행 시작.")
                self.mission_state = "DRONE_1_FLYING_WAYPOINTS"

            
    
        elif self.mission_state == "DRONE_1_FLYING_WAYPOINTS":
            if self.follow_waypoints(drone_id=1):
                if not self.precise_land_launched:
                    self.get_logger().info("1번 드론 웨이포인트 비행 완료. precise_land 노드를 실행합니다.")

                self.mission_state = "DRONE_1_WAITING_FOR_LANDING"

  

        elif self.mission_state == "DRONE_1_WAITING_FOR_LANDING":
            # Offboard 모드는 유지하면서 착륙 감지 대기
            self.publish_offboard_control_mode(1)
            
            if self.drone_states[1]['is_landed']:
                self.get_logger().info("1번 드론 착륙 완료! 2번 드론 미션 시작.")
                self.disarm(1)
                self.active_drone_id = 2
                self.offboard_setpoint_counter = 0
                self.mission_state = "DRONE_2_SETUP_OFFBOARD"

        # =============== 2번 드론 임무 ===============
        elif self.mission_state == "DRONE_2_SETUP_OFFBOARD":
            if self.offboard_setpoint_counter < 20:
                self.publish_offboard_control_mode(2)
                setpoint = TrajectorySetpoint()
                setpoint.position[0], setpoint.position[1], setpoint.position[2] = 0.0, 0.0, self.takeoff_altitude
                setpoint.yaw = 0.0  # Yaw 고정
                self.publish_trajectory_setpoint(2, setpoint)
                
                if self.offboard_setpoint_counter == 10:
                    self.get_logger().info("2번 드론 Offboard 모드 전환 및 ARM 시도.")
                    self.publish_vehicle_command(2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.arm(2)
                
                self.offboard_setpoint_counter += 1
            else:
                self.get_logger().info("2번 드론 Offboard 모드 설정 완료. 이륙 시작.")
                self.mission_state = "DRONE_2_TAKEOFF"

        elif self.mission_state == "DRONE_2_TAKEOFF":
            self.publish_offboard_control_mode(2)
            setpoint = TrajectorySetpoint()
            setpoint.position[0], setpoint.position[1], setpoint.position[2] = 0.0, 0.0, self.takeoff_altitude
            self.publish_trajectory_setpoint(2, setpoint)
            
            current_z = self.drone_states[2]['local_position'].z
            if abs(current_z - self.takeoff_altitude) < abs(self.takeoff_altitude * 0.05):
                self.get_logger().info("2번 드론 이륙 완료. 웨이포인트 비행 시작.")
                self.mission_state = "DRONE_2_FLYING_WAYPOINTS"

        elif self.mission_state == "DRONE_2_FLYING_WAYPOINTS":
            if self.follow_waypoints(drone_id=2):
                self.get_logger().info("2번 드론 웨이포인트 비행 완료. 착륙 명령 전송.")
                self.publish_vehicle_command(2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.mission_state = "DRONE_2_WAITING_FOR_LANDING"

        elif self.mission_state == "DRONE_2_WAITING_FOR_LANDING":
            self.publish_offboard_control_mode(2)
            
            if self.drone_states[2]['is_landed']:
                self.get_logger().info("2번 드론 착륙 완료! 전체 미션 종료.")
                self.disarm(2)
                self.mission_state = "MISSION_COMPLETE"

        elif self.mission_state == "MISSION_COMPLETE":
            self.get_logger().info("모든 드론 미션이 완료되었습니다. 노드 종료.", throttle_duration_sec=5.0)

    # --- 드론 제어 헬퍼 함수들 ---
    def follow_waypoints(self, drone_id):
        """지정한 드론이 웨이포인트를 따라 비행하도록 합니다."""
        self.publish_offboard_control_mode(drone_id)
        
        waypoint_idx = self.drone_states[drone_id]['waypoint_index']

        if waypoint_idx < len(self.waypoints):
            target_waypoint = self.waypoints[waypoint_idx]
            setpoint_msg = self.convert_pose_to_setpoint(target_waypoint)
            self.publish_trajectory_setpoint(drone_id, setpoint_msg)
            
            current_pos = self.drone_states[drone_id]['local_position']
            current_pos_ned = np.array([current_pos.x, current_pos.y, current_pos.z])
            target_pos_ned = np.array(setpoint_msg.position)

            if np.linalg.norm(current_pos_ned - target_pos_ned) < self.waypoint_tolerance:
                self.get_logger().info(f"드론 {drone_id}, 웨이포인트 {waypoint_idx + 1}/{len(self.waypoints)} 도착.")
                self.drone_states[drone_id]['waypoint_index'] += 1
            
            return False
        else:
            # 마지막 웨이포인트에서 호버링
            last_waypoint = self.waypoints[-1]
            setpoint_msg = self.convert_pose_to_setpoint(last_waypoint)
            self.publish_trajectory_setpoint(drone_id, setpoint_msg)
            return True

    def convert_pose_to_setpoint(self, pose_matrix):
        """(3,4) 포즈 행렬을 TrajectorySetpoint 메시지로 변환 (ENU -> NED 포함)"""
        target_pos_enu = pose_matrix[:, 3]
        
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.position[0] = target_pos_enu[1]   # ENU Y -> NED X
        setpoint_msg.position[1] = target_pos_enu[0]   # ENU X -> NED Y
        setpoint_msg.position[2] = -target_pos_enu[2]  # ENU Z -> NED Z

        r = Rotation.from_matrix(pose_matrix[:, 0:3])
        enu_yaw = r.as_euler('zyx')[0] 
        ned_yaw = - (enu_yaw - np.pi/2)
        setpoint_msg.yaw = ned_yaw
        return setpoint_msg

    # --- 퍼블리시 헬퍼 함수들 ---
    def arm(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_vehicle_command(self, drone_id, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pubs[drone_id].publish(msg)

    def publish_offboard_control_mode(self, drone_id):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration, msg.attitude, msg.body_rate = True, False, False, False, False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pubs[drone_id].publish(msg)

    def publish_trajectory_setpoint(self, drone_id, setpoint_msg):
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pubs[drone_id].publish(setpoint_msg)

    # --- 웨이포인트 생성 로직 ---
    def generate_waypoints(self):
        original_poses = self.load_poses_from_file()
        if not original_poses:
            self.get_logger().error("포즈 파일을 로드할 수 없어 노드를 종료합니다.")
            self.destroy_node()
            return
        
        smoothing_sigma = 4.0
        num_waypoints = 50
        positions_orig = np.array([p[:, 3] for p in original_poses])
        positions_smoothed = np.array([gaussian_filter1d(positions_orig[:, i], sigma=smoothing_sigma) for i in range(3)]).T
        rotations_orig = np.array([p[:, 0:3] for p in original_poses])
        rotations_smoothed = self.smooth_orientations(rotations_orig, sigma=smoothing_sigma)
        smoothed_poses = []
        for i in range(len(positions_smoothed)):
            pose = np.eye(3, 4)
            pose[:, 0:3] = rotations_smoothed[i]
            pose[:, 3] = positions_smoothed[i]
            smoothed_poses.append(pose)
        self.waypoints = self.get_waypoints_by_distance(smoothed_poses, num_waypoints)
        self.get_logger().info(f"총 {len(self.waypoints)}개의 웨이포인트가 생성되었습니다.")
    
    def load_poses_from_file(self):
        file_path = "/home/yoo/PROJECT/Airecon/main/poses_odom_modified.txt"
        if not os.path.exists(file_path):
            self.get_logger().error(f"오류: 경로에 파일이 없습니다! '{file_path}'")
            return None
        poses = []
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    values = np.array([float(v) for v in line.strip().split()])
                    if len(values) == 12:
                        poses.append(values.reshape((3, 4)))
            return poses
        except Exception as e:
            self.get_logger().error(f"파일 읽기 오류: {e}")
            return None

    def smooth_orientations(self, rotation_matrices, sigma):
        rotations = Rotation.from_matrix(rotation_matrices)
        quats = rotations.as_quat()
        for i in range(1, len(quats)):
            if np.dot(quats[i-1], quats[i]) < 0: quats[i] = -quats[i]
        smoothed_quats = np.array([gaussian_filter1d(quats[:, i], sigma=sigma) for i in range(4)]).T
        smoothed_quats /= np.linalg.norm(smoothed_quats, axis=1)[:, np.newaxis]
        return Rotation.from_quat(smoothed_quats).as_matrix()

    def get_waypoints_by_distance(self, poses, num_waypoints):
        positions = np.array([p[:, 3] for p in poses])
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
        total_distance = cumulative_distances[-1]
        waypoint_distances = np.linspace(0, total_distance, num_waypoints)
        waypoints = []
        for dist in waypoint_distances:
            idx = np.searchsorted(cumulative_distances, dist, side='right') - 1
            if idx < 0: idx = 0
            if idx >= len(cumulative_distances) - 1:
                interp_pose = poses[-1]
            else:
                dist_in_segment = dist - cumulative_distances[idx]
                segment_length = distances[idx]
                ratio = dist_in_segment / segment_length if segment_length > 0 else 0
                start_pos, end_pos = poses[idx][:, 3], poses[idx+1][:, 3]
                interp_pos = start_pos + ratio * (end_pos - start_pos)
                start_rot, end_rot = Rotation.from_matrix(poses[idx][:, 0:3]), Rotation.from_matrix(poses[idx+1][:, 0:3])
                slerp = RotationSpline([0, 1], Rotation.concatenate([start_rot, end_rot]))
                interp_rot_matrix = slerp(ratio).as_matrix()
                interp_pose = np.eye(3, 4)
                interp_pose[:, 0:3], interp_pose[:, 3] = interp_rot_matrix, interp_pos
            waypoints.append(interp_pose)
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
