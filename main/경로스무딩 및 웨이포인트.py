import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, RotationSpline
from scipy.ndimage import gaussian_filter1d

# --- 1. 데이터 로드 함수 ---
def load_poses_from_file(filename="poses_odom.txt"):
    poses = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                values = np.array([float(v) for v in line.strip().split()])
                if len(values) != 12:
                    continue  # 잘못된 줄은 건너뛰기
                pose_matrix = values.reshape((3, 4))
                poses.append(pose_matrix)
        return poses
    except FileNotFoundError:
        print(f"오류: '{filename}' 파일을 찾을 수 없습니다.")
        return None


# --- 2. 자세 스무딩 함수 ---
def smooth_orientations(rotation_matrices, sigma):
    rotations = Rotation.from_matrix(rotation_matrices)
    quats = rotations.as_quat()

    # 연속성 유지
    for i in range(1, len(quats)):
        if np.dot(quats[i - 1], quats[i]) < 0:
            quats[i] = -quats[i]

    smoothed_quats = np.zeros_like(quats)
    for i in range(4):
        smoothed_quats[:, i] = gaussian_filter1d(quats[:, i], sigma=sigma)

    # 정규화
    norms = np.linalg.norm(smoothed_quats, axis=1)
    smoothed_quats /= norms[:, np.newaxis]

    return Rotation.from_quat(smoothed_quats).as_matrix()


# --- 3. 시각화 함수 ---
def plot_poses(ax, poses, color, label, subsample_rate=30, axis_length=0.2):
    positions = np.array([p[:, 3] for p in poses])
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            color=color, label=label, zorder=1)
    
    for i in range(0, len(poses), subsample_rate):
        pose = poses[i]
        origin = pose[:, 3]
        rot_matrix = pose[:, 0:3]
        x_axis, y_axis, z_axis = rot_matrix.T

        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2],
                  length=axis_length, color='r')
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2],
                  length=axis_length, color='g')
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2],
                  length=axis_length, color='b')


# --- 4. 웨이포인트 추출 함수 ---
def get_waypoints_by_distance(poses, num_waypoints):
    positions = np.array([p[:, 3] for p in poses])
    distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
    total_distance = cumulative_distances[-1]

    waypoint_distances = np.linspace(0, total_distance, num_waypoints)
    waypoints = []

    for dist in waypoint_distances:
        idx = np.searchsorted(cumulative_distances, dist, side='right') - 1
        idx = np.clip(idx, 0, len(distances) - 1)

        dist_in_segment = dist - cumulative_distances[idx]
        segment_length = distances[idx]
        ratio = dist_in_segment / segment_length if segment_length > 0 else 0

        # 위치 보간
        start_pos = poses[idx][:, 3]
        end_pos = poses[idx + 1][:, 3]
        interp_pos = start_pos + ratio * (end_pos - start_pos)

        # 자세 보간 (Slerp)
        start_rot = Rotation.from_matrix(poses[idx][:, 0:3])
        end_rot = Rotation.from_matrix(poses[idx + 1][:, 0:3])
        slerp = RotationSpline([0, 1], Rotation.concatenate([start_rot, end_rot]))
        interp_rot_matrix = slerp(ratio).as_matrix()

        # 보간된 포즈 생성
        interp_pose = np.eye(3, 4)
        interp_pose[:, 0:3] = interp_rot_matrix
        interp_pose[:, 3] = interp_pos
        waypoints.append(interp_pose)

    return waypoints


# --- 5. 메인 실행 ---
if __name__ == "__main__":
    original_poses = load_poses_from_file("poses_odom_modified.txt")
    if original_poses is None or len(original_poses) == 0:
        exit()

    smoothing_sigma = 4.0
    positions_orig = np.array([p[:, 3] for p in original_poses])
    positions_smoothed = np.array([
        gaussian_filter1d(positions_orig[:, i], sigma=smoothing_sigma)
        for i in range(3)
    ]).T

    rotations_orig = np.array([p[:, 0:3] for p in original_poses])
    rotations_smoothed = smooth_orientations(rotations_orig, sigma=smoothing_sigma)

    smoothed_poses = []
    for i in range(len(positions_smoothed)):
        pose = np.eye(3, 4)
        pose[:, 0:3] = rotations_smoothed[i]
        pose[:, 3] = positions_smoothed[i]
        smoothed_poses.append(pose)

    # --- 웨이포인트 추출 ---
    num_waypoints = 100
    waypoints = get_waypoints_by_distance(smoothed_poses, num_waypoints)

    # --- 시각화 ---
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    plot_poses(ax, original_poses, 'lightgray', 'Original Path', subsample_rate=100, axis_length=0.2)
    plot_poses(ax, smoothed_poses, 'blue', f'Smoothed Path (sigma={smoothing_sigma})', subsample_rate=100, axis_length=0.2)
    plot_poses(ax, waypoints, 'red', f'{num_waypoints} Waypoints', subsample_rate=1, axis_length=0.5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Path Smoothing and Waypoint Generation')
    ax.legend()
    ax.set_box_aspect([1, 1, 1])  # 3D에서 axis('equal') 대신 사용
    plt.show()
