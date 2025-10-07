import matplotlib.pyplot as plt
import numpy as np

def visualize_rtabmap_pose_with_orientation(filepath, step=10, axis_length=0.2):
    """
    RTAB-Map 포즈 데이터 파일을 읽어 3D 경로와 자세(좌표축)를 시각화하는 함수.

    Args:
        filepath (str): 포즈 데이터가 저장된 .txt 파일의 경로.
        step (int): 몇 개의 포즈마다 좌표축을 그릴지 결정하는 간격.
        axis_length (float): 시각화될 좌표축의 길이.
    """
    poses = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 12:
                    try:
                        # 12개의 숫자를 3x4 행렬로 변환
                        pose_matrix = np.array([float(p) for p in parts]).reshape(3, 4)
                        poses.append(pose_matrix)
                    except ValueError:
                        print(f"경고: 숫자 변환 실패. 라인 건너뜀: {line.strip()}")
    except FileNotFoundError:
        print(f"오류: '{filepath}' 파일을 찾을 수 없습니다.")
        return
    except Exception as e:
        print(f"파일 처리 중 오류 발생: {e}")
        return

    if not poses:
        print("시각화할 유효한 데이터가 없습니다.")
        return

    # 위치 데이터만 추출하여 경로 생성
    translations = np.array([p[:, 3] for p in poses])
    x_coords, y_coords, z_coords = translations[:, 0], translations[:, 1], translations[:, 2]

    # 3D 그래프 생성
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 1. 전체 이동 경로 그리기
    ax.plot(x_coords, y_coords, z_coords, linestyle='-', color='cyan', label='Trajectory')

    # 2. 일정 간격(step)으로 자세(좌표축) 그리기
    for i, pose in enumerate(poses):
        if i % step == 0:
            origin = pose[:, 3]  # 이동 벡터 (좌표축의 원점)
            R = pose[:, :3]      # 회전 행렬

            # 회전 행렬의 각 열이 새로운 X, Y, Z 축의 방향 벡터가 됨
            # X축 (빨간색)
            ax.quiver(origin[0], origin[1], origin[2], R[0, 0], R[1, 0], R[2, 0],
                      length=axis_length, color='r', normalize=True)
            # Y축 (녹색)
            ax.quiver(origin[0], origin[1], origin[2], R[0, 1], R[1, 1], R[2, 1],
                      length=axis_length, color='g', normalize=True)
            # Z축 (파란색)
            ax.quiver(origin[0], origin[1], origin[2], R[0, 2], R[1, 2], R[2, 2],
                      length=axis_length, color='b', normalize=True)

    # 시작점과 끝점 표시
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='lime', s=100, label='Start', depthshade=True)
    ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='magenta', s=100, label='End', depthshade=True)

    # 그래프 제목 및 라벨 설정
    ax.set_title('RTAB-Map Trajectory with Pose Visualization')
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_zlabel('Z coordinate')
    ax.legend()
    
    # 각 축의 스케일을 동일하게 맞춰 왜곡을 방지
    # 데이터의 범위를 기반으로 축 제한을 설정하여 더 나은 시각화 제공
    max_range = np.array([x_coords.max()-x_coords.min(), y_coords.max()-y_coords.min(), z_coords.max()-z_coords.min()]).max() / 2.0
    mid_x = (x_coords.max()+x_coords.min()) * 0.5
    mid_y = (y_coords.max()+y_coords.min()) * 0.5
    mid_z = (z_coords.max()+z_coords.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


    plt.show()

# --- 메인 코드 실행 부분 ---
if __name__ == "__main__":
    file_to_visualize = 'poses_odom_modified.txt'
    
    # step: 15개의 포즈마다 하나씩 자세를 표시
    # axis_length: 좌표축의 길이를 0.3으로 설정
    visualize_rtabmap_pose_with_orientation(file_to_visualize, step=15, axis_length=0.3)
