import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

class YoloGimbalControllerNode(Node):


    def __init__(self):
        super().__init__('yolo_gimbal_controller')

        # --- Gimbal Control Parameters ---
        self.PITCH_DEG = -20.0
        self.YAW_SPEED_DEG_PER_SEC = 10.0
        self.FREQUENCY = 200.0

        self.pitch_rad_ = self.PITCH_DEG * math.pi / 180.0
        yaw_speed_rad_per_sec = self.YAW_SPEED_DEG_PER_SEC * math.pi / 180.0
        self.yaw_increment_ = yaw_speed_rad_per_sec / self.FREQUENCY
        self.current_yaw_rad_ = 0.0

        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # --- YOLO and OpenCV Parameters ---
        self.get_logger().info("Loading YOLOv8n model...")
        self.yolo_model_ = YOLO('yolov8n.pt')  # YOLOv8n 모델 
        self.yolo_model_.to(device)
        self.get_logger().info("YOLOv8n model loaded successfully.")
        self.bridge_ = CvBridge()

        # --- Publishers ---
        self.pitch_publisher_ = self.create_publisher(
            Float64, '/model/x500_gimbal_1/command/gimbal_pitch', 10)
        self.yaw_publisher_ = self.create_publisher(
            Float64, '/model/x500_gimbal_1/command/gimbal_yaw', 10)
        # 객체 인식 결과를 발행할 퍼블리셔
        self.detection_publisher_ = self.create_publisher(
            Image, '/gimbal/image_with_detections', 10)

        # --- Subscribers ---
        # 짐벌 카메라 이미지 토픽 구독
        self.image_subscription_ = self.create_subscription(
            Image,
            '/world/world_demo/model/x500_gimbal_1/link/camera_link/sensor/gimbal/image',
            self.image_callback,
            10)

        # --- Timers ---
        timer_period = 1.0 / self.FREQUENCY
        self.gimbal_timer_ = self.create_timer(timer_period, self.gimbal_timer_callback)

        self.get_logger().info(
            f"YOLO Gimbal Controller Node started.\n"
            f"Pitch: {self.PITCH_DEG} deg, Yaw Speed: {self.YAW_SPEED_DEG_PER_SEC} deg/s, Freq: {self.FREQUENCY} Hz"
        )

    def gimbal_timer_callback(self):
        """
        타이머에 의해 주기적으로 호출되어 짐벌을 제어하는 함수.
        """
        # Pitch 각도 발행
        pitch_msg = Float64()
        pitch_msg.data = self.pitch_rad_
        self.pitch_publisher_.publish(pitch_msg)

        # Yaw 각도 계산 및 발행
        self.current_yaw_rad_ += self.yaw_increment_
        if self.current_yaw_rad_ > (2 * math.pi):
            self.current_yaw_rad_ -= (2 * math.pi)

        yaw_msg = Float64()
        yaw_msg.data = self.current_yaw_rad_
        self.yaw_publisher_.publish(yaw_msg)

    def image_callback(self, msg: Image):
        """
        이미지 토픽을 수신했을 때 호출되는 콜백 함수.
        """
        try:
            # ROS Image 메시지를 OpenCV 이미지(BGR 형식)로 변환
            cv_image = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # YOLOv8 모델로 객체 인식 수행
        results = self.yolo_model_(cv_image, verbose=False) # verbose=False로 설정하여 콘솔 출력 최소화

        # 결과 이미지를 가져옴 (bounding box 등이 그려진 이미지)
        annotated_frame = results[0].plot()

        try:
            # 처리된 OpenCV 이미지를 ROS Image 메시지로 다시 변환하여 발행
            detection_msg = self.bridge_.cv2_to_imgmsg(annotated_frame, "bgr8")
            detection_msg.header = msg.header # 원본 이미지의 타임스탬프와 프레임 ID를 유지
            self.detection_publisher_.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')


def main(args=None):
    rclpy.init(args=args)
    yolo_gimbal_controller_node = YoloGimbalControllerNode()
    rclpy.spin(yolo_gimbal_controller_node)

    # 노드 종료 시
    yolo_gimbal_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
