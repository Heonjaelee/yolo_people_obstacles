import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs_py import point_cloud2 as pc2  # ROS2 Python helper

class YoloPeopleToCloud(Node):
    def __init__(self):
        super().__init__('yolo_people_to_cloud')
    # 파라미터
        self.declare_parameter('model_path', 'yolo11n-seg.engine')   # 또는 .pt
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('stride', 4)        # 마스크 샘플링 간격(픽셀)
        self.declare_parameter('max_depth_m', 5.0) # 너무 먼 거리 제거
        self.declare_parameter('min_depth_m', 0.3) # 너무 가까운(노이즈) 제거
        self.declare_parameter('use_segmentation', True) # seg 모델이면 True

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().double_value
        self.stride = self.get_parameter('stride').get_parameter_value().integer_value
        self.max_depth = self.get_parameter('max_depth_m').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth_m').get_parameter_value().double_value
        self.use_seg = self.get_parameter('use_segmentation').get_parameter_value().bool_value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # 카메라 파라미터 보관
        self.fx = self.fy = self.cx = self.cy = None

        # 퍼블리셔
        self.pub = self.create_publisher(PointCloud2, '/people_obstacles', 1)

        # 동기화된 구독자: RGB, Depth, CameraInfo
        self.sub_rgb = Subscriber(self, Image, '/camera/color/image_raw')
        self.sub_d = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.sub_info = Subscriber(self, CameraInfo, '/camera/color/camera_info')

        # Approximate sync (시간 차 허용)
        self.ts = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_d, self.sub_info],
                                            queue_size=10, slop=0.05)
        self.ts.registerCallback(self.callback)

        self.get_logger().info('yolo_people_to_cloud node started.')

    def callback(self, rgb_msg, depth_msg, info_msg):
        # 카메라 내부 파라미터 갱신
        K = info_msg.k
        self.fx, self.fy = K[0], K[4]
        self.cx, self.cy = K[2], K[5]

        # 이미지로 변환
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        # Depth: 16UC1(mm) → float32(m)
        depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_mm.dtype != np.uint16:
            # 드라이버 설정에 따라 32FC1(m)일 수도 있음
            if depth_mm.dtype == np.float32:
                depth_m = depth_mm
            else:
                self.get_logger().warn(f'Unexpected depth dtype: {depth_mm.dtype}')
                return
        else:
            depth_m = depth_mm.astype(np.float32) * 0.001

        h, w = depth_m.shape[:2]

        # YOLO 추론(BGR → RGB, 사이즈 자동)
        results = self.model(rgb[:, :, ::-1], conf=self.conf, verbose=False)
        if len(results) == 0 or len(results[0].boxes) == 0:
            return

        boxes = results[0].boxes
        classes = boxes.cls.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy()

        # seg 모델이면 masks 사용
        masks = None
        if self.use_seg and getattr(results[0], "masks", None) is not None:
            # results[0].masks.data: [N, H, W] (float 0/1)
            masks = results[0].masks.data.cpu().numpy()

        # 사람(class id가 COCO 기준 0인 경우가 일반적)
        people_idx = [i for i, c in enumerate(classes) if c == 0 and confs[i] >= self.conf]
        if len(people_idx) == 0:
            return

        # 포인트 누적
        points = []

        for i in people_idx:
            if masks is not None:
                # 세그멘테이션 마스크 사용 (원본 크기로 resize 필요)
                mask = (cv2.resize(masks[i], (w, h)) > 0.5)
            else:
                # 바운딩 박스 기반(간단)
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w - 1, x2), min(h - 1, y2)
                mask = np.zeros((h, w), dtype=bool)
                mask[y1:y2:self.stride, x1:x2:self.stride] = True  # stride 적용

            # stride 적용(마스크가 너무 조밀한 경우)
            if masks is not None and self.stride > 1:
                submask = np.zeros_like(mask)
                submask[::self.stride, ::self.stride] = mask[::self.stride, ::self.stride]
                mask = submask

            ys, xs = np.where(mask)
            if xs.size == 0:
                continue

            zs = depth_m[ys, xs]  # meters
            valid = (zs > self.min_depth) & (zs < self.max_depth) & np.isfinite(zs)
            xs, ys, zs = xs[valid], ys[valid], zs[valid]

            if xs.size == 0:
                continue

            # 픽셀 → 3D (카메라 좌표계)
            X = (xs - self.cx) * zs / self.fx
            Y = (ys - self.cy) * zs / self.fy
            # Z = zs (전방)

            # 필요시 점수 축소(퍼포먼스): 상한(예: 사람당 최대 2000점)
            max_pts_person = 2000
            if X.size > max_pts_person:
                idx = np.linspace(0, X.size - 1, max_pts_person).astype(int)
                X, Y, zs = X[idx], Y[idx], zs[idx]

            person_points = np.stack([X, Y, zs], axis=1)
            points.append(person_points)

        if len(points) == 0:
            return
        cloud_np = np.concatenate(points, axis=0).astype(np.float32)

        # PointCloud2 메시지 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = rgb_msg.header.frame_id  # 예: camera_color_optical_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg = pc2.create_cloud(header, fields, cloud_np)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = YoloPeopleToCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()