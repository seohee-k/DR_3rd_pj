import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from scipy.spatial.transform import Rotation as R


class LineDrawerNode(Node):
    def __init__(self):
        super().__init__('line_drawer_node')
        self.publisher_ = self.create_publisher(Pose, '/robot_pose', 10)
        self.z_offset = 15.0  # 공중 높이 (mm)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_counter = 1  # ✅ 경로 번호 카운터

    def publish_pose(self, x1, y1, z1, x2, y2, z2, label=''):
        step_id = self.pose_counter
        for t in np.linspace(0, 1, 10):
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            z = z1 if t == 0 else (z2 if t == 1 else (z1 if z1 == self.z_offset else z2))
            pose = Pose()
            pose.position = Point(x=x/100.0, y=y/100.0, z=z/100.0)
            q = R.from_euler('xyz', [0.0, 0.0, 0.0]).as_quat()
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.publisher_.publish(pose)
            self.get_logger().info(f"[{label}][#{step_id}] Move to: ({x}, {y}, {z})")
        self.pose_counter += 1


# 이미지 불러오기 및 전처리
img = cv2.imread('nemo.png', cv2.IMREAD_COLOR)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, threshold = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
threshold = cv2.bitwise_not(threshold)
kernel = np.ones((5, 5), np.uint8)
threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel, iterations=2)
threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel, iterations=1)
edges = cv2.Canny(threshold, 5, 30, apertureSize=5)

# 외곽 윤곽선 추출
contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
external_contour = max(contours, key=cv2.contourArea)

# ROS 2 노드 시작
rclpy.init()
node = LineDrawerNode()

# 1. 외곽 먼저 그리기
for i in range(len(external_contour) - 1):
    x1, y1 = map(int, external_contour[i][0])
    x2, y2 = map(int, external_contour[i + 1][0])
    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    print(f"[외곽][#{node.pose_counter}] ({x1}, {y1}) → ({x2}, {y2})")
    node.publish_pose(x1, y1, 0.0, x2, y2, 0.0, label="외곽")

# 2. 내부 디테일 (Hough 선)
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=15, minLineLength=3, maxLineGap=50)
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = map(int, line[0])
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2

        if cv2.pointPolygonTest(external_contour, (x1, y1), False) >= 0 and \
           cv2.pointPolygonTest(external_contour, (x2, y2), False) >= 0:

            z = node.z_offset if cv2.pointPolygonTest(external_contour, (mid_x, mid_y), False) < 0 else 0.0

            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            print(f"[내부][#{node.pose_counter}] ({x1}, {y1}) → ({x2}, {y2}) z={z}")
            node.publish_pose(x1, y1, z, x2, y2, z, label="내부")

# 이미지 시각화
cv2.imshow('One-Stroke Drawing Result', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 종료
node.destroy_node()
rclpy.shutdown()
