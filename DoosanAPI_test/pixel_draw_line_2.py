import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler  # ROS tf

class LineDrawerNode(Node):
    def __init__(self):
        super().__init__('line_drawer_node')
        self.publisher_ = self.create_publisher(Pose, '/robot_pose', 10)
        self.z_offset = 15.0  # Z축 오프셋 (공중에 띄우기 위한 높이, mm)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # 기본 수직 자세 (0, 0, 0)

    def publish_pose(self, x1, y1, z1, x2, y2, z2, rx=0.0, ry=0.0, rz=0.0):
        # 두 점 사이 보간 (단순화된 직선 경로)
        for t in np.linspace(0, 1, 10):  # 10단계로 보간
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            z = z1 if t == 0 else (z2 if t == 1 else (z1 if z1 == self.z_offset else z2))
            pose = Pose()
            pose.position = Point(x=x/100.0, y=y/100.0, z=z/100.0)  # 픽셀을 mm로 변환 (100:1 비율)
            # Euler 각도를 쿼터니언으로 변환
            q = quaternion_from_euler(rx, ry, rz)
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.publisher_.publish(pose)
            self.get_logger().info(f"Move to: ({x}, {y}, {z}, rx={rx}, ry={ry}, rz={rz})")

# 이미지 처리
img = cv2.imread('pixel_character.jpg', cv2.IMREAD_COLOR)  # 이미지 파일명 수정
gray = cv2.imread('pixel_character.jpg', cv2.IMREAD_GRAYSCALE)

# 전처리: 이진화 및 에지 검출
_, threshold = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
threshold = cv2.bitwise_not(threshold)  # 배경을 검정, 캐릭터를 흰색으로
kernel = np.ones((5, 5), np.uint8)
threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel, iterations=2)
threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel, iterations=1)
edges = cv2.Canny(threshold, 5, 30, apertureSize=5)

# Hough Transform으로 직선 탐지
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=15, minLineLength=3, maxLineGap=50)

# 외곽 윤곽선만 추출
contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
external_contour = max(contours, key=cv2.contourArea)  # 가장 큰 외곽 윤곽선 선택

# ROS 2 노드 초기화
rclpy.init()
node = LineDrawerNode()

# 이미지에 선 그리기 및 로봇 경로 계획
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        # 외곽 윤곽선에 속하는 선만 그리기
        if any(cv2.pointPolygonTest(external_contour, (x1, y1), False) >= 0 and 
               cv2.pointPolygonTest(external_contour, (x2, y2), False) >= 0):
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            print(f"Hough Line coordinates: ({x1}, {y1}) to ({x2}, {y2})")
            # 로봇 팔 경로: 내부 라인은 Z축 올리기
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            if cv2.pointPolygonTest(external_contour, (mid_x, mid_y), False) < 0:
                node.publish_pose(x1, y1, node.z_offset, x2, y2, node.z_offset)  # 공중
            else:
                node.publish_pose(x1, y1, 0.0, x2, y2, 0.0)  # 종이

# 윤곽선으로 보완 (외곽만)
for i in range(len(external_contour) - 1):
    x1, y1 = external_contour[i][0]
    x2, y2 = external_contour[i + 1][0]
    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    print(f"Contour Line coordinates: ({x1}, {y1}) to ({x2}, {y2})")
    node.publish_pose(x1, y1, 0.0, x2, y2, 0.0)  # 외곽은 종이 위

# 결과 이미지 표시
cv2.imshow('Detected Lines', img)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()

# ROS 2 종료
node.destroy_node()
rclpy.shutdown()