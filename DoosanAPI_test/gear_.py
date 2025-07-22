import rclpy
import DR_init
import cv2
import numpy as np
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import skeletonize
from collections import deque

# === 로봇 설정 ===
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
Z_UP = 100.0
Z_DOWN = 5.0

# === Doosan ROS2 초기화 ===
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("gear_drawer_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node  # ✅ 반드시 node 연결

# ✅ 이제 DSR API import 가능
from DSR_ROBOT2 import movej, movel, get_current_posx
from DR_common2 import posx, posj

# === 이미지 불러오기 및 전처리 ===
img_path = "/home/rokey/Downloads/cat.jpg"
img = cv2.imread(img_path)
if img is None:
    print(f"❌ 이미지 파일을 찾을 수 없습니다: {img_path}")
    exit(1)

PIXEL_WIDTH, PIXEL_HEIGHT = 210, 297
REAL_WIDTH_MM, REAL_HEIGHT_MM = 210.0, 297.0
scale_x = REAL_WIDTH_MM / PIXEL_WIDTH
scale_y = REAL_HEIGHT_MM / PIXEL_HEIGHT

img = cv2.resize(img, (PIXEL_WIDTH, PIXEL_HEIGHT))
gray_img = rgb2gray(img)
thresh_val = threshold_otsu(gray_img)
binary_img = gray_img < thresh_val
skeleton = skeletonize(binary_img)

# === 한붓그리기 경로 추출 ===
def find_endpoints(skel_img):
    endpoints = []
    for y in range(1, skel_img.shape[0] - 1):
        for x in range(1, skel_img.shape[1] - 1):
            if skel_img[y, x] == 1:
                neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                if neighbors == 1:
                    endpoints.append((y, x))
    return endpoints

def fast_trace_path(skel_img, start):
    visited = np.zeros_like(skel_img, dtype=bool)
    path = []
    q = deque()
    q.append(start)
    visited[start] = True

    while q:
        y, x = q.popleft()
        path.append((y, x))
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dy == 0 and dx == 0:
                    continue
                ny, nx = y + dy, x + dx
                if 0 <= ny < skel_img.shape[0] and 0 <= nx < skel_img.shape[1]:
                    if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))
    return path

endpoints = find_endpoints(skeleton)
start_point = endpoints[0] if endpoints else (0, 0)
skeleton_path = fast_trace_path(skeleton, start_point)
robot_path = [(x * scale_x, y * scale_y) for y, x in skeleton_path[::10]]  # 10픽셀 간격 샘플링 후 mm 변환

# === 로봇 제어 함수 ===
def safe_draw_path(x, y, index):
    print(f"[#{index}] 이동: ({x:.1f}, {y:.1f})")
    movel(posx([x, y, Z_UP, 180, 0, 180]), vel=VELOCITY, acc=ACC)
    movel(posx([x, y, Z_DOWN, 180, 0, 180]), vel=VELOCITY, acc=ACC)
    movel(posx([x, y, Z_UP, 180, 0, 180]), vel=VELOCITY, acc=ACC)

# === 메인 ===
def main(args=None):
    set_tool = DR_init._DR_set_tool
    set_tcp = DR_init._DR_set_tcp
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    print("[▶] 시작 자세 이동")
    JReady = posj([0, 0, 90, 0, 90, 0])
    movej(JReady, vel=VELOCITY, acc=ACC)

    for i, (x, y) in enumerate(robot_path):
        safe_draw_path(x, y, i + 1)

    print("[🏁] 종료 자세 복귀")
    movej(JReady, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
