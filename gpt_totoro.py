import rclpy
import DR_init
import time
import numpy as np
import cv2
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import skeletonize
from collections import deque
import matplotlib.pyplot as plt

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, amovel, wait,
            set_digital_output, set_singularity_handling,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, move_periodic, check_position_condition, task_compliance_ctrl, DR_MV_MOD_ABS, movesx, amovesx,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL, DR_FC_MOD_ABS,
            DR_FC_MOD_REL,
            mwait
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    def draw_with_force_sync(path_list, velocity=100, accel=100):
        try:
            print("\n[🔧] 힘 제어 시작")
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(0.2)
            set_desired_force(fd=[0, 0, -3, 0, 0, 0],
                              dir=[0, 0, 1, 0, 0, 0],
                              mod=DR_FC_MOD_REL)
            time.sleep(0.3)

            print(f"[✏️] {len(path_list)}점 경로 실행")
            amovesx(path_list, vel=velocity, acc=accel)

            print("[⏳] 로봇 동작 완료 대기 중 (mwait)...")
            mwait()
            print("[✅] 동작 완료")

        except Exception as e:
            print(f"[❌] 예외 발생: {e}")
        finally:
            print("[🔓] 힘 제어 해제")
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()

    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("[⚙️] Speed and compliance setup completed")

    img_path = "/home/rokey/Downloads/IMG_1446.jpg"
    img = cv2.imread(img_path)
    gray_img = rgb2gray(img)
    thresh_val = threshold_otsu(gray_img)
    binary_img = gray_img < thresh_val
    skeleton = skeletonize(binary_img)

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
                    if (0 <= ny < skel_img.shape[0]) and (0 <= nx < skel_img.shape[1]):
                        if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
        return path

    endpoints = find_endpoints(skeleton)
    start_point = (273, 135)
    skeleton_path = fast_trace_path(skeleton, start_point)

    robot_path = [(x, y) for y, x in skeleton_path]

    posx_sets = [[] for _ in range(9)]
    for (x, y) in robot_path[5::5]:
        p = posx([float(x)+300.0, float(y)-220.0, 90.0, 118.03, 180.0, 118.08])
        for s in posx_sets:
            if len(s) < 80:
                s.append(p)
                break

    JReady = [0, -20, 110, 0, 90, 0]
    movej(JReady, v=20, a=20)
    Ready = posx([567.68, 112.22, 100, 118.03, 180, 118.08])
    movel(Ready, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    for path in posx_sets:
        if path:
            draw_with_force_sync(path)

    print("\n✅ 전체 그리기 작업 완료.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
