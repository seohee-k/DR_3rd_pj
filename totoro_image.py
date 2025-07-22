import rclpy
import DR_init
import time
import numpy as np
import cv2
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import skeletonize
from skimage.filters import threshold_otsu

from collections import deque
import matplotlib.pyplot as plt
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1
'''
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, amovel, wait,
            set_digital_output, set_singularity_handling,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, move_periodic, check_position_condition, task_compliance_ctrl,DR_MV_MOD_ABS, movesx, amovesx, mwait,
            DR_BASE, DR_TOOL, DR_AXIS_Z, DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_FC_MOD_REL
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    # 초기 세팅
    set_tool("Tool Weight_test") # 덮어쓰기 되어서 상관없음
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")
    # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
    img_path = "/home/shindonghyun/doosan_ws/image/totoro_image.jpg"
    img = cv2.imread(img_path)
    # img = cv2.resize(img, (210, 297))  # (width, height) => 토토로 픽셀에 맞추고 나서 필요없음

    # 2. 흑백 변환 및 이진화
    gray_img = rgb2gray(img)
    thresh_val = threshold_otsu(gray_img)
    binary_img = gray_img < thresh_val  # 검정 선 추출을 위해 반전

    # 3. 스켈레톤 중심선 추출
    skeleton = skeletonize(binary_img)

    # 4. 끝점 찾기 함수
    def find_endpoints(skel_img):
        endpoints = []
        for y in range(1, skel_img.shape[0] - 1):
            for x in range(1, skel_img.shape[1] - 1):
                if skel_img[y, x] == 1:
                    neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                    if neighbors == 1:
                        endpoints.append((y, x))
        return endpoints

    # 5. BFS 기반 경로 추적
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

    # 6. 시작점 설정 및 경로 추출
    endpoints = find_endpoints(skeleton)
    start_point = (273, 135)   # 수동 설정 가능: (y, x)
    skeleton_path = fast_trace_path(skeleton, start_point)

    # 7. 시각화 (10개마다 점찍기)
    img_vis = img.copy()
    for idx, (y, x) in enumerate(skeleton_path[::10]):
        cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
        cv2.putText(img_vis, str(idx * 10 + 1), (x + 3, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0), 1)

    # plt로 시각화 해서 확인
    # plt.figure(figsize=(8, 11))
    # plt.imshow(cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB))
    # plt.title("Skeleton Path (Resized to 210x297)")
    # plt.axis("off")
    # plt.show()

    # 8. 로봇 좌표로 변환
    robot_path = [(x, y) for y, x in skeleton_path]
    JReady = [0, -20, 110, 0, 90, 0]
    movej(JReady, v=20, a=20)
    Ready = posx([567.68, 112.22, 100, 118.03, 180, 118.08])
    movel(Ready, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # movel(posx([0, -20, 110, 0, 60, 0]), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
    old_x, old_y = robot_path[0]
    # 경로 점들을 posx 형태로 변환 (절대 위치 기준)
    posx_path = []
    posx_path2 = []
    posx_path3 = []
    posx_path4 = []
    posx_path5 = []
    posx_path6 = []
    posx_path7 = []
    posx_path8 = []
    posx_path9 = []
    for (x, y) in robot_path[5::5]:
        p = posx([float(x)+300.0, float(y)-220.0, 90.0, 118.03, 180.0, 118.08])
        if len(posx_path) < 80 : 
            posx_path.append(p)
        elif len(posx_path2) < 80 :
            posx_path2.append(p)
        elif len(posx_path3) < 80 :
            posx_path3.append(p)
        elif len(posx_path4) < 80 :
            posx_path4.append(p)
        elif len(posx_path5) < 80 :
            posx_path5.append(p)
        elif len(posx_path6) < 80 :
            posx_path6.append(p) 
        elif len(posx_path7) < 80 :
            posx_path7.append(p)
        elif len(posx_path8) < 80 :
            posx_path8.append(p)
        elif len(posx_path9) < 80 :
            posx_path9.append(p) 
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    wait(0.5)
    set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    amovesx(posx_path, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    # 다음 경로 블럭 실행
    amovesx(posx_path2, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    # 3. 이후 경로들 연속 실행 (force mode 지속 중)
    amovesx(posx_path2, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    amovesx(posx_path3, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    amovesx(posx_path4, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    amovesx(posx_path5, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    amovesx(posx_path6, vel=VELOCITY, acc=ACC)
    wait(0.5)
    mwait(0)

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


# 지피티 시발놈
'''
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, amovel, wait,
            set_digital_output, set_singularity_handling,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, move_periodic, check_position_condition,
            task_compliance_ctrl, DR_MV_MOD_ABS, movesx, amovesx, mwait,
            DR_BASE, DR_TOOL, DR_AXIS_Z, DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_FC_MOD_REL
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    # ---------- 기본 세팅 ----------
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")

    # ---------- 이미지 처리 ----------
    img_path = "/home/shindonghyun/doosan_ws/image/totoro_image.jpg"
    img = cv2.imread(img_path)
    gray_img = rgb2gray(img)
    binary_img = rgb2gray(img) < threshold_otsu(gray_img)
    skeleton = skeletonize(binary_img)

    def fast_trace_path(skel_img, start):
        visited = np.zeros_like(skel_img, dtype=bool)
        path = []
        q = deque([start])
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

    skeleton_path = fast_trace_path(skeleton, (273, 135))
    robot_path = [(x, y) for y, x in skeleton_path]

    # ---------- 로봇 초기 위치 ----------
    movej([0, -20, 110, 0, 90, 0], v=20, a=20)
    movel(posx([567.68, 112.22, 100, 118.03, 180, 118.08]), v=20, a=20,
          vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # ---------- 경로 포인트 변환 및 분할 ----------
    posx_path_list = [[] for _ in range(9)]
    path_index = 0
    for i, (x, y) in enumerate(robot_path):
        if i % 5 != 0:
            continue
        p = posx([float(x) + 300.0, float(y) - 220.0, 90.0, 118.03, 180.0, 118.08])
        if len(posx_path_list[path_index]) < 80:
            posx_path_list[path_index].append(p)
        else:
            path_index += 1
            if path_index >= len(posx_path_list):
                break
            posx_path_list[path_index].append(p)

    # ---------- 힘 제어 ----------
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    wait(0.5)
    set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # ---------- 경로 따라 이동 ----------
    for idx, path in enumerate(posx_path_list):
        if not path:
            continue
        print(f"[INFO] Executing path block {idx + 1} with {len(path)} points.")
        amovesx(path, vel=VELOCITY, acc=ACC)
        wait(0.5)
        mwait(0)
    print(f"[INFO] Executing path block {idx + 1} with {len(path)} points.")
    amovesx(path, vel=VELOCITY, acc=ACC)
    print(f"[DEBUG] Waiting for motion to complete (mwait)")
    wait(0.5)
    mwait(0)
    print(f"[INFO] Path block {idx + 1} completed.\n")
    # (선택) 힘제어 종료
    release_compliance_ctrl()
    print("✅ Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


