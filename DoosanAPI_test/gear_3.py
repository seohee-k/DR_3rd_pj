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
            check_force_condition, move_periodic, check_position_condition, task_compliance_ctrl,DR_MV_MOD_ABS, movesx,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL, DR_FC_MOD_ABS,
            DR_FC_MOD_REL,
            set_stiffnessx
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    # 초기 세팅
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")
    # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
    img_path = "/home/rokey/Downloads/cat.jpg"
    img = cv2.imread(img_path)
    img = cv2.resize(img, (210, 297))  # (width, height)

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
    start_point = endpoints[0]  # 수동 설정 가능: (y, x)
    skeleton_path = fast_trace_path(skeleton, start_point)

    # 7. 시각화 (10개마다 점찍기)
    img_vis = img.copy()
    for idx, (y, x) in enumerate(skeleton_path[::10]):
        cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
        cv2.putText(img_vis, str(idx * 10 + 1), (x + 3, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0), 1)

    # plt.figure(figsize=(8, 11))
    # plt.imshow(cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB))
    # plt.title("Skeleton Path (Resized to 210x297)")
    # plt.axis("off")
    # plt.show()

    # 8. 로봇 좌표로 변환
    robot_path = [(x, y) for y, x in skeleton_path]

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def close():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(3.0)

    # def set_compliance():
    #     wait(0.1)
    #     set_desired_force([0.0, 0.0, -10.0, 0, 0, 0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    def center_gear():
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        close()
        movel(posx(0.0, 0.0, 150.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=68.18, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(300.0, 0.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=139.61, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, 0.0, -115.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # wait(3.0)
        
        set_stiffnessx(stx=[500, 500, 500, 100, 100, 100],time=0.0)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100],time=0.0)
        wait(0.5)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        wait(1.0)
        detecting()

    def pick_and_place():
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        close()
        movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=100.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(300.0, 0.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=100.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        release()
        wait(0.5)
        movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    def detecting():
        while rclpy.ok():
            while not check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
                print("힘 체크 완료")
                move_periodic(
                    amp=[0, 0, 0, 0, 0, 20],
                    period=[0, 0, 0, 0, 0, 1.5],
                    atime=0.0, repeat=3, ref=DR_TOOL
                )
            
                while check_position_condition(axis=DR_AXIS_Z, max=60, ref=DR_BASE):
                    print("포지션 체크")

                    release()
                    release_force(time=0.0)
                    movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    break

        release_compliance_ctrl()

    # 초기 자세로 이동

    # movel(posx([0, 0, 90, 0, 180, 0]), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
    JReady = [0, -20, 110, 0, 90, 0]
    movej(JReady, v=20, a=20)
    Ready = posx([567.68, 112.22, 100, 118.03, 180, 118.08])
    movel(Ready, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # movel(posx([0, -20, 110, 0, 60, 0]), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
    old_x, old_y = robot_path[0]

    # for i, (x, y) in enumerate(robot_path[::10]):  # 간격 줄이기        
    #     if i == 0 :
    #         movel(posx([567.68 - x , 112.22 - y, 90, 118.03, 180, 118.08]) ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    #         # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    #         # wait(3.0)
    #         # # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
    #         # wait(0.5)
    #         # set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            

    #     else:
            
    #         print("작동중")
    #         print(f"x:{x}, y:{y}")
    #         # wait(1.0)
    #         current_x = old_x - x
    #         current_y = old_y - y
    #         movel(posx([current_x, current_y, 0, 0, 0, 0]) ,vel=VELOCITY, acc=ACC, radius=1.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
    #         old_x = x
    #         old_y = y
    base_x, base_y = 567.68, 112.22

    # 경로 점들을 posx 형태로 변환 (절대 위치 기준)
    posx_path = []
    posx_path2 = []
    posx_path3 = []
    for (x, y) in robot_path[10::10]:
        p = posx([float(x)+210.0, float(y)-220.0, 90.0, 118.03, 180.0, 118.08])
        if len(posx_path) < 80 : 
            posx_path.append(p)
        elif len(posx_path2) < 80 :
            posx_path2.append(p)
        elif len(posx_path3) < 80 :
            posx_path3.append(p)
    # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    # wait(3.0)
    # #         # # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
    # wait(0.5)
    # set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # 연속 직선 이동
    movesx(posx_path, vel=VELOCITY, acc=ACC)
    

    wait(1.0)
    movesx(posx_path2, vel=VELOCITY, acc=ACC)
    wait(1.0)
    movesx(posx_path3, vel=VELOCITY, acc=ACC)
    # 위치 설정
    # place1 = posx([214.47, -5.91, 155.0, 116.43, -177.79, 119.23])
    # place2 = posx([302.56, 53.13, 155.0, 123.30, 178.09, 123.12])
    # place3 = posx([309.74, -48.01, 155.0, 132.11, 178.60, 149.86])
    # place4 = posx([274.77, 1.97, 155.0, 93.90, 179.10, 90.43])

    # # release()
    # movel(place1, vel=VELOCITY, acc=ACC)
    # pick_and_place()

    # movel(place2, vel=VELOCITY, acc=ACC)
    # pick_and_place()

    # movel(place3, vel=VELOCITY, acc=ACC)
    # pick_and_place()
    # movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    # movel(place4, vel=VELOCITY, acc=ACC)
    # release()
    # center_gear()

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
