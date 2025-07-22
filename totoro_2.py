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
            check_force_condition, move_periodic, check_position_condition, task_compliance_ctrl,DR_MV_MOD_ABS, movesx, amovesx,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL, DR_FC_MOD_ABS,
            DR_FC_MOD_REL, mwait
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return
    

    def draw_with_force_sync(path_list, velocity=100, accel=100):
        try:
            print("힘 제어 시작")
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(0.2)
            set_desired_force(fd=[0, 0, -3, 0, 0, 0],
                            dir=[0, 0, 1, 0, 0, 0],
                            mod=DR_FC_MOD_REL)
            time.sleep(0.3)

            print(f"{len(path_list)}점 경로 그리기 시작")
            amovesx(path_list, vel=velocity, acc=accel)

            # 로봇 동작 완료 대기
            print("로봇 동작 종료 대기 중 ")
            mwait(1)
            print("로봇 동작 완료")
        except Exception as e:
            print(f"예외 발생: {e}")
        finally:
            print("힘 제어 해제")
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()


    # 초기 세팅
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")
    # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
    img_path = "/home/rokey/Downloads/IMG_1446.jpg"
    img = cv2.imread(img_path)
    # img = cv2.resize(img, (210, 297))  # (width, height)

    # 2. 흑백 변환 및 이진화
    gray_img = rgb2gray(img)
    thresh_val = threshold_otsu(gray_img)
    binary_img = gray_img < thresh_val  # 검정 선 추출을 위해 반전

    # 3. 스켈레톤 중심선 추출
    skeleton = skeletonize(binary_img)

    # 4. 끝점 찾기 함수
    def find_endpoints(skel_img):
        endpoints = []
        for x in range(1, skel_img.shape[0] - 1):
            for y in range(1, skel_img.shape[1] - 1):
                if skel_img[x, y] == 1:
                    neighbors = np.sum(skel_img[x-1:x+2, y-1:y+2]) - 1
                    if neighbors == 1:
                        endpoints.append((x, y))
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
            path.append((x, y))
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < skel_img.shape[1]) and (0 <= ny < skel_img.shape[0]):
                        if skel_img[nx, ny] == 1 and not visited[nx, ny]:
                            visited[nx, ny] = True
                            q.append((nx, ny))
        return path

    # 6. 시작점 설정 및 경로 추출
    endpoints = find_endpoints(skeleton)
    start_point = (273, 135) 
    skeleton_path = fast_trace_path(skeleton, start_point)
    
    # 7. 시각화 (10개마다 점찍기)
    # img_vis = img.copy()
    # for idx, (x, y) in enumerate(skeleton_path[::10]):
    #     cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
    #     cv2.putText(img_vis, str(idx * 10 + 1), (x + 3, y - 3),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0), 1)

    # 8. 로봇 좌표로 변환
    robot_path = [(x, y) for x, y in skeleton_path]

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def close():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(3.0)


    def pick_and_place():
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        close()
        movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=100.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(300.0, 0.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=100.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        release()
        wait(0.5)
        movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=30.0, ref=DR_BASE, mod=DR_MV_MOD_REL)



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

    # Remove ##

    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    wait(3.0)
    # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
    # wait(0.5)
    set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)


    # 연속 직선 이동
    movesx(posx_path, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path2, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path3, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path4, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path5, vel=VELOCITY, acc=ACC)
    time(1.0)
    movesx(posx_path6, vel=VELOCITY, acc=ACC)

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
