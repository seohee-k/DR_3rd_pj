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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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
            check_force_condition, task_compliance_ctrl,DR_MV_MOD_ABS, movesx,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL
        )
        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    def wait_for_nudge_then_draw(path_list, vel=100, acc=100, threshold=20.0):
        print("Z축 방향으로 최소 20.0N 이상 누르면 다음 동작을 수행합니다.")
        try:
            while rclpy.ok():
                if check_force_condition(axis=DR_AXIS_Z, min=threshold, ref=DR_TOOL):
                    print("Nudge 감지됨! 후속 동작 수행 중")
                    break
                time.sleep(0.1)

            # Set eyes position (spiral)

            movesx(path_list, vel=100, acc=100)
            print("경로 실행 중...")
            time.sleep(3.0)
            print("경로 실행 완료")

        except Exception as e:
            print(f"예외 발생: {e}")

    # RViz 마커 퍼블리셔 설정
    marker_pub = node.create_publisher(Marker, "/skeleton_path_marker", 10)
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "skeleton_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.002
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.points = []
    def marker_timer_callback():
        if marker_timer_callback.idx < len(robot_path):
            x, y = robot_path[marker_timer_callback.idx]
            p = Point()
            p.x = (x + 300.0) / 1000.0
            p.y = (y - 220.0) / 1000.0
            p.z = 0.0
            marker.points.append(p)
            marker.header.stamp = node.get_clock().now().to_msg()
            marker_pub.publish(marker)
            marker_timer_callback.idx += 1


    # 초기 세팅
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")

    # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
    img_path = "/home/rokey/Downloads/IMG_1446.jpg"
    img = cv2.imread(img_path)

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
    start_point = (263, 142)    # 수동 설정 가능: (y, x)
    skeleton_path = fast_trace_path(skeleton, start_point)

    # 7. 시각화 (10개마다 점찍기)
    # img_vis = img.copy()
    # for idx, (y, x) in enumerate(skeleton_path[::10]):
    #     cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
    #     cv2.putText(img_vis, str(idx * 10 + 1), (x + 3, y - 3),movesx)
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
    # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    # time.sleep(3.0)
    # set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    # time.sleep(0.3)

    # # 연속 직선 이동
    # movesx(posx_path, vel=VELOCITY, acc=ACC)
    
    # release_force()
    # time.sleep(0.1)
    # release_compliance_ctrl()
    # time.sleep(0.1)

    # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    
    # time.sleep(0.1)
    # set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    movesx(posx_path2, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path3, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path4, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path5, vel=VELOCITY, acc=ACC)
    time.sleep(1.0)
    movesx(posx_path6, vel=VELOCITY, acc=ACC)

    wait_for_nudge_then_draw(robot_path, vel=100, acc=100, threshold=20.0)

    # for path in posx_sets:
    #         if path:
    #             movesx(path, vel=VELOCITY, acc=ACC)
    #             time.sleep(1.0)

    #     # 2. Nudge 감지되면 후속 동작 수행
    #     print("Z축 방향으로 최소 2.0N 이상 누르면 다음 동작을 수행합니다...")
   
    # Or

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
