## Title : m0609_all_move_gear
## Time : 2025-06-16 11:16:37
import rclpy
import DR_init
import time
import numpy as np

# Doosan robot 설정
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
            set_tool,
            set_tcp,
            movej,
            movel,
            amovel,
            wait,
            set_digital_output,
            set_singularity_handling,
            set_desired_force,
            release_force,
            DR_MV_MOD_REL,
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    ## 초기 세팅
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)

    # 속도 설정은 movej/movel 호출시마다 직접 지정
    print("Speed and compliance setup completed")

    def release():
        set_digital_output(2, ON)
        wait(0.5)

    def close():
        set_digital_output(2, OFF)
        wait(3.0)

    def set_compliance():
        set_desired_force(1, [0, 0, 20.0, 0, 0, 0], [0, 0, 1, 0, 0, 0])

    def center_gear():
        movel(posx([0, 0, -30, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        close()
        movel(posx([0, 0, 68.18, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(posx([0, 139.61, 0, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(posx([0, 0, -25, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        set_compliance()
        wait(1.0)  # 검출 대기 (강제 조건 감지 대신 딜레이)
        set_digital_output(2, ON)
        release_force()
        movel(posx([0, 0, 30, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def pick_and_place():
        movel(posx([0, 0, -30, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        close()
        movel(posx([0, 0, 100, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(posx([0, 100, 0, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(posx([0, 0, -30, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
        wait(0.5)
        movel(posx([0, 0, 30, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    # 초기 위치로 이동
    JReady = posj([0, 0, 90, 0, 90, 0])
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 작업 위치 정의 (예시 좌표)
    home = posx([214.47, -5.91, 155.0, 180, 0, 180])
    place1 = posx([300, -100, 100, 180, 0, 180])
    place2 = posx([200, 100, 100, 180, 0, 180])
    place3 = posx([100, 200, 100, 180, 0, 180])

    # 동작 시퀀스
    release()
    movel(home, vel=VELOCITY, acc=ACC)
    movel(place1, vel=VELOCITY, acc=ACC)
    pick_and_place()

    movel(place2, vel=VELOCITY, acc=ACC)
    pick_and_place()

    movel(place3, vel=VELOCITY, acc=ACC)
    pick_and_place()

    movel(home, vel=VELOCITY, acc=ACC)
    release()
    center_gear()

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
