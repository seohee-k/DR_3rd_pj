import rclpy
import DR_init
import time
import numpy as np

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
            check_force_condition, move_periodic, check_position_condition,task_compliance_ctrl,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL, DR_FC_MOD_ABS,
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

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def close():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(3.0)

    def center_gear():
        movel(posx(0.0, 0.0, -100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        close()
        movel(posx(0.0, 0.0, 150.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=68.18, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(300.0, 0.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=139.61, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, 0.0, -115.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        set_stiffnessx(stx = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0], time=0.0)        
        task_compliance_ctrl(stx = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0], time=0.0)

        wait(0.1)
        set_desired_force([0.0, 0.0, -10.0, 0, 0, 0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

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
        while True:
            while check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
                print("Check force")
                move_periodic(
                    amp=[0, 0, 0, 0, 0, 20],
                    period=[0, 0, 0, 0, 0, 1.5],
                    atime=0.0, repeat=1, ref=DR_TOOL
                )
                break
        
        
            
    # 초기 자세로 이동
    JReady = posj([0, 0, 90, 0, 90, 0])
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 위치 설정
    place1 = posx([214.47, -5.91, 155.0, 116.43, -177.79, 119.23])
    place2 = posx([302.56, 53.13, 155.0, 123.30, 178.09, 123.12])
    place3 = posx([309.74, -48.01, 155.0, 132.11, 178.60, 149.86])
    place4 = posx([274.77, 1.97, 155.0, 93.90, 179.10, 90.43])

    # release()
    # movel(place1, vel=VELOCITY, acc=ACC)
    # pick_and_place()

    # movel(place2, vel=VELOCITY, acc=ACC)
    # pick_and_place()

    # movel(place3, vel=VELOCITY, acc=ACC)
    # pick_and_place()

    movel(place4, vel=VELOCITY, acc=ACC)
    release()
    center_gear()

    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
