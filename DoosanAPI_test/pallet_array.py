import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def get_pattern_point(pos1, pos2, pos3, pos4, idx, row, col, stack, thickness, offset):
    i = idx % col                    # column index (X direction)
    j = (idx // col) % row           # row index (Y direction)
    k = idx // (col * row)           # stack index (Z direction)

    x_step = (pos2[0] - pos1[0]) / (col - 1) if col > 1 else 0
    y_step = (pos3[1] - pos1[1]) / (row - 1) if row > 1 else 0
    z_step = thickness

    x = pos1[0] + i * x_step + offset[0]
    y = pos1[1] + j * y_step + offset[1]
    z = pos1[2] + k * z_step + offset[2]

    return [x, y, z, pos1[3], pos1[4], pos1[5]]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, wait, trans,
            set_digital_output,
            DR_BASE
        )
        from DR_common2 import posx
    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    def grip():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait(0.3)

    def release():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        wait(0.3)

    set_tool("Tool Weight_test")
    set_tcp("test_TCP")

    pos1 = posx(250, 100, 38.3, 0, 180, 0)
    pos2 = posx(255, 100, 38.3, 0, 180, 0)
    pos3 = posx(300, 100, 38.3, 0, 180, 0)
    pos4 = posx(250, 150, 38.3, 0, 180, 0)
    pos5 = posx(255, 150, 38.3, 0, 180, 0)
    pos6 = posx(300, 150, 38.3, 0, 180, 0)
    pos7 = posx(250, 200, 38.3, 0, 180, 0)
    pos8 = posx(255, 200, 38.3, 0, 180, 0)
    pos9 = posx(300, 200, 38.3, 0, 180, 0)

    row, col, stack = 3, 3, 1
    total_count = row * col * stack
    v, a = 200, 50

    item_heights = [5.2, 5.7, 5.9, 7.1, 7.3, 7.5, 9.0, 9.2, 9.4]  # Example in cm
    categorized_indices = []

    for i, h in enumerate(item_heights):
        if 6.0 <= h < 6.9:
            categorized_indices.append((i, 0))  # Pallet B index 0~2
        elif 7.0 <= h < 7.9:
            categorized_indices.append((i, 3))  # Pallet B index 3~5
        elif 8.0 <= h < 8.9:
            categorized_indices.append((i, 6))  # Pallet B index 6~8

    for orig_idx, target_base_idx in categorized_indices:
        z_offset = item_heights[orig_idx]

        pos_from = get_pattern_point(pos1, pos2, pos3, pos4, orig_idx, row, col, stack, 0, [0, 0, z_offset])
        pos_to = get_pattern_point(pos1, pos2, pos3, pos4, target_base_idx, row, col, stack, 0, [0, 150, z_offset])

        above_from = trans(pos_from, [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)
        above_to = trans(pos_to, [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE)

        movel(above_from, v, a)
        release()
        movel(pos_from, v, a)
        grip()
        wait(1.0)
        movel(above_from, v, a)
        movel(above_to, v, a)
        movel(pos_to, v, a)
        release()
        wait(1.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
