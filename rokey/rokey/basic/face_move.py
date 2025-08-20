import rclpy
import DR_init
import math
import time
from std_msgs.msg import Float64MultiArray
# from robot_control.onrobot import RG

from pydub import AudioSegment
from pydub.playback import play

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
ON, OFF = 1, 0

# DSR 로봇 제어 초기화
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# 전역 변수
received = False
xy_coords = [0.0, 0.0]
MOVE_DISTANCE = 50

def xy_callback(msg):
    global xy_coords, received
    if not received and len(msg.data) >= 2:
        xy_coords = msg.data[:2]
        received = True
        print(f"[INFO] 좌표 수신 완료: x={xy_coords[0]:.2f}, z={xy_coords[1]:.2f}")
    elif len(msg.data) < 2:
        print("[WARN] 메시지에 좌표가 부족합니다.")

def move_to_face(node, movel, posx):
    global xy_coords, received

    received = False  # 매 요청마다 초기화
    print("[INFO] 좌표를 기다리는 중...")

    # 메시지 수신 대기
    while rclpy.ok() and not received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if received:
        # 감지된 좌표를 posx에 반영 (x와 z 좌표만 사용)
        pos = posx([
            xy_coords[0],    # x
            -252.0,          # y (고정)
            540,             # z (고정)
            91, -88.00, 90.09  # 오리엔테이션 고정
        ])
        print("→ movel 감지된 좌표")
        movel(pos, vel=VELOCITY, acc=ACC)
    else:
        print("[ERROR] 좌표를 수신하지 못했습니다.")


def main(args=None):
    global xy_coords, received

    rclpy.init(args=args)
    node = rclpy.create_node("face_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR 관련 모듈 import (init 이후)
    from DSR_ROBOT2 import movej, movel, set_tool, set_tcp, get_tool_force, get_current_posx, trans, DR_BASE
    from DR_common2 import posx, posj

    """
    def release():
        print("[INFO] 그리퍼 열기")
        gripper.open_gripper()
        while gripper.get_status()[0]:  # 동작 중이면 대기
            time.sleep(0.2)

    def grip():
        print("[INFO] 그리퍼 닫기")
        gripper.close_gripper()
        while gripper.get_status()[0]:  # 동작 중이면 대기
            time.sleep(0.2)
"""
    def check_grip():
        threshold = 1.5
        initial_force = get_tool_force() # force_ext: WORLD좌표계 기준 툴의 외력
        while True:
            current_force = get_tool_force()
            # print(current_force)
            force_diff = [abs(c - i) for c, i in zip(current_force, initial_force)]
            if any(diff > threshold for diff in force_diff):
                print(f"[INFO] 외력 변화 감지: {force_diff}")
                break
            time.sleep(0.1)

    def move_up():
        current_pos = get_current_posx()[0]
        current_pos[2] = current_pos[2] + MOVE_DISTANCE
        print(current_pos)
        movel(current_pos, 30, 30)

    def move_down():
        current_pos = get_current_posx()[0]
        current_pos[2] = current_pos[2] - MOVE_DISTANCE
        print(current_pos)
        movel(current_pos, 30, 30)

    def move_up():
        current_pos = get_current_posx()[0]
        current_pos[2] = current_pos[2] + MOVE_DISTANCE
        print(current_pos)
        movel(current_pos, 30, 30)

    
    JReady = [0, 0, 90, 0, 90, 0]

    # 도구 및 TCP 설정
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    print("→ movej JReady")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 좌표 구독 시작 (한 번만 설정)
    node.create_subscription(Float64MultiArray, "/remapped_coord", xy_callback, 10)

    print("▶ '1' 입력 시 감지된 좌표로 movel 실행 (Ctrl+C로 종료):")

    try:
        while rclpy.ok():
            user_input = input("> ")
            # 여기로 와
            if user_input.strip() == "1":
                song = AudioSegment.from_mp3("start_moving.mp3")
                play(song)
                move_to_face(node, movel, posx)
            # 
            elif user_input.strip() =="2":
                print("그리퍼를 놓습니다")
                # release()
            elif user_input.strip() =="3":
                print("그리퍼를 닫습니다")
                # grip()
            elif user_input.strip() =="4":
                check_grip()
                # release()

            # 기본 자세
            elif user_input.strip() =="5":
                movej(JReady, VELOCITY, ACC)

            # 위로 가기
            elif user_input.strip() =="6":
                move_up()

            else:
                print("[INFO] '1'이 입력되지 않았습니다. 다시 입력하거나 Ctrl+C로 종료하세요.")
    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료 (Ctrl+C).")

    rclpy.shutdown()

if __name__ == "__main__":
    main()