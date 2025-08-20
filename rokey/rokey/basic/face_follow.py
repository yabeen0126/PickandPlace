import rclpy
import DR_init
from std_msgs.msg import Float64MultiArray

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

# DSR 로봇 제어 초기화
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 전역 변수
received = False
xy_coords = [0.0, 0.0]

def xy_callback(msg):
    global xy_coords, received
    if not received and len(msg.data) >= 2:
        xy_coords = msg.data[:2]
        received = True
        print(f"[INFO] 좌표 수신 완료: x={xy_coords[0]:.2f}, z={xy_coords[1]:.2f}")
    elif len(msg.data) < 2:
        print("[WARN] 메시지에 좌표가 부족합니다.")

def main(args=None):
    global xy_coords, received

    rclpy.init(args=args)
    node = rclpy.create_node("face_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR 관련 모듈 import (init 이후)
    from DSR_ROBOT2 import movej, movel, set_tool, set_tcp
    from DR_common2 import posx, posj

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
            # user_input = input("> ")
            # if user_input.strip() == "1":
            if True:
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
                        540,             # z 고정
                        # xy_coords[1],    # z
                        91, -88.00, 90.09  # 오리엔테이션 고정
                    ])

                    # print("→ movej JReady")
                    # movej(JReady, vel=VELOCITY, acc=ACC)

                    print("→ movel 감지된 좌표")
                    movel(pos, vel=VELOCITY, acc=ACC)
                else:
                    print("[ERROR] 좌표를 수신하지 못했습니다.")
            else:
                print("[INFO] '1'이 입력되지 않았습니다. 다시 입력하거나 Ctrl+C로 종료하세요.")
    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료 (Ctrl+C).")

    rclpy.shutdown()

if __name__ == "__main__":
    main()