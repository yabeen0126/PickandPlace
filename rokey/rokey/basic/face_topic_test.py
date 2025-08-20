import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# 전역 변수
received = False
xy_coords = [0.0, 0.0]

def xy_callback(msg):
    global xy_coords, received
    if not received and len(msg.data) >= 2:
        xy_coords = msg.data[:2]
        received = True
        print(f"[INFO] 수신된 좌표:")
        print(f"    x = {xy_coords[0]:.2f}")
        print(f"    z = {xy_coords[1]:.2f}")
    elif len(msg.data) < 2:
        print("[WARN] 메시지에 좌표가 부족합니다.")

def main(args=None):
    global xy_coords, received

    rclpy.init(args=args)
    node = rclpy.create_node("face_topic_listener")

    print("▶ '1' 입력 시 /remapped_coord 토픽 수신 시작 (종료하려면 Ctrl+C):")

    while rclpy.ok():
        user_input = input("> ")
        if user_input.strip() == "1":
            received = False  # 초기화 (반복 수신 가능하도록)
            print("[INFO] 좌표를 수신 중...")

            # 토픽 구독 시작
            node.create_subscription(Float64MultiArray, "/remapped_coord", xy_callback, 10)

            # 메시지 수신 대기 루프
            while rclpy.ok() and not received:
                rclpy.spin_once(node, timeout_sec=0.1)

            print("[INFO] 수신 완료. 다시 '1'을 입력해 반복하거나 Ctrl+C로 종료하세요.")
        else:
            print("[INFO] '1'이 입력되지 않았습니다. 다시 입력하거나 Ctrl+C로 종료하세요.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
