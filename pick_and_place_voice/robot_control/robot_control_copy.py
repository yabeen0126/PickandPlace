import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# DSR Init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Gripper Init
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        # 좌표 구독 설정 (move_to_face 용)
        self.xy_coords = [0.0, 0.0]
        self.received = False
        self.create_subscription(Float64MultiArray, "/remapped_coord", self.xy_callback, 10)

    def xy_callback(self, msg):
        if not self.received and len(msg.data) >= 2:
            self.xy_coords = msg.data[:2]
            self.received = True
            self.get_logger().info(f"[INFO] 좌표 수신 완료: x={self.xy_coords[0]:.2f}, z={self.xy_coords[1]:.2f}")

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def robot_control(self):
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)

        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()
            target_list = get_keyword_result.message.split()

            if "come_here" in target_list:
                self.get_logger().info("[INFO] 'face' 명령 감지 → move_to_face 실행")
                self.move_to_face()
                return

            for target in target_list:
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    self.pick_and_place_target(target_pos)
        else:
            self.get_logger().warn(f"{get_keyword_future.result().message}")

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                return None

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
            return target_pos
        return None

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def move_to_face(self):
        self.received = False
        self.get_logger().info("[INFO] 얼굴 좌표를 기다리는 중...")

        timeout_counter = 0
        while rclpy.ok() and not self.received:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_counter += 1
            if timeout_counter > 100:  # 10초 이상 기다렸다면 중단
                self.get_logger().error("[ERROR] 얼굴 좌표를 수신하지 못했습니다.")
                return

        pos = posx([
            self.xy_coords[0],  # x
            -252.0,             # y 고정
            540,                # z 고정
            91, -88.00, 90.09   # 오리엔테이션 고정
        ])
        self.get_logger().info("→ 얼굴 좌표로 이동")
        movel(pos, vel=VELOCITY, acc=ACC)


def main(args=None):
    node = RobotController()
    try:
        while rclpy.ok():
            node.robot_control()
    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료 (Ctrl+C).")
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
