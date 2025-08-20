from ultralytics import YOLO
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def remap(val, old_min, old_max, new_min, new_max):
    return new_min + ((val - old_min) / (old_max - old_min)) * (new_max - new_min)

class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('yolo_remap_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/remapped_coord', 10)

        # YOLO 모델 로드
        self.model = YOLO("face_best.pt")

        # 해상도 기준값
        self.ORIG_W, self.ORIG_H = 640, 480

        # 목표 좌표 범위
        self.TARGET_X_MIN, self.TARGET_X_MAX = 300, 600
        self.TARGET_Y_MIN, self.TARGET_Y_MAX = 330, 630

    def run(self):
        results = self.model.predict(source=0, stream=True, conf=0.1)
        for result in results:
            frame = result.orig_img.copy()
            boxes = result.boxes

            if boxes is not None and boxes.xyxy is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2

                    # 좌표 리매핑
                    mapped_x = remap(x_center, 0, self.ORIG_W, self.TARGET_X_MIN, self.TARGET_X_MAX)
                    mapped_y = remap(y_center, 0, self.ORIG_H, self.TARGET_Y_MIN, self.TARGET_Y_MAX)

                    # ROS 메시지 생성 및 퍼블리시
                    msg = Float64MultiArray()
                    msg.data = [mapped_x, mapped_y]
                    self.publisher_.publish(msg)

                    # 로그 출력 및 시각화
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    self.get_logger().info(f"[{cls_name}] 퍼블리시 좌표: ({mapped_x:.1f}, {mapped_y:.1f})")

                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"({int(mapped_x)},{int(mapped_y)})", (int(x1), int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                self.get_logger().warn("❌ 감지된 객체 없음")

            cv2.imshow("Remapped Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
