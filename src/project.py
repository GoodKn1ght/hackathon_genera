#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2 as cv

# Імпортуємо модулі з поточного каталогу
from led_detector import LEDDetector
from pid import PIDController

class DroneFollowerNode(Node):
    def __init__(self):
        super().__init__('drone_follower_node')

        # Читаємо параметр ID поточного дрона
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value

        # Матриця цільових кольорів (копія з LedController.cc)
        color_matrix = {
            1: [1.0, 0.0, 0.0, 1.0],  # Дрон 1 шукає Червоного (0)
            2: [0.0, 1.0, 0.0, 1.0],  # Дрон 2 шукає Зеленого (1)
            3: [0.0, 0.0, 0.8, 1.0],  # Дрон 3 шукає Синього (2)
            4: [1.0, 1.0, 0.0, 1.0]   # Дрон 4 шукає Жовтого (3)
        }
        target_rgba = color_matrix.get(self.drone_id, [0.0, 1.0, 0.0, 1.0])

        # Ініціалізація імпортованого детектора
        self.detector = LEDDetector(rgba_color=target_rgba, min_area=30)
        self.bridge = CvBridge()

        # Геометрія камери та цілі
        self.cam_width = 640
        self.target_cx = self.cam_width / 2.0  
        self.target_area = 1500.0 

        # Ініціалізація двох імпортованих PID регуляторів
        self.pid_yaw = PIDController(kp=0.004, ki=0.0, kd=0.001, max_output=1.5, min_output=-1.5)
        self.pid_range = PIDController(kp=0.001, ki=0.0, kd=0.0002, max_output=2.0, min_output=-1.0)

        # Шлях до топіка Gazebo Harmonic
        camera_topic = (
            f'/world/baylands_custom/model/x500_mono_cam_{self.drone_id}'
            f'/link/mono_cam/base_link/sensor/camera_sensor/image'
        )
        self.img_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)

        # Налаштування топіків швидкості
        vel_topic = f'/model/x500_mono_cam_{self.drone_id}/cmd_vel'
        if self.drone_id == 4:
            vel_topic = f'/model/x500_{self.drone_id}/cmd_vel'
            
        self.vel_pub = self.create_publisher(Twist, vel_topic, 10)
        self.get_logger().info(f"=== Мозок Дрона {self.drone_id} успішно ініціалізовано ===")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Помилка CvBridge: {e}")
            return

        # Передаємо кадр у детектор
        cx, cy, area = self.detector.detect(cv_image, self.drone_id)

        twist_cmd = Twist()

        if cx is not None and cy is not None:
            # Обчислюємо вихідні значення через PID
            yaw_output = self.pid_yaw.update(current_value=cx, target_value=self.target_cx)
            range_output = self.pid_range.update(current_value=area, target_value=self.target_area)
            
            twist_cmd.angular.z = yaw_output
            twist_cmd.linear.x = range_output
        else:
            # Скидаємо PID, якщо втратили маяк із поля зору
            self.pid_yaw.clear()
            self.pid_range.clear()
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.0

        self.vel_pub.publish(twist_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DroneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()