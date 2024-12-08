import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import math

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每 0.1 秒查询一次

    def timer_callback(self):
        try:
            # 指定源坐标系和目标坐标系
            target_frame = 'world'
            source_frame = 'camera_color_frame'

            # 获取两坐标系之间的相对变换
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(seconds=0)  # 查询最新的变换
            )

            # 打印位置信息
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            self.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
            self.get_logger().info(f"Rotation (quaternion): x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

            # 可选：将四元数转换为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)
            self.get_logger().info(f"Rotation (euler): roll={roll}, pitch={pitch}, yaw={yaw}")

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {str(e)}")

    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
