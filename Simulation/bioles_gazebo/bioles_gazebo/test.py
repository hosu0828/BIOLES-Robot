import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Test(Node):
    def __init__(self):
        super().__init__('test')

        self.test_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint',
            1
        )
        self.test_timer = self.create_timer(0.01, self.test_callback)

        self.count = 0
        self.check = True

    def test_callback(self):
        output = Float64MultiArray()
        output.data = [0.0]*12

        output.data[0] = 0.0
        output.data[1] = 0.0
        # output.data[2] = 0.0
        # output.data[3] = 0.0
        output.data[4] = 0.0
        output.data[5] = 0.0
        output.data[6] = 0.0
        output.data[7] = 0.0
        output.data[8] = 0.0
        output.data[9] = 0.0
        output.data[10] = 0.0
        output.data[11] = 0.0

        if self.check == True:
            self.count = self.count + 0.01
        else:
            self.count = self.count - 0.01

        output.data[2] = self.count
        output.data[3] = -1*self.count

        if self.count >= 18.9:
            self.check = False
        elif self.count <= 0:
            self.check = True

        # 세바퀴(1080도) 18.84956
        # 한바퀴(360도) 6.28319

        self.test_publisher.publish(output)
        self.get_logger().info('ok %f' %(output.data[2]))

def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()