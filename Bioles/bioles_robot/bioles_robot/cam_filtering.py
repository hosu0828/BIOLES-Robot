import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bioles_interfaces.msg import Encoder
import numpy as np
from scipy.signal import savgol_filter

alpha = 0.3
window = 10
left_sum = 0
right_sum = 0

class Cam_filter(Node):
    def __init__(self):
        super().__init__('filtered_cam')

        self.cam_subscription = self.create_subscription(
            Encoder,
            'Cam_deg',
            self.cam_callback,
            qos_profile_sensor_data
        )

        self.filtered_cam_publisher = self.create_publisher(
            Encoder,
            'Filtered_cam',
            qos_profile_sensor_data
        )

        self.left_past = 0
        self.right_past = 0
        self.left_data = [0]*(window-1)
        self.right_data = [0]*(window-1)

    '''
    # s-g filter
    def cam_callback(self, value):

        self.left_data.append(value.left)
        self.right_data.append(value.right)

        left_sg = savgol_filter(self.left_data, window, 2)
        right_sg = savgol_filter(self.right_data, window, 2)

        value.filtered_left = left_sg[len(left_sg)-1]
        value.filtered_right = right_sg[len(right_sg)-1]

        self.filtered_cam_publisher.publish(value)

        self.left_data.pop(0)
        self.right_data.pop(0)

        self.get_logger().info('filt_left : %f, filt_right : %f' %(value.filtered_left,value.filtered_right))
    '''

    '''
    # moving average filter
    def cam_callback(self, value):
        global left_sum, right_sum

        self.left_data.append(value.left)
        self.right_data.append(value.right)

        for i in range(0,window,1):
            left_sum = left_sum + self.left_data[i]
            right_sum = right_sum + self.right_data[i]

        left_avg = left_sum/window
        right_avg = right_sum/window

        value.filtered_left = left_avg
        value.filtered_right = right_avg
        self.filtered_cam_publisher.publish(value)

        self.left_data.pop(0)
        self.right_data.pop(0)
        left_sum = 0
        right_sum = 0

        self.get_logger().info('filt_left : %f, filt_right : %f' %(value.filtered_left,value.filtered_right))
    '''
    def cam_callback(self, value):
        global left_sum, right_sum

        self.left_data.append(value.left)
        self.right_data.append(value.right)

        for i in range(0,window,1):
            left_sum = left_sum + self.left_data[i]
            right_sum = right_sum + self.right_data[i]

        left_avg = left_sum/window
        right_avg = right_sum/window

        value.filtered_left = left_avg
        value.filtered_right = right_avg
        self.filtered_cam_publisher.publish(value)

        self.left_data.pop(0)
        self.right_data.pop(0)
        left_sum = 0
        right_sum = 0
        
        self.get_logger().info('filt_left : %f, filt_right : %f' %(value.filtered_left,value.filtered_right))

def main(args=None):
    rclpy.init(args=args)
    node = Cam_filter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





