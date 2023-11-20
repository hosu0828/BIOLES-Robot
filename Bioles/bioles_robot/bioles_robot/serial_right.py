import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bioles_interfaces.msg import Encoder
from bioles_interfaces.msg import Actuator
import serial
import time
import sys

# 시리얼 포트 설정
ser = serial.Serial(
    '/dev/ttyACM0',
    baudrate = 500000,
    bytesize = serial.EIGHTBITS,
    timeout=0.5
)

# 바로 읽어버리면 오류가 있어서 몇번정도 미리 읽어서 오류 수정해주기.
# 이렇게 해도 안되면 다른 파이썬 파일로 읽게 한다음 실행하기!!!
ser.readline().decode('utf-8').rstrip()
ser.readline().decode('utf-8').rstrip()
ser.readline().decode('utf-8').rstrip()
ser.readline().decode('utf-8').rstrip()
ser.readline().decode('utf-8').rstrip()
ser.readline().decode('utf-8').rstrip()


# 아직 제어 값이 정해지지 않아서 임의로 default값 정했음. 나중에 수정하기.
rwaist = 95
rarm_1 = 105
rarm_2 = 35
rarm_3 = 97





class Arduino_pub(Node):
    def __init__(self):
        super().__init__('Right_sensor')

        self.encoder_timer = self.create_timer(0.00001, self.encoder_timer_callback)
        self.encoder_pub = self.create_publisher(
            Encoder,
            'wheel_cam_deg',
            qos_profile_sensor_data
        )
        self.mpc_sub = self.create_subscription(
            Actuator,
            'mpc_output',
            self.motor_callback,
            qos_profile_sensor_data
        )

        self.rwheel_deg = 0
        self.rcam_deg = 0


    def encoder_timer_callback(self):
        deg = Encoder()
        self.line = ser.readline().decode('utf-8').rstrip()

        if self.line[17:18] == '@':
            self.rwheel_deg = self.line[1:9]
            self.rcam_deg = self.line[9:17]

        deg.right_wheel = float(self.rwheel_deg)
        deg.right_cam = float(self.rcam_deg)
        self.encoder_pub.publish(deg)

        #self.get_logger().info('rwheel deg : %f, rcam deg : %f' %(deg.right_wheel,deg.right_cam))


    def motor_callback(self,motor):
        right_wheel = int(motor.right_wheel)
        right_wheel = '{0:>4}'.format(str(right_wheel))     # 아두이노로 데이터 보낼때 주어진 자리수에 맞추기 위함.
        right_cam = int(motor.right_cam)
        right_cam = '{0:>4}'.format(str(right_cam))
        rarm_1_output = int(motor.right_arm1)
        rarm_1_output = '{0:>3}'.format(str(rarm_1_output))
        rarm_2_output = int(motor.right_arm2)
        rarm_2_output = '{0:>3}'.format(str(rarm_2_output))
        rarm_3_output = int(motor.right_arm3)
        rarm_3_output = '{0:>3}'.format(str(rarm_3_output))


        # 이 부분은 나중에 정확한 계산값 후에 적용하면 됨. 우선은 default로 정해진 값만 송출.
        rwaist_output = '{0:>3}'.format(str(rwaist))

        output = '#' + right_wheel + right_cam + rwaist_output + rarm_1_output + rarm_2_output + rarm_3_output + '$'

        ser.write(output.encode('utf-8'))     # 아두이노로 data 전송.

        self.get_logger().info(output)



def main(args=None):
    rclpy.init(args=args)
    node = Arduino_pub()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
