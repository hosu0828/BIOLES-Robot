import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bioles_interfaces.msg import Imu
from bioles_interfaces.msg import Encoder
from bioles_interfaces.msg import Actuator
import serial
import time
import sys

# 시리얼 포트 설정
ser = serial.Serial(
    '/dev/ttyACM1',
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
lwaist = 95
larm_1 = 100
larm_2 = 180
larm_3 = 90





class Arduino_pub(Node):
    def __init__(self):
        super().__init__('Left_sensor')

        self.imu_timer = self.create_timer(0.00001, self.imu_timer_callback)
        self.imu_pub = self.create_publisher(
            Imu,
            'IMU_raw',
            qos_profile_sensor_data
        )
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

        self.identify = False     # 데이터 형식이 잘 들어왔는지 확인하는 bool형 변수.
        self.lwheel_deg = 0
        self.lcam_deg = 0

    def imu_timer_callback(self):
        val = Imu()
        self.line = ser.readline().decode('utf-8').rstrip()

        if self.line[89:90] == '@':        # Serial로 들어온 buffer data가 형식에 잘 맞춰서 왔을 경우 변수에 지정하기 위함.

            self.identify = True

            ax = self.line[1:9]     #데이터 있는 요소만 추출 ex) 첫번째 char부터 8번째 char까지 가져와서 하나의 string 변수로 설정.
            ay = self.line[9:17]
            az = self.line[17:25]

            gx = self.line[25:33]     #데이터 있는 요소만 추출
            gy = self.line[33:41]
            gz = self.line[41:49]

            mx = self.line[49:57]     #데이터 있는 요소만 추출
            my = self.line[57:65]
            mz = self.line[65:73]

            val.ax = float(ax)     # ros2 interface 형식을 맞춰주기 위해 string --> float로 변경.
            val.ay = float(ay)
            val.az = float(az)
            val.gx = float(gx)
            val.gy = float(gy)
            val.gz = float(gz)
            val.mx = float(mx)
            val.my = float(my)
            val.mz = float(mz)
            self.imu_pub.publish(val)     # interface를 통해 data 전송(topic)

            #self.get_logger().info('ax: %8.3f, ay: %8.3f, az: %8.3f, gx: %8.3f, gy: %8.3f, gz: %8.3f, mx: %8.3f, my: %8.3f, mz: %8.3f' %(val.ax,val.ay,val.az,val.gx,val.gy,val.gz,val.mx,val.my,val.mz))


    def encoder_timer_callback(self):
        deg = Encoder()

        if self.identify == True:
            self.lwheel_deg = self.line[73:81]
            self.lcam_deg = self.line[81:89]
            self.identify = False

        deg.left_wheel = float(self.lwheel_deg)
        deg.left_cam = float(self.lcam_deg)
        self.encoder_pub.publish(deg)

        #self.get_logger().info('lwheel deg : %f, lcam deg : %f' %(deg.left_wheel,deg.left_cam))



    def motor_callback(self,motor):
        left_wheel = int(motor.left_wheel)
        left_wheel = '{0:>4}'.format(str(left_wheel))     # 아두이노로 데이터 보낼때 주어진 자리수에 맞추기 위함.
        left_cam = int(motor.left_cam)
        left_cam = '{0:>4}'.format(str(left_cam))
        larm_1_output = int(motor.left_arm1)
        larm_1_output = '{0:>3}'.format(str(larm_1_output))
        larm_2_output = int(motor.left_arm2)
        larm_2_output = '{0:>3}'.format(str(larm_2_output))
        larm_3_output = int(motor.left_arm3)
        larm_3_output = '{0:>3}'.format(str(larm_3_output))

        # 이 부분은 나중에 정확한 계산값 후에 적용하면 됨. 우선은 default로 정해진 값만 송출.
        lwaist_output = '{0:>3}'.format(str(lwaist))



        output = '#' + left_wheel + left_cam + lwaist_output + larm_1_output + larm_2_output + larm_3_output + '$'

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
