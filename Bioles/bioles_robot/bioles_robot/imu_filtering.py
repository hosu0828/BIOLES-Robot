import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bioles_interfaces.msg import Imu
import numpy as np
from numpy.linalg import inv
from scipy import io
import math

# Initialization for system model.
# Matrix: A, H, Q, R, P_0
# Vector: x_0
A = np.zeros((3, 3))
H = np.eye(3)
Q = np.array([[0.0001, 0, 0],
              [0, 0.0001, 0],
              [0, 0, 0.001]])
R = 0.005 * np.eye(3)

# Initialization for estimation.
x_0 = np.zeros(3)  # (phi, the, psi) by my definition.
P_0 = 0.001 * np.eye(3)

time_step = 0.005

class Imu_filter(Node):
    def __init__(self):
        super().__init__('filtered_imu')

        self.imu_subscription = self.create_subscription(
            Imu,
            'IMU_raw',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.filtered_imu_publisher = self.create_publisher(
            Imu,
            'Filtered_imu',
            qos_profile_sensor_data
        )

        self.phi, self.the, self.psi = 0, 0, 0
        self.x_esti, self.P = x_0, P_0
        self.phi = 0
        self.the = 0
        self.psi = 0
        self.gx = 0
        self.gy = 0
        self.gz = 0

    def Ajacob_at(self):

        sinPhi = np.sin(self.phi)
        cosPhi = np.cos(self.phi)
        tanThe = np.tan(self.the)
        secThe = 1. / np.cos(self.the)

        A = np.zeros((3, 3))

        A[0][0] = self.gy*cosPhi*tanThe - self.gz*sinPhi*tanThe
        A[0][1] = self.gy*sinPhi*secThe**2 + self.gz*cosPhi*secThe**2
        A[0][2] = 0

        A[1][0] = -self.gy*sinPhi - self.gz*cosPhi
        A[1][1] = 0
        A[1][2] = 0

        A[2][0] = self.gy*cosPhi*secThe - self.gz*sinPhi*secThe
        A[2][1] = self.gy*sinPhi*secThe*tanThe + self.gz*cosPhi*secThe*tanThe
        A[2][2] = 0

        self.A = np.eye(3) + A * time_step
        return self.A

    def Hjacob_at(x_pred):
        return H

    def fx(self):

        sinPhi = np.sin(self.phi)
        cosPhi = np.cos(self.phi)
        tanThe = np.tan(self.the)
        secThe = 1. / np.cos(self.the)

        xdot = np.zeros(3)
        xdot[0] = self.gx + self.gy*sinPhi*tanThe + self.gz*cosPhi*tanThe
        xdot[1] = self.gy*cosPhi - self.gz*sinPhi
        xdot[2] = self.gy*sinPhi*secThe + self.gz*cosPhi*secThe

        self.x_pred = self.x_esti + xdot*time_step
        return self.x_pred


    def extended_kalman_filter(self):
        """Extended Kalman Filter Algorithm."""
        # (1) Prediction.
        A = self.Ajacob_at()
        x_pred = self.fx()
        P_pred = A @ self.P @ A.T + Q

        # (2) Kalman Gain.
        K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)

        # (3) Estimation.
        x_esti = x_pred + K @ (self.z_meas - H @ x_pred)

        # (4) Error Covariance.
        P = P_pred - K @ H @ P_pred

        return x_esti, P



    def imu_callback(self, value):

        ax = value.ax
        ay = value.ay
        az = value.az
        gx = value.gx
        gy = value.gy
        gz = value.gz
        mx = value.mx
        my = value.my
        mz = value.mz

        pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
        roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        Yh = my*math.cos(roll) - mz*math.sin(roll)
        Xh = mx*math.cos(pitch) + my*math.sin(roll) * \
            math.sin(pitch) + mz*math.cos(roll)*math.sin(pitch)

        yaw = math.atan2(Yh, Xh)

        self.phi = roll
        self.the = yaw
        self.psi = pitch

        self.z_meas = np.array([self.phi, self.the, self.psi])
        self.x_esti, self.P = self.extended_kalman_filter()


        value.roll = float(self.x_esti[0])
        value.yaw = float(self.x_esti[1])
        value.pitch = float(self.x_esti[2])

        self.filtered_imu_publisher.publish(value)

        self.get_logger().info('roll : %f, pitch : %f, yaw : %f' %(value.roll,value.pitch, value.yaw))


def main(args=None):
    rclpy.init(args=args)
    node = Imu_filter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
