import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bioles_interfaces.msg import Imu
from bioles_interfaces.msg import Encoder
from bioles_interfaces.msg import Actuator
from casadi import *
# Import do_mpc package:
import do_mpc
import numpy as np
import sys
import math
import time

for i in range (1,10,1):
    time.sleep(1)
    print(i)

model_type = 'continuous'  # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)


# Parameter m, kg, s
m0 = 700
m1 = 400
m2 = 400
m3 = 2230
l_11 = 0.11
l_12 = 0.11
l_21 = 0.16
l_22 = 0.12
t2 = 1.99
t3 = 0.42
g = 9.8
r = 0.05

# Output boundary
boundary = 255

# arm filtering parameter
window = 10
left_sum = 0
right_sum = 0

# wheel filtering parameter
wheel_window = 5
wheel_left_sum = 0
wheel_right_sum = 0


t_w = model.set_variable('_x',  't_w')
t_imu = model.set_variable('_x',  't_imu')
dt_w = model.set_variable('_x',  'dt_w')
dt_imu = model.set_variable('_x',  'dt_imu')
ddt_w = model.set_variable('_z', 'ddt_w')
ddt_imu = model.set_variable('_z', 'ddt_imu')

u = model.set_variable('_u',  'force')

model.set_rhs('t_w', dt_w)
model.set_rhs('t_imu', dt_imu)
model.set_rhs('dt_w', ddt_w)
model.set_rhs('dt_imu', ddt_imu)

euler_lagrange = vertcat(
    # t_w
    u + (l_22*m3*r*sin(t2 + t3)*ddt_imu)/2 + l_21*m2*r*sin(t2)*ddt_imu +
    l_21*m3*r*sin(t2)*ddt_imu + l_11*m1*r*t_imu*ddt_imu + l_11*m2*r*t_imu*ddt_imu +
    l_11*m3*r*t_imu*ddt_imu + l_12*m2*r*t_imu*ddt_imu +
    l_12*m3*r*t_imu*ddt_imu + (l_22*m3*r*sin(t2 - t3)*ddt_imu)/2 +
    l_21*m2*r*cos(t2)*t_imu*ddt_imu + l_21*m3*r*cos(t2)*t_imu*ddt_imu +
    l_22*m3*r*cos(t2 - t3)*t_imu*ddt_imu - m0*r**2*ddt_w -
    m1*r**2*ddt_w - m2*r**2*ddt_w - m3*r**2 * \
    ddt_w - l_22*m3*r*cos(t2)*sin(t3)*ddt_imu,

    # t_imu
    g*l_11*m1 + g*l_11*m2 + g*l_11*m3 + g*l_12*m2 + g*l_12*m3 + l_11**2*m1*ddt_imu +
    l_11**2*m2*ddt_imu + l_11**2*m3*ddt_imu + l_12**2*m2*ddt_imu + l_12**2*m3*ddt_imu +
    l_21**2*m2*ddt_imu + l_21**2*m3*ddt_imu + l_22**2*m3*ddt_imu + l_11**2*m1*t_imu**2*ddt_imu +
    l_11**2*m2*t_imu**2*ddt_imu + l_11**2*m3*t_imu**2*ddt_imu + l_12**2*m2*t_imu**2*ddt_imu +
    l_12**2*m3*t_imu**2*ddt_imu + l_21**2*m2*t_imu**2*ddt_imu + l_21**2*m3*t_imu**2*ddt_imu +
    l_22**2*m3*t_imu**2*ddt_imu + g*l_21*m2*cos(t2) + g*l_21*m3*cos(t2) + g*l_22*m3*cos(t2 - t3) +
    2*l_11*l_12*m2*ddt_imu + 2*l_11*l_12*m3*ddt_imu + 2*l_11*l_12*m2*t_imu**2*ddt_imu +
    2*l_11*l_12*m3*t_imu**2*ddt_imu + 2*l_11*l_21*m2*cos(t2)*ddt_imu + 2*l_11*l_21*m3*cos(t2)*ddt_imu +
    2*l_12*l_21*m2*cos(t2)*ddt_imu + 2*l_12*l_21*m3*cos(t2)*ddt_imu + 2*l_21*l_22*m3*cos(t3)*ddt_imu +
    2*l_11*l_22*m3*cos(t2 - t3)*ddt_imu + 2*l_12*l_22*m3*cos(t2 - t3)*ddt_imu +
    2*l_11*l_22*m3*cos(t2 - t3)*t_imu**2*ddt_imu + 2*l_12*l_22*m3*cos(t2 - t3)*t_imu**2*ddt_imu +
    2*l_11*l_21*m2*cos(t2)*t_imu**2*ddt_imu + 2*l_11*l_21*m3*cos(t2)*t_imu**2*ddt_imu +
    2*l_12*l_21*m2*cos(t2)*t_imu**2*ddt_imu + 2*l_12*l_21*m3*cos(t2)*t_imu**2*ddt_imu +
    2*l_21*l_22*m3*cos(t3)*t_imu**2*ddt_imu - g*l_21*m2*sin(t2)*t_imu - g*l_21*m3*sin(t2)*t_imu -
    g*l_22*m3*sin(t2 - t3)*t_imu - l_21*m2*r*sin(t2)*ddt_w - l_21*m3*r*sin(t2)*ddt_w -
    l_11*m1*r*t_imu*ddt_w - l_11*m2*r*t_imu*ddt_w - l_11*m3*r*t_imu*ddt_w - l_12*m2*r*t_imu*ddt_w -
    l_12*m3*r*t_imu*ddt_w - l_22*m3*r*sin(t2 - t3)*ddt_w - l_21*m2*r*cos(t2)*t_imu*ddt_w -
    l_21*m3*r*cos(t2)*t_imu*ddt_w - l_22*m3*r*cos(t2 - t3)*t_imu*ddt_w
)

model.set_alg('euler_lagrange', euler_lagrange)


E_kin = (m3*((l_22*sin(t2 - t3 + t_imu)*dt_imu - r*dt_w + sin(t_imu)*(l_11 + l_12)*dt_imu + l_21*sin(t2 + t_imu)*dt_imu)**2 +
             (l_22*cos(t2 - t3 + t_imu)*dt_imu + cos(t_imu)*(l_11 + l_12)*dt_imu + l_21*cos(t2 + t_imu)*dt_imu)**2))/2 + (m1*((r*dt_w - l_11*sin(t_imu)*dt_imu)**2 + l_11**2*cos(t_imu)**2*dt_imu**2))/2 + (m2*((cos(t_imu)*(l_11 + l_12)*dt_imu +
                                                                                                                                                                                                                 l_21*cos(t2 + t_imu)*dt_imu)**2 + (sin(t_imu)*(l_11 + l_12)*dt_imu - r*dt_w + l_21*sin(t2 + t_imu)*dt_imu)**2))/2 + (m0*r**2*dt_w**2)/2

E_pot = g*(m2*(sin(t_imu)*(l_11 + l_12) + l_21*sin(t2 + t_imu)) + m3*(l_22*sin(t2 - t3 + t_imu) +
                                                                      sin(t_imu)*(l_11 + l_12) + l_21*sin(t2 + t_imu)) + l_11*m1*sin(t_imu))

model.set_expression('E_kin', E_kin)
model.set_expression('E_pot', E_pot)

# Build the model
model.setup()


setup_mpc = {
    'n_horizon':7,
    'n_robust': 0,
    'open_loop': 0,
    't_step': 0.045,
    'state_discretization': 'collocation',
    'collocation_type': 'radau',
    'collocation_deg': 2,
    'collocation_ni': 1,
    'store_full_solution': True,
    # Use MA27 linear solver in ipopt for faster calculations:
    'nlpsol_opts': {'ipopt.linear_solver': 'mumps'}
}


class MPC(Node):
    def __init__(self):
        super().__init__('mpc')

        self.imu_subscription = self.create_subscription(
            Imu,
            'Filtered_imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.imu_subscription = self.create_subscription(
            Encoder,
            'wheel_cam_deg',
            self.wheel_cam_control_callback,
            qos_profile_sensor_data
        )

        self.mpc_publisher = self.create_publisher(
            Actuator,
            'mpc_output',
            qos_profile_sensor_data
        )

        self.wencoder_left = 0
        self.wencoder_right = 0
        self.cencoder_left = 0
        self.cencoder_right = 0

        self.m_l_pre = 0
        self.m_r_pre = 0
        self.m_l = 0
        self.m_r = 0

        self.left_data = [0]*(window-1)
        self.right_data = [0]*(window-1)
        self.control_pitch = 0

        self.wheel_left_data = [0]*(wheel_window-1)
        self.wheel_right_data = [0]*(wheel_window-1)
        self.wheel_control = 0

        self.l_gap = 0
        self.r_gap = 0
        self.pre_lcam = 180
        self.pre_rcam = 180
        self.l_cam_count = 0
        self.r_cam_count = 0
        self.l_cam_output = -255
        self.r_cam_output = 255

    def imu_callback(self, value):

        global left_sum, right_sum, wheel_left_sum, wheel_right_sum

        roll = value.roll
        yaw = value.yaw
        pitch = value.pitch

        output = Actuator()

        mpc = do_mpc.controller.MPC(model)

        mpc.set_param(**setup_mpc)

        mterm = model.aux['E_kin'] - model.aux['E_pot']  # terminal cost
        lterm = model.aux['E_kin'] - model.aux['E_pot']  # stage cost

        mpc.set_objective(mterm=mterm, lterm=lterm)
        # Input force is implicitly restricted through the objective.
        mpc.set_rterm(force=0.5)

        mpc.bounds['lower', '_u', 'force'] = -boundary
        mpc.bounds['upper', '_u', 'force'] = boundary
        #mpc.bounds['lower', '_x', 't_imu'] = 0.48
        #mpc.bounds['upper', '_x', 't_imu'] = 0.88

        mpc.setup()

        estimator = do_mpc.estimator.StateFeedback(model)

        simulator = do_mpc.simulator.Simulator(model)

        params_simulator = {
            # Note: cvode doesn't support DAE systems.
            'integration_tool': 'idas',
            'abstol': 1e-10,
            'reltol': 1e-10,
            't_step': 0.045
        }

        simulator.set_param(**params_simulator)

        simulator.setup()

        simulator.x0['t_imu'] = pitch + 0.85  # default : +0.71

        x0 = simulator.x0.cat.full()

        mpc.x0 = x0
        estimator.x0 = x0

        mpc.set_initial_guess()

        self.u0 = mpc.make_step(x0)

        # Quickly reset the history of the MPC data object.
        mpc.reset_history()

        # balancing control
        m_l = self.u0 - 0*self.wencoder_left
        m_r = self.u0 - 0*self.wencoder_right

        left_wheel_error = m_l - self.m_l_pre
        right_wheel_error = m_r - self.m_r_pre

        self.m_l = m_l + left_wheel_error
        self.m_r = m_r + right_wheel_error

        if self.m_l >= boundary:
            self.m_l = boundary
        elif self.m_l <= (-1)*boundary:
            self.m_l = (-1)*boundary

        if self.m_r >= boundary:
            self.m_r = boundary
        elif self.m_r <= (-1)*boundary:
            self.m_r = (-1)*boundary

        self.wheel_left_data.append(self.m_l)
        self.wheel_right_data.append(self.m_r)

        for j in range(0,wheel_window,1):
            wheel_left_sum = wheel_left_sum + self.wheel_left_data[j]
            wheel_right_sum = wheel_right_sum + self.wheel_right_data[j]

        wheel_left_avg = wheel_left_sum/wheel_window
        wheel_right_avg = wheel_right_sum/wheel_window

        # arm control
        self.control_pitch = pitch
        if pitch>-0.01:
            self.control_pitch = -0.01
        elif pitch<-0.35:
            self.control_pitch = -0.35

        left_arm1 = (self.control_pitch-(-0.35))*(130-70)/(-0.01-(-0.35)) + 70
        right_arm1 = (self.control_pitch-(-0.35))*(75-135)/(-0.01-(-0.35)) + 135

        self.left_data.append(left_arm1)
        self.right_data.append(right_arm1)

        for i in range(0,window,1):
            left_sum = left_sum + self.left_data[i]
            right_sum = right_sum + self.right_data[i]

        left_avg = left_sum/window
        right_avg = right_sum/window

        output.left_wheel = float(wheel_left_avg)
        output.right_wheel = float(wheel_right_avg)
        output.left_cam = float(self.l_cam_output)
        output.right_cam = float(self.r_cam_output)
        output.left_arm1 = float(left_avg)
        output.left_arm2 = float(92)   # base:180, horizon: 92 
        output.left_arm3 = float(90)
        output.right_arm1 = float(right_avg)
        output.right_arm2 = float(90)   # base:35, horizon: 90
        output.right_arm3 = float(97)

        self.mpc_publisher.publish(output)
        self.get_logger().info('left : %f, right : %f' %(output.left_wheel, output.right_wheel))

        # error initial
        self.m_l_pre = m_l
        self.m_r_pre = m_r

        # wheel initial
        self.wheel_left_data.pop(0)
        self.wheel_right_data.pop(0)
        wheel_left_sum = 0
        wheel_right_sum = 0

        # arm initial
        self.left_data.pop(0)
        self.right_data.pop(0)
        left_sum = 0
        right_sum = 0

    def wheel_cam_control_callback(self, value):
        self.wencoder_left = value.left_wheel
        self.wencoder_right = value.right_wheel
        self.cencoder_left = value.left_cam
        self.cencoder_right = value.right_cam

        self.l_gap = self.pre_lcam - self.cencoder_left
        self.r_gap = self.pre_rcam - self.cencoder_right

        if self.l_gap > 200:
            self.l_cam_count = self.l_cam_count + 1
        if self.r_gap < -200:
            self.r_cam_count = self.r_cam_count + 1
        if self.l_cam_count >= 3 and self.cencoder_left >= 78:
            self.l_cam_output = 0
            #self.r_cam_output = 0
        if self.r_cam_count >= 4 and self.cencoder_right <= 308:
            self.r_cam_output = 0

        self.pre_lcam = self.cencoder_left
        self.pre_rcam = self.cencoder_right


def main(args=None):
    rclpy.init(args=args)
    node = MPC()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

