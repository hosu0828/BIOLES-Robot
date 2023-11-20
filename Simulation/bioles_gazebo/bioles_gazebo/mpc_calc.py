import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from casadi import *
# Import do_mpc package:
import do_mpc
import numpy as np
import sys


model_type = 'continuous'  # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)


# Parameter m, kg, s
# m0 = 700
# m1 = 400
# m2 = 400
# m3 = 2230
# l_11 = 0.11
# l_12 = 0.11
# l_21 = 0.16
# l_22 = 0.12
# t2 = 1.99
# t3 = 0.42
# g = 9.8
# r = 0.05
m0 = 135
m1 = 320.33
m2 = 130.44
m3 = 998.67
l_11 = 0.072
l_12 = 0.108
l_21 = 0.107
l_22 = 0.0776
t2 = 2.594781 # base angle : 1.77028746, min angle : 2.594781
t3 = 0.3583161 # base angle : 0.17767452, min angle : 0.3583161
g = 9.8
r = 0.05


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
    'n_horizon':5,
    'n_robust': 0,
    'open_loop': 0,
    't_step': 0.01,
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

        #imu filtering
        # self.imu_subscription = self.create_subscription(
        #     Float32MultiArray,
        #     'Filtered_imu',
        #     self.imu_callback,
        #     qos_profile_sensor_data
        # )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.mpc_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint',
            1
        )

        self.desired_yaw = 0.0
        self.wencoder_left = 0
        self.wencoder_right = 0
        self.m_l_pre = 0
        self.m_r_pre = 0
        self.m_l = 0
        self.m_r = 0

        self.count = 0
        self.check = True

    def imu_callback(self, value):

        output = Float64MultiArray()
        output.data = [0.0]*12

        roll = value.orientation.x
        yaw = value.orientation.z
        pitch = value.orientation.y
        # roll = value.data[0]
        # yaw = value.data[1]
        # pitch = value.data[2]

        mpc = do_mpc.controller.MPC(model)

        mpc.set_param(**setup_mpc)

        mterm = model.aux['E_kin'] - model.aux['E_pot']  # terminal cost
        lterm = model.aux['E_kin'] - model.aux['E_pot']  # stage cost

        mpc.set_objective(mterm=mterm, lterm=lterm)
        # Input force is implicitly restricted through the objective.
        mpc.set_rterm(force=0.008) # base pose : 0.002, min pose : 0.008

        mpc.bounds['lower', '_u', 'force'] = -50
        mpc.bounds['upper', '_u', 'force'] = 50

        mpc.setup()

        estimator = do_mpc.estimator.StateFeedback(model)

        simulator = do_mpc.simulator.Simulator(model)

        params_simulator = {
            # Note: cvode doesn't support DAE systems.
            'integration_tool': 'idas',
            'abstol': 1e-10,
            'reltol': 1e-10,
            't_step': 0.01
        }

        simulator.set_param(**params_simulator)

        simulator.setup()

        simulator.x0['t_imu'] = roll + 0.6  # base pose: 0.955, min pose : 0.57

        x0 = simulator.x0.cat.full()

        mpc.x0 = x0
        estimator.x0 = x0

        mpc.set_initial_guess()

        self.u0 = mpc.make_step(x0)

        # Quickly reset the history of the MPC data object.
        mpc.reset_history()

        # yaw applying
        u = 50*(yaw - self.desired_yaw)  # base pose : 100, min pose : 50
        m_l = self.u0 + u
        m_r = self.u0 - u

        # distance applying --> 0.05 is time
        # m_l = m_l - (0.05*self.u0)
        # m_r = m_r - (0.05*self.u0) 다시 고치기

        # balancing applying
        left_wheel_error = m_l - self.m_l_pre
        right_wheel_error = m_r - self.m_r_pre

        self.m_l = m_l + left_wheel_error
        self.m_r = m_r + right_wheel_error

        if self.m_l >= 50:
            self.m_l = 50
        elif self.m_l <= -50:
            self.m_l = -50

        if self.m_r >= 50:
            self.m_r = 50
        elif self.m_r <= -50:
            self.m_r = -50

        # 조인트 설정값 입력
        output.data[0] = float(self.m_l)
        output.data[1] = float(-1*self.m_r)
        # output.data[2] = 12.4
        # output.data[3] = 12.4
        output.data[4] = 0.0
        output.data[5] = 0.0
        # output.data[6] = 0.0
        output.data[7] = 1.57
        output.data[8] = 0.0
        # output.data[9] = 0.0
        output.data[10] = -1.57
        output.data[11] = 0.0

        # arm1 control
        value = float((roll-(-0.12))*(0.75-(-0.75))/(0.06-(-0.12))+(-0.75))
        output.data[6] = value
        output.data[9] = -1*value

        # 테스트할때!! 조인트 몇개 각도에 맞게 움직이기
        if self.check == True:
            self.count = self.count + 0.01
        else:
            self.count = self.count - 0.01

        output.data[2] = self.count
        output.data[3] = -1*self.count

        if self.count >= 16.0:
            self.check = False
        elif self.count <= 0:
            self.check = True

        # 세바퀴(1080도) 18.84956
        # 한바퀴(360도) 6.28319

        self.mpc_publisher.publish(output)
        self.get_logger().info('left : %f, right : %f' %(output.data[0], output.data[1]))

        self.m_l_pre = m_l
        self.m_r_pre = m_r


def main(args=None):
    rclpy.init(args=args)
    node = MPC()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()