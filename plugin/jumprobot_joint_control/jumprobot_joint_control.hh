#ifndef GAZEBO_JUMPROBOT_JOINT_CONTROL_HH
#define GAZEBO_JUMPROBOT_JOINT_CONTROL_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace gazebo{
    class JointControlPlugin : public ModelPlugin{
        public:
            JointControlPlugin();
            virtual ~JointControlPlugin();

            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        private:
            virtual void OnUpdate();
            virtual void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

            physics::ModelPtr model;
            physics::JointPtr l_wheel;
            physics::JointPtr r_wheel;
            physics::JointPtr l_cam;
            physics::JointPtr r_cam;
            physics::JointPtr l_waist;
            physics::JointPtr r_waist;
            physics::JointPtr l_arm1;
            physics::JointPtr l_arm2;
            physics::JointPtr l_arm3;
            physics::JointPtr r_arm1;
            physics::JointPtr r_arm2;
            physics::JointPtr r_arm3;
            common::PID wheel_pid;
            common::PID cam_pid;
            common::PID waist_pid;
            common::PID arm_pid;

            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
            rclcpp::Node::SharedPtr node_ptr_;

            event::ConnectionPtr updateConnection;
            rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    };
}
#endif