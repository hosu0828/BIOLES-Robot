#include "jumprobot_joint_control.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JointControlPlugin)

JointControlPlugin::JointControlPlugin() : ModelPlugin(){}

JointControlPlugin::~JointControlPlugin(){}

void JointControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    this->model = _model;

    // this->wheel_pid = common::PID(0.3,0,0);
    // this->cam_pid = common::PID(1.0,0,0);
    // this->waist_pid = common::PID(1.0,0,0);
    // this->arm_pid = common::PID(2.0,0,0.1);
    this->wheel_pid = common::PID(0.3,0,0);
    this->cam_pid = common::PID(0.3,0,0);
    this->waist_pid = common::PID(1.0,0,0);
    this->arm_pid = common::PID(2.0,0,0.1);


    if(_sdf->HasElement("l_wheel")){
        this->l_wheel = _model->GetJoint(_sdf->GetElement("l_wheel")->Get<std::string>());
        std::cout<< "left_wheel : " << this->l_wheel->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_wheel")){
        this->r_wheel = _model->GetJoint(_sdf->GetElement("r_wheel")->Get<std::string>());
        std::cout<< "right_wheel : " << this->r_wheel->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("l_cam")){
        this->l_cam = _model->GetJoint(_sdf->GetElement("l_cam")->Get<std::string>());
        std::cout<< "left_cam : " << this->l_cam->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_cam")){
        this->r_cam = _model->GetJoint(_sdf->GetElement("r_cam")->Get<std::string>());
        std::cout<< "right_cam : " << this->r_cam->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("l_waist")){
        this->l_waist = _model->GetJoint(_sdf->GetElement("l_waist")->Get<std::string>());
        std::cout<< "left_waist : " << this->l_waist->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_waist")){
        this->r_waist = _model->GetJoint(_sdf->GetElement("r_waist")->Get<std::string>());
        std::cout<< "right_waist : " << this->r_waist->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("l_arm1")){
        this->l_arm1 = _model->GetJoint(_sdf->GetElement("l_arm1")->Get<std::string>());
        std::cout<< "left_arm1 : " << this->l_arm1->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("l_arm2")){
        this->l_arm2 = _model->GetJoint(_sdf->GetElement("l_arm2")->Get<std::string>());
        std::cout<< "left_arm2 : " << this->l_arm2->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("l_arm3")){
        this->l_arm3 = _model->GetJoint(_sdf->GetElement("l_arm3")->Get<std::string>());
        std::cout<< "left_arm3 : " << this->l_arm3->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_arm1")){
        this->r_arm1 = _model->GetJoint(_sdf->GetElement("r_arm1")->Get<std::string>());
        std::cout<< "right_arm1 : " << this->r_arm1->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_arm2")){
        this->r_arm2 = _model->GetJoint(_sdf->GetElement("r_arm2")->Get<std::string>());
        std::cout<< "right_arm2 : " << this->r_arm2->GetScopedName() << std::endl;
    }
    if(_sdf->HasElement("r_arm3")){
        this->r_arm3 = _model->GetJoint(_sdf->GetElement("r_arm3")->Get<std::string>());
        std::cout<< "right_arm3 : " << this->r_arm3->GetScopedName() << std::endl;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();

    this->node_ptr_ = rclcpp::Node::make_shared("joint_control_node");
    this->sub_ = this->node_ptr_->create_subscription<std_msgs::msg::Float64MultiArray>("/joint", qos, std::bind(&JointControlPlugin::topic_callback, this, std::placeholders::_1));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointControlPlugin::OnUpdate, this));
    this->executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
    this->executor_->add_node(this->node_ptr_);
}

void JointControlPlugin::OnUpdate(){
    this->executor_->spin_once(std::chrono::nanoseconds(1));
    // std::cout << "onupdate" << std::endl;
}

void JointControlPlugin::topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    // std::cout << "callback" << std::endl;
    this->model->GetJointController()->SetVelocityTarget(this->l_wheel->GetScopedName(), msg->data[0]);
    this->model->GetJointController()->SetVelocityPID(this->l_wheel->GetScopedName(), this->wheel_pid);
    this->model->GetJointController()->SetVelocityTarget(this->r_wheel->GetScopedName(), msg->data[1]);
    this->model->GetJointController()->SetVelocityPID(this->r_wheel->GetScopedName(), this->wheel_pid);
    this->model->GetJointController()->SetPositionTarget(this->l_cam->GetScopedName(), msg->data[2]);
    this->model->GetJointController()->SetPositionPID(this->l_cam->GetScopedName(), this->cam_pid);
    this->model->GetJointController()->SetPositionTarget(this->r_cam->GetScopedName(), msg->data[3]);
    this->model->GetJointController()->SetPositionPID(this->r_cam->GetScopedName(), this->cam_pid);
    this->model->GetJointController()->SetPositionTarget(this->l_waist->GetScopedName(), msg->data[4]);
    this->model->GetJointController()->SetPositionPID(this->l_waist->GetScopedName(), this->waist_pid);
    this->model->GetJointController()->SetPositionTarget(this->r_waist->GetScopedName(), msg->data[5]);
    this->model->GetJointController()->SetPositionPID(this->r_waist->GetScopedName(), this->waist_pid);
    this->model->GetJointController()->SetPositionTarget(this->l_arm1->GetScopedName(), msg->data[6]);
    this->model->GetJointController()->SetPositionPID(this->l_arm1->GetScopedName(), this->arm_pid);
    this->model->GetJointController()->SetPositionTarget(this->l_arm2->GetScopedName(), msg->data[7]);
    this->model->GetJointController()->SetPositionPID(this->l_arm2->GetScopedName(), this->arm_pid);
    this->model->GetJointController()->SetPositionTarget(this->l_arm3->GetScopedName(), msg->data[8]);
    this->model->GetJointController()->SetPositionPID(this->l_arm3->GetScopedName(), this->arm_pid);
    this->model->GetJointController()->SetPositionTarget(this->r_arm1->GetScopedName(), msg->data[9]);
    this->model->GetJointController()->SetPositionPID(this->r_arm1->GetScopedName(), this->arm_pid);
    this->model->GetJointController()->SetPositionTarget(this->r_arm2->GetScopedName(), msg->data[10]);
    this->model->GetJointController()->SetPositionPID(this->r_arm2->GetScopedName(), this->arm_pid);
    this->model->GetJointController()->SetPositionTarget(this->r_arm3->GetScopedName(), msg->data[11]);
    this->model->GetJointController()->SetPositionPID(this->r_arm3->GetScopedName(), this->arm_pid);
}