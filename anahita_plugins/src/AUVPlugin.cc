#include <anahita_plugins/AUVPlugin.hh>

using namespace std;
namespace gazebo
{
AUV::AUV() {
    std::cout << "AUV plugin started successfully" << std::endl;
}
AUV::~AUV() {}

void AUV::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model_ = _model;
    this->sdf_ = _sdf;

    this->baseLink = _model->GetLink("anahita::base_link");

    this->north_link_ = _model->GetLink("anahita::north");
    this->south_link_ = _model->GetLink("anahita::south");
    this->east_link_ = _model->GetLink("anahita::east");
    this->west_link_ = _model->GetLink("anahita::west");
    this->north_east_link_ = _model->GetLink("anahita::north_east");
    this->north_west_link_ = _model->GetLink("anahita::north_west");
    this->south_east_link_ = _model->GetLink("anahita::south_east");
    this->south_west_link_ = _model->GetLink("anahita::south_west");

    cout << "Model Name: " << _model->GetName() << endl;
    
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle("anahita"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/forwardLeft", 1,
        boost::bind(&AUV::ForwardLeftCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/forwardRight", 1,
        boost::bind(&AUV::ForwardRightCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/sidewardFront", 1,
        boost::bind(&AUV::SidewardFrontCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so4 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/sidewardBack", 1,
        boost::bind(&AUV::SidewardBackCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so5 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/upwardBack", 1,
        boost::bind(&AUV::UpwardSouthEastCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so6 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/upwardBack", 1,
        boost::bind(&AUV::UpwardSouthWestCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so7 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/upwardFront", 1,
        boost::bind(&AUV::UpwardNorthEastCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so8 = ros::SubscribeOptions::create<std_msgs::Int32>(
        "/pwm/upwardFront", 1,
        boost::bind(&AUV::UpwardNorthWestCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->west_sub_ = this->nh_->subscribe(so1);
    this->east_sub_ = this->nh_->subscribe(so2);
    this->north_sub_ = this->nh_->subscribe(so3);
    this->south_sub_ = this->nh_->subscribe(so4);
    this->south_east_sub_ = this->nh_->subscribe(so5);
    this->south_west_sub_ = this->nh_->subscribe(so6);
    this->north_east_sub_ = this->nh_->subscribe(so7);
    this->north_west_sub_ = this->nh_->subscribe(so8);

    this->rosQueueThread = std::thread(std::bind(&AUV::QueueThread, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));
}

double AUV::ThrustConversionFnc(int pwm) {
    int x = pwm;
    double A = -231.54;
    double B = 0.42;
    double C = -0.00027;
    double D = 0.0000000622;

    double f = A + B*x + C*x*x + D*x*x*x;
    return f; 
}

/// \brief ROS helper function that processes messages
void AUV::QueueThread()
{
  static const double timeout = 0.01;
  while (this->nh_->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void AUV::Update() {
    
    ignition::math::Vector3d buoyancyForce;
    buoyancyForce = ignition::math::Vector3d(0, 0, 0);

    double mass = 0;

    std::vector<gazebo::physics::LinkPtr> links_ = this->model_->GetLinks();

    double g = 9.8;

    ignition::math::Vector3d CoB = ignition::math::Vector3d(0, 0, 0.05);

    ignition::math::Vector3d f1(1, 0, 0);

    for (int i = 0; i < links_.size(); i++) {
        mass = mass + links_[i]->GetInertial()->Mass();
    }

    buoyancyForce = ignition::math::Vector3d(0, 0, mass*g);
    this->baseLink->AddForceAtRelativePosition(buoyancyForce, CoB);

    this->north_link_->AddRelativeForce(ignition::math::Vector3d(0, pwm_north_, 0)); 
    this->south_link_->AddRelativeForce(ignition::math::Vector3d(0, pwm_south_, 0));
    this->east_link_->AddRelativeForce(ignition::math::Vector3d(-pwm_east_, 0, 0));
    this->west_link_->AddRelativeForce(ignition::math::Vector3d(-pwm_west_, 0, 0));
    this->north_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_north_east_));
    this->north_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_north_west_));
    this->south_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_south_east_));
    this->south_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_south_west_));
}

void AUV::SidewardBackCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_ = (static_cast<double>(_msg->data)/400)*3;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::SidewardFrontCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_ = (static_cast<double>(_msg->data)/400)*3;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::ForwardLeftCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_west_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::ForwardRightCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_east_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::UpwardNorthEastCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_east_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::UpwardNorthWestCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_west_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::UpwardSouthEastCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_east_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

void AUV::UpwardSouthWestCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_west_ = (static_cast<double>(_msg->data)/400)*1;
    // this->ThrustConversionFnc(_msg->data + 1500);
}

GZ_REGISTER_MODEL_PLUGIN(AUV)

}