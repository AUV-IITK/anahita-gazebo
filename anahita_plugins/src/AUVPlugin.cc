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
    
    int argc = 0;

    ros::init(argc, NULL, "anahita");

    this->nh_ = new ros::NodeHandle("anahita");

    this->north_sub_ = this->nh_->subscribe("/pwm/sidewardFront", 
        1, &AUV::SidewardBackCB, this);

    this->south_sub_ = this->nh_->subscribe("/pwm/sidewardBack", 
        1, &AUV::SidewardFrontCB, this);

    this->west_sub_ = this->nh_->subscribe("/pwm/forwardLeft", 
        1, &AUV::ForwardLeftCB, this);

    this->east_sub_ = this->nh_->subscribe("/pwm/forwardRight", 
        1, &AUV::ForwardRightCB, this);

    this->north_east_sub_ = this->nh_->subscribe("/pwm/upwardNorthEast", 
        1, &AUV::UpwardNorthEastCB, this);

    this->north_west_sub_ = this->nh_->subscribe("/pwm/upwardNorthWest", 
        1, &AUV::UpwardNorthWestCB, this);

    this->south_west_sub_ = this->nh_->subscribe("/pwm/upwardSouthWest", 
        1, &AUV::UpwardSouthWestCB, this);

    this->south_east_sub_ = this->nh_->subscribe("/pwm/upwardSouthEast", 
        1, &AUV::UpwardSouthEastCB, this);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));

}

double AUV::ThrustConversionFnc(int pwm) {
    int x = pwm;
    double A = -222.54;
    double B = 0.42;
    double C = -0.00027;
    double D = 0.0000000622;

    double f = A + B*x + C*x*x + D*x*x*x;
    return f; 
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

    this->north_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->south_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->north_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->north_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->south_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));
    this->south_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, 0));

}

void AUV::Init() {

}

void AUV::SidewardBackCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_ = _msg->data;
}

void AUV::SidewardFrontCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_ = _msg->data;
}

void AUV::ForwardLeftCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_west_ = _msg->data;
}

void AUV::ForwardRightCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_east_ = _msg->data;
}

void AUV::UpwardNorthEastCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_east_ = _msg->data;
}

void AUV::UpwardNorthWestCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_north_west_ = _msg->data;
}

void AUV::UpwardSouthEastCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_east_ = _msg->data;
}

void AUV::UpwardSouthWestCB(const std_msgs::Int32::ConstPtr& _msg) {
    this->pwm_south_west_ = _msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(AUV)

}