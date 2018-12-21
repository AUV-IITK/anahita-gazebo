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

    std::vector<gazebo::physics::LinkPtr> links_ = this->model_->GetLinks();

    for (int i = 0; i < links_.size(); i++) {
        mass_ = mass_ + links_[i]->GetInertial()->Mass();
    }

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
    ros::SubscribeOptions so = ros::SubscribeOptions::create<hyperion_msgs::Thrust>(
        "/pwm", 1,
        boost::bind(&AUV::thrustCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->sub_ = this->nh_->subscribe(so);

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
    
    static ignition::math::Vector3d buoyancyForce = ignition::math::Vector3d(0, 0, 0);

    static double g = 9.8;

    static ignition::math::Vector3d CoB = ignition::math::Vector3d(0, 0, 0.05);
    static ignition::math::Pose3d pos = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    static double height = -1.5;

    pos = model_->WorldPose();
    height = pos.Pos().Z();

    buoyancyForce = ignition::math::Vector3d(0, 0, mass_*g);

    if (height <= 0) {
        this->baseLink->AddForceAtRelativePosition(buoyancyForce, CoB);
    }

    this->north_link_->AddRelativeForce(ignition::math::Vector3d(0, pwm_north_, 0)); 
    this->south_link_->AddRelativeForce(ignition::math::Vector3d(0, pwm_south_, 0));
    this->east_link_->AddRelativeForce(ignition::math::Vector3d(-pwm_east_, 0, 0));
    this->west_link_->AddRelativeForce(ignition::math::Vector3d(-pwm_west_, 0, 0));
    this->north_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_north_east_));
    this->north_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_north_west_));
    this->south_east_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_south_east_));
    this->south_west_link_->AddRelativeForce(ignition::math::Vector3d(0, 0, pwm_south_west_));
}

void AUV::thrustCB (const hyperion_msgs::ThrustConstPtr& pwm) {
    pwm_west_ = pwm->forward_left;
    pwm_east_ = pwm->forward_right;
    
    pwm_south_ = pwm->sideward_back;
    pwm_north_ = pwm->sideward_front;
    
    pwm_north_east_ = pwm->upward_north_east;
    pwm_north_west_ = pwm->upward_north_west;
    
    pwm_south_east_ = pwm->upward_south_east;
    pwm_south_west_ = pwm->upward_south_west;
} 

GZ_REGISTER_MODEL_PLUGIN(AUV)

}