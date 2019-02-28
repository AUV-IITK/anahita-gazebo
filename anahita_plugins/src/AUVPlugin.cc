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

    pub_ = nh_->advertise<std_msgs::Float32>("/anahita/z_coordinate", 1);

    this->rosQueueThread = std::thread(std::bind(&AUV::QueueThread, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));
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

    height = -1*height;

    static std_msgs::Float32 height_;
    height_.data = height;

    pub_.publish(height_);
}

void AUV::thrustCB (const hyperion_msgs::ThrustConstPtr& pwm) {

    pwm_west_ = (static_cast<double>(pwm->forward_left)/200);
    pwm_east_ = (static_cast<double>(pwm->forward_right)/200);
    
    pwm_south_ = (static_cast<double>(pwm->sideward_back)/200);
    pwm_north_ = (static_cast<double>(pwm->sideward_front)/200);
    
    pwm_north_east_ = (static_cast<double>(pwm->upward_north_east)/200);
    pwm_north_west_ = (static_cast<double>(pwm->upward_north_west)/200);
    
    pwm_south_east_ = (static_cast<double>(pwm->upward_south_east)/200);
    pwm_south_west_ = (static_cast<double>(pwm->upward_south_west)/200);

    static double vel_x = 0;
    static double vel_y = 0;
    static double vel_z = 0;
    static double ang_vel_z = 0;
    static double ang_vel_y = 0;
    static double ang_vel_x = 0;
    static ignition::math::Vector3d lin_vel = ignition::math::Vector3d(0, 0, 0);
    static ignition::math::Vector3d ang_vel = ignition::math::Vector3d(0, 0, 0);

    vel_x = (-pwm_east_ - pwm_west_)/2;
    vel_y = (pwm_south_ + pwm_north_)/2;
    vel_z = (-pwm_south_east_ - pwm_south_west_ - pwm_north_east_ - pwm_north_west_)/4;

    ang_vel_z = (pwm_north_ - pwm_south_ + pwm_west_ - pwm_east_)/2;
    ang_vel_x = (pwm_north_west_ + pwm_south_west_ - pwm_north_east_ - pwm_south_east_)/2;
    ang_vel_y = (pwm_north_east_ + pwm_north_west_ - pwm_south_east_ - pwm_south_west_)/2;

    lin_vel = ignition::math::Vector3d(vel_x, vel_y, vel_z);
    ang_vel = ignition::math::Vector3d(0, 0, ang_vel_z);
    // ang_vel = ignition::math::Vector3d(ang_vel_x, ang_vel_y, ang_vel_z);

    this->model_->SetLinearVel(lin_vel);
    this->model_->SetAngularVel(ang_vel);

} 

GZ_REGISTER_MODEL_PLUGIN(AUV)

}