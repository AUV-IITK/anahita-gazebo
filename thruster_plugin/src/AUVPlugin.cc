#include <thruster_plugin/AUVPlugin.hh>

namespace gazebo
{
AUV::AUV() {
    ROS_INFO("AUV plugin started successfully");
    this->inputCommand = 0;
}
AUV::~AUV() {}

void AUV::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model_ = _model;
    this->sdf_ = _sdf;

    std::string linkName = _sdf->Get<std::string>("linkName");
    this->thrusterLink1 = _model->GetLink("east");
    this->thrusterLink2 = _model->GetLink("west");
    this->baseLink = _model->GetLink("base_link");

    if (thrusterLink1 == NULL) {
        ROS_INFO("East thruster link is NULL");
    }
    if (thrusterLink2 == NULL) {
        ROS_INFO("West thruster link is NULL");
    }
    if (baseLink == NULL) {
        ROS_INFO("Base Link is NULL");
    }
 
    this->thrusterID = _sdf->Get<int>("thrusterID");
    this->thrustEfficiency = _sdf->Get<int>("thrustEfficiency");

    // Thrust force interval
    if (_sdf->HasElement("thrustMin"))
        this->thrustMin = _sdf->Get<double>("thrustMin");

    if (_sdf->HasElement("thrustMax"))
        this->thrustMax = _sdf->Get<double>("thrustMax");

    if (this->thrustMin >= this->thrustMax)
    {
        gzmsg << "thrustMax must be greater than thrustMin, returning to default values..." << std::endl;
        this->thrustMin = std::numeric_limits<double>::lowest();
        this->thrustMax = std::numeric_limits<double>::max();
    }

    // Root string for topics
    std::stringstream strs;
    strs << "/" << _model->GetName() << "/thrusters/" << this->thrusterID << "/";
    this->topicPrefix = strs.str();

    // Advertise the thrust topic
    this->thrustTopicPublisher =
        this->nh_.advertise<std_msgs::Int32>(this->topicPrefix + "thrust", 1000);

    // Subscribe to the input signal topic
    this->commandSubscriber = this->nh_.subscribe(this->topicPrefix + "input", 
        1, &AUV::UpdateInput, this);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));
}

void AUV::Update() {
    
    ignition::math::Vector3d buoyancyForce;
    buoyancyForce = ignition::math::Vector3d(0, 0, 0);

    double mass;
#if GAZEBO_MAJOR_VERSION >= 8
    mass = this->baseLink->GetInertial()->Mass();
#else
    mass = this->baseLink->GetInertial()->GetMass();
#endif

    double g = 9.8;
    buoyancyForce = ignition::math::Vector3d(0, 0, mass*g);

    ignition::math::Vector3d CoB = ignition::math::Vector3d(0, 0, 0.05);
    this->baseLink->AddForceAtRelativePosition(buoyancyForce, CoB);

    // this->thrustForce = this->thrustEfficiency * this->inputCommand * this->inputCommand;

    // // Use the thrust force limits
    // this->thrustForce = std::max(this->thrustForce, this->thrustMin);
    // this->thrustForce = std::min(this->thrustForce, this->thrustMax);

    // ignition::math::Vector3d force(1, 0, 0);
    // this->thrusterLink1->AddRelativeForce(force);
    // this->thrusterLink2->AddRelativeForce(force);
}

void AUV::Init() {

}

void AUV::UpdateInput(const std_msgs::Int32::ConstPtr& _msg) {
    this->inputCommand = _msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(AUV)

}