#include <thruster_plugin/AUVPlugin.hh>

using namespace std;
namespace gazebo
{
AUV::AUV() {
    std::cout << "AUV plugin started successfully" << std::endl;
    this->inputCommand = 0;

    // char** argv;
    // int argc = 1;

    // ros::init(argc, argv, "AUV");
}
AUV::~AUV() {}

void AUV::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model_ = _model;
    this->sdf_ = _sdf;

    // std::string linkName = _sdf->Get<std::string>("linkName");
    this->baseLink = _model->GetLink("anahita::base_link");

    std::vector<gazebo::physics::LinkPtr> links_ = _model->GetLinks();

    cout << "links counts: " << links_.size() << endl;

    // cout << "link 2: " << links_[1]->GetName() << endl;

    if (_model == NULL) {
        std::cout << "Model link is NULL" << std::endl;
    }

    cout << "Model Name: " << _model->GetName() << endl;

    if (thrusterLink1 == NULL) {
        std::cout << "East thruster link is NULL" << std::endl;
    }
    if (thrusterLink2 == NULL) {
        std::cout << "West thruster link is NULL" << std::endl;
    }
    if (baseLink == NULL) {
        std::cout << "Base Link is NULL" << std::endl;
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
    // this->thrustTopicPublisher =
    //     this->nh_.advertise<std_msgs::Int32>(this->topicPrefix + "thrust", 1000);

    // // Subscribe to the input signal topic
    // this->commandSubscriber = this->nh_.subscribe(this->topicPrefix + "input", 
    //     1, &AUV::UpdateInput, this);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));
    // std::cout << "What the fuck happened" << std::endl;
}

void AUV::Update() {
    
    // std::cout << "here" << std::endl;
    ignition::math::Vector3d buoyancyForce;
    buoyancyForce = ignition::math::Vector3d(0, 0, 0);

    // cout << "what the fuck is happening" << endl;

    double mass = 0;
    // mass = this->baseLink->GetInertial()->Mass();
    // cout << "mass: " << mass << endl;
#if GAZEBO_MAJOR_VERSION >= 8
    mass = this->baseLink->GetInertial()->Mass();
    // cout << "this one" << endl;
#else
    mass = this->baseLink->GetInertial()->GetMass();
    // cout << "that one" << endl;
#endif

    std::vector<gazebo::physics::LinkPtr> links_ = this->model_->GetLinks();

    double g = 9.8;

    ignition::math::Vector3d CoB = ignition::math::Vector3d(0, 0, 0.05);
    ignition::math::Vector3d f1(1, 0, 0);
    ignition::math::Vector3d f2(-1, 0, 0);
    mass = 0;

    for (int i = 0; i < links_.size(); i++) {
        mass = mass + links_[i]->GetInertial()->Mass();
        cout << "links_[i]->GetInertial()->Mass(): " << links_[i]->GetInertial()->Mass() << endl;
    }
    cout << "mass: " << mass << endl;

    buoyancyForce = ignition::math::Vector3d(0, 0, mass*g);
    this->baseLink->AddForceAtRelativePosition(buoyancyForce, CoB);

    // ignition::math::Vector3d g1(0, 0, this->thrusterLink1->GetInertial()->Mass()*g);
    // ignition::math::Vector3d g2(0, 0, this->thrusterLink1->GetInertial()->Mass()*g);

    // this->thrusterLink1->AddForceAtRelativePosition(g1, CoB);
    // this->thrusterLink2->AddForceAtRelativePosition(g2, CoB);

    // this->thrustForce = this->thrustEfficiency * this->inputCommand * this->inputCommand;

    // // Use the thrust force limits
    // this->thrustForce = std::max(this->thrustForce, this->thrustMin);
    // this->thrustForce = std::min(this->thrustForce, this->thrustMax);

    // ignition::math::Vector3d f(1, 0, 0);
    // ignition::math::Vector3d f(1, 0, 0);

    this->thrusterLink1 = model_->GetLink("anahita::south_east");
    this->thrusterLink2 = model_->GetLink("anahita::south_west");
    gazebo::physics::LinkPtr thrusterLink3 = model_->GetLink("anahita::north_east");
    gazebo::physics::LinkPtr thrusterLink4 = model_->GetLink("anahita::north_west");

    // this->thrusterLink1->AddRelativeForce(f1);
    // this->thrusterLink2->AddRelativeForce(f1);
    // thrusterLink3->AddRelativeForce(f2);
    // thrusterLink4->AddRelativeForce(f2);

    // std::cout << "what is happening" << std::endl;
}

void AUV::Init() {

}

void AUV::UpdateInput(const std_msgs::Int32::ConstPtr& _msg) {
    this->inputCommand = _msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(AUV)

}