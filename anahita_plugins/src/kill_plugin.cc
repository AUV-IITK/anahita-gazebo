#include <anahita_plugins/kill_plugin.hh>

using namespace std;

namespace gazebo {

KillPlugin::KillPlugin() {}
KillPlugin::~KillPlugin() {}

void KillPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    cout << "KillPlugin Loaded!!" << endl;
    
    this->model_ = _model;
    this->sdf_ = _sdf;
    this->world_ = this->model_->GetWorld();

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle("anahita/kill_plugin"));

    ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Bool>(
        "/kill/angularvelocity/z", 1,
        boost::bind(&KillPlugin::AngularVelCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Bool>(
        "/kill/linearvelocity/x", 1,
        boost::bind(&KillPlugin::LinearVelXCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Bool>(
        "/kill/linearvelocity/y", 1,
        boost::bind(&KillPlugin::LinearVelYCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so4 = ros::SubscribeOptions::create<std_msgs::Bool>(
        "/kill/linearvelocity/z", 1,
        boost::bind(&KillPlugin::LinearVelZCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->angular_vel_sub_ = this->nh_->subscribe(so1);
    this->linear_vel_x_sub_ = this->nh_->subscribe(so2);
    this->linear_vel_y_sub_ = this->nh_->subscribe(so3);
    this->linear_vel_z_sub_ = this->nh_->subscribe(so4);

    this->rosQueueThread = std::thread(std::bind(&KillPlugin::QueueThread, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&KillPlugin::OnUpdate, this));
}

void KillPlugin::OnUpdate() {
    this->curr_time_ = this->world_->SimTime().Double();
}

/// \brief ROS helper function that processes messages
void KillPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->nh_->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void KillPlugin::AngularVelCB(const std_msgs::Bool::ConstPtr& msg) {    
    if (msg->data) {
        ignition::math::Vector3d vel = this->model_->RelativeAngularVel();
        vel = ignition::math::Vector3d(vel.X(), vel.Y(), 0);
        this->model_->SetAngularVel(vel);
        // cout << "Angular velocity set to 0" << endl;
        ang_vel_relax_time_ = this->world_->SimTime().Double();
    }
}

void KillPlugin::LinearVelXCB(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ignition::math::Vector3d vel = this->model_->RelativeLinearVel();
        vel = ignition::math::Vector3d(0, vel.Y(), vel.Z());
        this->model_->SetLinearVel(vel);
        // cout << "Linear velocity x set to 0" << endl;
        lin_velx_relax_time_ = this->world_->SimTime().Double();
    }
}

void KillPlugin::LinearVelYCB(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ignition::math::Vector3d vel = this->model_->RelativeLinearVel();
        vel = ignition::math::Vector3d(vel.X(), 0, vel.Z());
        this->model_->SetLinearVel(vel);
        // cout << "Linear velocity y set to 0" << endl;
        lin_vely_relax_time_ = this->world_->SimTime().Double();
    }
}

void KillPlugin::LinearVelZCB(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ignition::math::Vector3d vel = this->model_->RelativeLinearVel();
        vel = ignition::math::Vector3d(vel.X(), vel.Y(), 0);
        this->model_->SetLinearVel(vel);
        // cout << "Linear velocity z set to 0" << endl;
        lin_velz_relax_time_ = this->world_->SimTime().Double();
    }
}

GZ_REGISTER_MODEL_PLUGIN(KillPlugin)

}