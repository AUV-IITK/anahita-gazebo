#include <anahita_plugins/imuPlugin.hh>

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(gazebo::IMUPlugin)
  ////////////////////////////////////////////////////

  IMUPlugin::IMUPlugin(): SensorPlugin()
  {
    accelerometer_data = ignition::math::Vector3d(0, 0, 0);
    gyroscope_data = ignition::math::Vector3d(0, 0, 0);
    orientation = ignition::math::Quaterniond(1,0,0,0);
    sensor_ = NULL;
  }

  IMUPlugin::~IMUPlugin() {}

  void IMUPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    this->sdf_ = _sdf;
    this->world_name_ = _sensor->WorldName();

    this->sensor_ =  dynamic_cast<sensors::ImuSensor*>(_sensor.get());

    if (this->sensor_ == NULL)
    {
      gzthrow("ERROR: Sensor pointer is NULL");
      return;
    }

    this->sensor_->SetActive(true);

    // Topic to which the IMU data will be published
    if (this->sdf_->HasElement("topic"))
    {
      this->topic_name_ = sdf_->Get<std::string>("topic");
      std::cerr << "<topic> set to: "<< topic_name_ << std::endl;
    }
    else
    {
      this->topic_name_ = "/varun/sensors/imu/data";
      std::cerr << "missing <topic>, set to /namespace/default: " << topic_name_ << std::endl;
    }

    // Update Rate
    if (this->sdf_->HasElement("update_rate"))
    {
      update_rate =  sdf_->Get<double>("update_rate");
      std::cerr << "<update_rate> set to: " << update_rate << std::endl;
    }
    else
    {
      update_rate = 1.0;
      std::cerr << "missing <update_rate>, set to default: " << update_rate << std::endl;
    }

    connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&IMUPlugin::OnUpdate, this, _1));
    
    std::cerr << "IMU plugin loaded successfully!!!" << std::endl;
    std::cerr << "Orientation is representated in Quanternion" << std::endl;

  }

  void IMUPlugin::OnUpdate(const common::UpdateInfo &)
  {

    orientation = sensor_->Orientation();
    accelerometer_data = sensor_->LinearAcceleration();
    gyroscope_data = sensor_->AngularVelocity();
  }

}
