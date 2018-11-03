#ifndef AUVPLUGIN_HH
#define AUVPLUGIN_HH

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/common/Plugin.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Int32.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace gazebo
{
    class AUV : public ModelPlugin {
        public: AUV();
        public: ~AUV();

        public: virtual void Load(physics::ModelPtr, sdf::ElementPtr);
        public: virtual void Update();
        public: virtual void Init();
        public: void UpdateInput(const std_msgs::Int32::ConstPtr&);

        // Pointer to the model
        private: physics::ModelPtr model_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // world name
        private: std::string world_name_;

        // Pointer to the east thruster link
        protected: gazebo::physics::LinkPtr thrusterLink1;

        // Pointer to the west thruster link
        protected: gazebo::physics::LinkPtr thrusterLink2;

        // Pointer to the base link
        protected: gazebo::physics::LinkPtr baseLink;

        // Thruster ID, used to generated topic names automatically
        protected: int thrusterID;

        // Thruster efficiency
        protected: int thrustEfficiency;

        // Thrust force generated from the thruster
        protected: int thrustForce;

        /// \brief: Optional: Minimum thrust force output
        protected: int thrustMin;

        /// \brief: Optional: Maximum thrust force output
        protected: int thrustMax;

        // \brief: to store pwm sent from controller
        protected: int inputCommand;

        /// \brief Thruster topics prefix
        protected: std::string topicPrefix;

        // ROS node
        protected: ros::NodeHandle nh_;

        // Subscriber to the reference signal topic.
        protected: ros::Subscriber commandSubscriber;

        // Publisher to the output thrust topic
        protected: ros::Publisher thrustTopicPublisher;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: event::ConnectionPtr update_connection_;
    };
}

#endif // !AUVPLUGIN_HH
