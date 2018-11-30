#ifndef AUVPLUGIN_HH
#define AUVPLUGIN_HH

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

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
        public: void QueueThread();
        public: double ThrustConversionFnc(int pwm);

        public: void SidewardBackCB (const std_msgs::Int32::ConstPtr&);
        public: void SidewardFrontCB (const std_msgs::Int32::ConstPtr&);
        public: void ForwardLeftCB (const std_msgs::Int32::ConstPtr&);
        public: void ForwardRightCB (const std_msgs::Int32::ConstPtr&);
        public: void UpwardNorthEastCB (const std_msgs::Int32::ConstPtr&);
        public: void UpwardNorthWestCB (const std_msgs::Int32::ConstPtr&);
        public: void UpwardSouthEastCB (const std_msgs::Int32::ConstPtr&);
        public: void UpwardSouthWestCB (const std_msgs::Int32::ConstPtr&);

        // Pointer to the model
        private: physics::ModelPtr model_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // world name
        private: std::string world_name_;

        // Pointer to the north thruster link
        protected: gazebo::physics::LinkPtr north_link_;

        // Pointer to the south thruster link
        protected: gazebo::physics::LinkPtr south_link_;

        protected: gazebo::physics::LinkPtr east_link_;
        protected: gazebo::physics::LinkPtr west_link_;
        protected: gazebo::physics::LinkPtr south_west_link_;
        protected: gazebo::physics::LinkPtr south_east_link_;
        protected: gazebo::physics::LinkPtr north_west_link_;
        protected: gazebo::physics::LinkPtr north_east_link_;

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
        protected: int pwm_north_ = 0;
        protected: int pwm_south_ = 0;
        protected: int pwm_east_ = 0;
        protected: int pwm_west_ = 0;
        protected: int pwm_north_east_ = 0;
        protected: int pwm_north_west_ = 0;
        protected: int pwm_south_west_ = 0;
        protected: int pwm_south_east_ = 0;

        /// \brief Thruster topics prefix
        protected: std::string topicPrefix;

        // ROS node
        private: std::unique_ptr<ros::NodeHandle> nh_;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber north_sub_;
        protected: ros::Subscriber south_sub_;
        protected: ros::Subscriber east_sub_;
        protected: ros::Subscriber west_sub_;
        protected: ros::Subscriber north_west_sub_;
        protected: ros::Subscriber north_east_sub_;
        protected: ros::Subscriber south_east_sub_;
        protected: ros::Subscriber south_west_sub_;

        // Publisher to the output thrust topic
        protected: ros::Publisher thrustTopicPublisher;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: event::ConnectionPtr update_connection_;
    };
}

#endif // !AUVPLUGIN_HH
