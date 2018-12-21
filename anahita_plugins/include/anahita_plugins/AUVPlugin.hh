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
#include <hyperion_msgs/Thrust.h>

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
        public: void QueueThread();
        public: double ThrustConversionFnc(int pwm);

        public: void thrustCB (const hyperion_msgs::ThrustConstPtr&);

        // Pointer to the model
        private: physics::ModelPtr model_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // total mass
        private: double mass_ = 0;

        // Pointer to links
        protected: gazebo::physics::LinkPtr north_link_;
        protected: gazebo::physics::LinkPtr south_link_;
        protected: gazebo::physics::LinkPtr east_link_;
        protected: gazebo::physics::LinkPtr west_link_;
        protected: gazebo::physics::LinkPtr south_west_link_;
        protected: gazebo::physics::LinkPtr south_east_link_;
        protected: gazebo::physics::LinkPtr north_west_link_;
        protected: gazebo::physics::LinkPtr north_east_link_;

        // Pointer to the base link
        protected: gazebo::physics::LinkPtr baseLink;

        // \brief: to store pwm sent from controller
        protected: double pwm_north_ = 0;
        protected: double pwm_south_ = 0;
        protected: double pwm_east_ = 0;
        protected: double pwm_west_ = 0;
        protected: double pwm_north_east_ = 0;
        protected: double pwm_north_west_ = 0;
        protected: double pwm_south_west_ = 0;
        protected: double pwm_south_east_ = 0;

        // ROS node
        private: std::unique_ptr<ros::NodeHandle> nh_;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber sub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: event::ConnectionPtr update_connection_;
    };
}

#endif // !AUVPLUGIN_HH
