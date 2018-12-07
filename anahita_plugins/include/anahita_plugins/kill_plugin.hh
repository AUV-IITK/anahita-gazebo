#pragma once

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <string>
#include <vector>

namespace gazebo {
    class KillPlugin : public ModelPlugin {
        public: KillPlugin();
        public: ~KillPlugin();
        public: virtual void Load(physics::ModelPtr, sdf::ElementPtr);
        public: virtual void OnUpdate();
        public: void AngularVelCB(const std_msgs::Bool::ConstPtr&);
        public: void LinearVelXCB(const std_msgs::Bool::ConstPtr&);
        public: void LinearVelYCB(const std_msgs::Bool::ConstPtr&);
        public: void LinearVelZCB(const std_msgs::Bool::ConstPtr&);
        public: void QueueThread();

        private: double ang_vel_relax_time_ = 0;
        private: double lin_velx_relax_time_ = 0;
        private: double lin_vely_relax_time_ = 0;
        private: double lin_velz_relax_time_ = 0;
        private: double curr_time_ = 0;

        // Pointer to the model
        private: physics::ModelPtr model_;

        // Pointer to the world
        private: physics::WorldPtr world_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // ROS node
        private: std::unique_ptr<ros::NodeHandle> nh_;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber angular_vel_sub_;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber linear_vel_x_sub_;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber linear_vel_y_sub_;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber linear_vel_z_sub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: event::ConnectionPtr update_connection_;
    };
}