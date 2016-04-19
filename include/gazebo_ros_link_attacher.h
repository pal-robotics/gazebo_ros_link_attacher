/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#ifndef GAZEBO_ROS_LINK_ATTACHER_HH
#define GAZEBO_ROS_LINK_ATTACHER_HH

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

#include <std_msgs/String.h>
#include "gazebo_ros_link_attacher/Attach.h"
#include <geometry_msgs/Point.h>

namespace gazebo
{

   class GazeboRosLinkAttacher : public WorldPlugin
   {
      public:
        /// \brief Constructor
        GazeboRosLinkAttacher();

        /// \brief Destructor
        virtual ~GazeboRosLinkAttacher();

        /// \brief Load the controller
        void Load( physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/ );

        /// \brief Attach with a revolute joint link1 to link2
        bool attach(std::string model1, std::string link1,
                    std::string model2, std::string link2);

        /// \brief Detach link1 from link2
        bool detach();

      
      private:
        ros::NodeHandle nh_;
        ros::Subscriber attach_by_name_subscriber_;
        ros::Subscriber detach_subscriber_;

        void attach_callback(const gazebo_ros_link_attacher::Attach msg);
        void detach_callback(const std_msgs::String msg);

        bool attached;
        physics::JointPtr fixedJoint;

        physics::LinkPtr link1;
        physics::LinkPtr link2;

        /// \brief Model that contains this
        physics::ModelPtr model;

        /// \brief The physics engine.
        physics::PhysicsEnginePtr physics;

        /// \brief Pointer to the world.
        physics::WorldPtr world;

   };

}

#endif

