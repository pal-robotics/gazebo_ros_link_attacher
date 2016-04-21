#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "gazebo_ros_link_attacher.h"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

  // Constructor
  GazeboRosLinkAttacher::GazeboRosLinkAttacher() :
    nh_("link_attacher_node")
  {
  }


  // Destructor
  GazeboRosLinkAttacher::~GazeboRosLinkAttacher()
  {
  }

  void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    this->world = _world;
    this->physics = this->world->GetPhysicsEngine();
    this->fixed_joint_counter = 0;
    this->attach_by_name_subscriber_ = this->nh_.subscribe("attach_models", 1, &GazeboRosLinkAttacher::attach_callback, this);
    this->detach_subscriber_ = this->nh_.subscribe("detach", 1, &GazeboRosLinkAttacher::detach_callback, this);
    ROS_INFO("Link attacher node initialized");
  }

  bool GazeboRosLinkAttacher::attach(std::string model1, std::string link1,
                                     std::string model2, std::string link2)
  {

    ROS_INFO_STREAM("Getting BasePtr of " << model1);
    physics::BasePtr b1 = this->world->GetByName(model1);
    if (b1 == NULL){
      ROS_ERROR_STREAM(model1 << " model was not found");
      return false;
    }
    ROS_INFO_STREAM("Getting BasePtr of " << model2);
    physics::BasePtr b2 = this->world->GetByName(model2);
    if (b2 == NULL){
      ROS_ERROR_STREAM(model2 << " model was not found");
      return false;
    }

    ROS_INFO_STREAM("Casting into ModelPtr");
    physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
    physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));

    ROS_INFO_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
    physics::LinkPtr l1 = m1->GetLink(link1);
    if (l1 == NULL){
      ROS_ERROR_STREAM(link1 << " link was not found");
      return false;
    }
    if (l1->GetInertial() == NULL){
        ROS_ERROR_STREAM("link1 inertia is NULL!");
    }
    else
        ROS_INFO_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->GetMass());
    ROS_INFO_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
    physics::LinkPtr l2 = m2->GetLink(link2);
    if (l2 == NULL){
      ROS_ERROR_STREAM(link2 << " link was not found");
      return false;
    }
    if (l2->GetInertial() == NULL){
        ROS_ERROR_STREAM("link2 inertia is NULL!");
    }
    else
        ROS_INFO_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->GetMass());

    ROS_INFO_STREAM("Links are: "  << l1->GetName() << " and " << l2->GetName());

    ROS_INFO_STREAM("Creating revolute joint on model: '" << model1 << "'");
    this->model = m1; // Store the model we created the joint, just in case
    this->fixedJoint[this->fixed_joint_counter] = this->physics->CreateJoint("revolute", m1);

    this->link1 = l1; // Store the links too
    this->link2 = l2;
    ROS_INFO_STREAM("Attach");
    this->fixedJoint[this->fixed_joint_counter]->Attach(l1, l2);
    ROS_INFO_STREAM("Loading links");
    this->fixedJoint[this->fixed_joint_counter]->Load(l1,
                           l2, math::Pose());
    ROS_INFO_STREAM("SetModel");
    this->fixedJoint[this->fixed_joint_counter]->SetModel(m2);
    /*
     * If SetModel is not done we get:
     * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
     failed in void gazebo::physics::Entity::PublishPose():
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
     An entity without a parent model should not happen

     * If SetModel is given the same model than CreateJoint given
     * Gazebo crashes with
     * ***** Internal Program Error - assertion (self->inertial != __null)
     failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
     */

//    ROS_INFO_STREAM("SetAxis");
//    this->fixedJoint[this->fixed_joint_counter]->SetAxis(0, math::Vector3(0, 0, 1));
    ROS_INFO_STREAM("SetHightstop");
    this->fixedJoint[this->fixed_joint_counter]->SetHighStop(0, 0);
    ROS_INFO_STREAM("SetLowStop");
    this->fixedJoint[this->fixed_joint_counter]->SetLowStop(0, 0);
//    ROS_INFO_STREAM("Giving a name");
//    this->fixedJoint[this->fixed_joint_counter]->SetName("fixedjoint");
    ROS_INFO_STREAM("Init");
    this->fixedJoint[this->fixed_joint_counter]->Init();
    ROS_INFO_STREAM("We are done");
    this->attached = true;
    this->fixed_joint_counter++;

    return true;
  }

  bool GazeboRosLinkAttacher::detach()
  {
    this->fixedJoint[this->fixed_joint_counter-1]->Detach();
    this->fixedJoint[this->fixed_joint_counter-1]->Fini();
    this->fixedJoint[this->fixed_joint_counter-1]->Reset();
    this->attached = false;
    return true;
  }


  void GazeboRosLinkAttacher::attach_callback(const gazebo_ros_link_attacher::Attach msg)
  {
    ROS_INFO_STREAM("Received request to attach model: '" << msg.model_name_1
                    << "' using link: '" << msg.link_name_1 << "' with model: '"
                    << msg.model_name_2 << "' using link: '" <<  msg.link_name_2 << "'");
    if (! this->attach(msg.model_name_1, msg.link_name_1,
                       msg.model_name_2, msg.link_name_2)){
      ROS_ERROR_STREAM("Could not make the attach.");
    }

  }

  void GazeboRosLinkAttacher::detach_callback(const std_msgs::String msg){
    this->detach();
  }

}
