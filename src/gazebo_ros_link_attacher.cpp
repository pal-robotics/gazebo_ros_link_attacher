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
    this->attach_by_name_subscriber_ = this->nh_.subscribe("attach_models", 1, &GazeboRosLinkAttacher::attach_callback, this);
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
    ROS_INFO_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
    physics::LinkPtr l2 = m2->GetLink(link2);
    if (l2 == NULL){
      ROS_ERROR_STREAM(link2 << " link was not found");
      return false;
    }

    ROS_INFO_STREAM("Links are: "  << l1->GetName() << " and " << l2->GetName());

    ROS_INFO_STREAM("Creating revolute joint on model: '" << model1 << "'");
    this->model = m1; // Store the model we created the joint, just in case
    this->fixedJoint = this->physics->CreateJoint("revolute", m1);

    this->link1 = l1; // Store the links too
    this->link2 = l2;
    ROS_INFO_STREAM("Attach");
    this->fixedJoint->Attach(l1, l2);
    ROS_INFO_STREAM("Loading links");
    this->fixedJoint->Load(l1,
                           l2, math::Pose());
    ROS_INFO_STREAM("SetModel");
    this->fixedJoint->SetModel(m1);
    ROS_INFO_STREAM("SetAxis");
    this->fixedJoint->SetAxis(0, math::Vector3(0, 0, 1));
    ROS_INFO_STREAM("SetHightstop");
    this->fixedJoint->SetHighStop(0, 0);
    ROS_INFO_STREAM("SetLowStop");
    this->fixedJoint->SetLowStop(0, 0);
    ROS_INFO_STREAM("Giving a name");
    this->fixedJoint->SetName("fixedjoint");
    ROS_INFO_STREAM("Init");
    this->fixedJoint->Init();
    ROS_INFO_STREAM("We are done");
    this->attached = true;

    return true;
  }

  bool GazeboRosLinkAttacher::detach()
  {
    this->fixedJoint->Detach();
    this->fixedJoint->Fini();
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
