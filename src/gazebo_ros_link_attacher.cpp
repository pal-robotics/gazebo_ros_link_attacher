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
    this->detach_subscriber_ = this->nh_.subscribe("detach", 1, &GazeboRosLinkAttacher::detach_callback, this);
    ROS_INFO("Link attacher node initialized");
  }

  bool GazeboRosLinkAttacher::attach(std::string model1, std::string link1,
                                     std::string model2, std::string link2)
  {

    // look for any previous instance of the joint first.
    fixedJoint j;
    if(this->getJoint(model1, link1, model2, link2, j)){
        ROS_INFO_STREAM("Joint already existed");
    }
    else{
        ROS_INFO_STREAM("Creating new joint");
    }
    j.model1 = model1;
    j.link1 = link1;
    j.model2 = model2;
    j.link2 = link2;
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
    j.m1 = m1;
    physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
    j.m2 = m2;

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
    j.l1 = l1;
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
    j.l2 = l2;

    ROS_INFO_STREAM("Links are: "  << l1->GetName() << " and " << l2->GetName());

    ROS_INFO_STREAM("Creating revolute joint on model: '" << model1 << "'");
    j.joint = this->physics->CreateJoint("revolute", m1);
    this->joints.push_back(j);

    ROS_INFO_STREAM("Attach");
    j.joint->Attach(l1, l2);
    ROS_INFO_STREAM("Loading links");
    j.joint->Load(l1, l2, math::Pose());
    ROS_INFO_STREAM("SetModel");
    j.joint->SetModel(m2);
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

    ROS_INFO_STREAM("SetHightstop");
    j.joint->SetHighStop(0, 0);
    ROS_INFO_STREAM("SetLowStop");
    j.joint->SetLowStop(0, 0);
    ROS_INFO_STREAM("Init");
    j.joint->Init();
    ROS_INFO_STREAM("We are done");

    return true;
  }

  bool GazeboRosLinkAttacher::detach(std::string model1, std::string link1,
                                     std::string model2, std::string link2)
  {
      // search for the instance of joint and do detach
      fixedJoint j;
      if(this->getJoint(model1, link1, model2, link2, j)){
          j.joint->Detach();
          j.joint->Fini();
          j.joint->Reset();
          return true;
      }

    return false;
  }

  bool GazeboRosLinkAttacher::getJoint(std::string model1, std::string link1,
                                       std::string model2, std::string link2,
                                       fixedJoint &joint){
    fixedJoint j;
    for(std::vector<fixedJoint>::iterator it = this->joints.begin(); it != this->joints.end(); ++it){
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0)
                && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)){
            joint = j;
            return true;
        }
    }
    return false;

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

  void GazeboRosLinkAttacher::detach_callback(const gazebo_ros_link_attacher::Attach msg){
      ROS_INFO_STREAM("Received request to detach model: '" << msg.model_name_1
                      << "' using link: '" << msg.link_name_1 << "' with model: '"
                      << msg.model_name_2 << "' using link: '" <<  msg.link_name_2 << "'");
      if (! this->detach(msg.model_name_1, msg.link_name_1,
                         msg.model_name_2, msg.link_name_2)){
        ROS_ERROR_STREAM("Could not make the detach.");
      }
  }

}
