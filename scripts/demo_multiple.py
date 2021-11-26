#!/usr/bin/env python3

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from gazebo_ros_link_attacher.srv import Attach
from copy import deepcopy
from transforms3d.euler import euler2quat

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnEntity.Request()
    req.name = modelname
    req.xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = euler2quat(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_attach_links')

    attach_srv = node.create_client(Attach, '/attach')
    while not attach_srv.wait_for_service(timeout_sec=1.0):
      node.get_logger().info(f'Waiting for service {attach_srv.srv_name}...')

    service_name = 'spawn_entity'
    spawn_srv = node.create_client(SpawnEntity, '/spawn_entity')
    while not spawn_srv.wait_for_service(timeout_sec=1.0):
      node.get_logger().info(f'Waiting for service {spawn_srv.srv_name}...')

    node.get_logger().info("Connected to service!")

    # Spawn object 1
    node.get_logger().info("Spawning cube1")
    req1 = create_cube_request("cube1",
                              0.0, 0.0, 0.51,  # position
                              0.0, 0.0, 0.0,  # rotation
                              1.0, 1.0, 1.0)  # size
    resp = spawn_srv.call_async(req1)
    rclpy.spin_until_future_complete(node, resp)

    # Spawn object 2
    node.get_logger().info("Spawning cube2")
    req2 = create_cube_request("cube2",
                              0.0, 1.1, 0.41,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.8, 0.8, 0.8)  # size
    resp = spawn_srv.call_async(req2)
    rclpy.spin_until_future_complete(node, resp)

    # Spawn object 3
    node.get_logger().info("Spawning cube3")
    req3 = create_cube_request("cube3",
                              0.0, 2.1, 0.41,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.4, 0.4, 0.4)  # size

    resp = spawn_srv.call_async(req3)
    rclpy.spin_until_future_complete(node, resp)

    # Link them
    node.get_logger().info("Attaching cube1 and cube2")

    amsg = Attach.Request()
    amsg.model_name_1 = "cube1"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube2"
    amsg.link_name_2 = "link"

    resp = attach_srv.call_async(amsg)
    rclpy.spin_until_future_complete(node, resp)
    # From the shell:
    """
ros2 service call /attach 'gazebo_ros_link_attacher/srv/Attach' '{model_name_1: 'cube1',
link_name_1: 'link',
model_name_2: 'cube2',
link_name_2: 'link'}'
    """
    node.get_logger().info("Published into linking service!")


    node.get_logger().info("Attaching cube2 and cube3")
    amsg = Attach.Request()
    amsg.model_name_1 = "cube2"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube3"
    amsg.link_name_2 = "link"

    resp = attach_srv.call_async(amsg)
    rclpy.spin_until_future_complete(node, resp)


    node.get_logger().info("Attaching cube3 and cube1")
    amsg = Attach.Request()
    amsg.model_name_1 = "cube3"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube1"
    amsg.link_name_2 = "link"

    resp = attach_srv.call_async(amsg)
    rclpy.spin_until_future_complete(node, resp)
