#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.msg import Attach

if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    detach_pub = rospy.Publisher('/link_attacher_node/detach',
                                 Attach, queue_size=1)
    rospy.loginfo("Created publisher to /link_attacher_node/detach")
    rospy.sleep(1.0)

    # Link them
    rospy.loginfo("Detaching cube1 and cube2")
    amsg = Attach()
    amsg.model_name_1 = "cube1"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube2"
    amsg.link_name_2 = "link"

    detach_pub.publish(amsg)
    rospy.sleep(1.0)
    # From the shell:
    """
rostopic pub /link_attacher_node/attach_models gazebo_ros_link_attacher/Attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """
    rospy.loginfo("Published into linking service!")


    rospy.loginfo("Detaching cube2 and cube3")
    amsg = Attach()
    amsg.model_name_1 = "cube2"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube3"
    amsg.link_name_2 = "link"

    detach_pub.publish(amsg)
    rospy.sleep(1.0)


    rospy.loginfo("Detaching cube3 and cube1")
    amsg = Attach()
    amsg.model_name_1 = "cube3"
    amsg.link_name_1 = "link"
    amsg.model_name_2 = "cube1"
    amsg.link_name_2 = "link"

    detach_pub.publish(amsg)
    rospy.sleep(2.0)
