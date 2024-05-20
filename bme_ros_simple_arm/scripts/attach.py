#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def attach_left_finger(target):
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and red box.")
    req = AttachRequest()
    req.model_name_1 = "mogi_arm"
    req.link_name_1 = "left_finger"
    req.model_name_2 = target
    req.link_name_2 = "link"

    attach_srv.call(req)

"""
if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and red box.")
    req = AttachRequest()
    req.model_name_1 = "mogi_arm"
    req.link_name_1 = "left_finger"
    req.model_name_2 = "red_box"
    req.link_name_2 = "link"

    attach_srv.call(req)

    # From command line:
    
    rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
    link_name_1: 'link'
    model_name_2: 'cube2'
    link_name_2: 'link'"
    
"""
