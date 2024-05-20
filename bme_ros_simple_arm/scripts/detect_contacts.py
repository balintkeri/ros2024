#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ContactsState

class ContactChecker:
    def __init__(self):
        self.contact_detected = False
        self.contact_received = False
        self.subscriber = None

    def get_contacts (self, msg):
        if (len(msg.states) == 0):
            rospy.loginfo("No contacts were detected!")
            self.contact_detected = False

        else:
            if 'left_finger' in msg.states[0].collision1_name:
                rospy.loginfo("Collision detected with %s." % msg.states[0].collision2_name.split("::")[0])
            elif 'left_finger' in msg.states[0].collision2_name:
                rospy.loginfo("Collision detected with %s." % msg.states[0].collision1_name.split("::")[0])
            else:
                rospy.loginfo("Unknown collision")
            self.contact_detected = True
        self.contact_received = True
        # Unregister the subscriber once a message is received
        self.subscriber.unregister()

def check_for_contact(timeout=5):
    #rospy.init_node('gps_waypoint_follower', anonymous=True)
    contact_checker = ContactChecker()
    contact_checker.subscriber = rospy.Subscriber('/contact_vals', ContactsState, contact_checker.get_contacts)

    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        if contact_checker.contact_received:
            return contact_checker.contact_detected
        if rospy.get_time() - start_time > timeout:
            break
        rospy.sleep(0.1)

    return False

"""
rospy.init_node('gps_waypoint_follower')

sub_contacts = rospy.Subscriber ('/contact_vals', ContactsState, get_contacts)

rospy.spin()
"""
