#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import DetectObject

def detect_object_client(threshold):
    """
    Call the detect_object service with a threshold value.
    """
    rospy.wait_for_service('detect_object')
    try:
        detect_object = rospy.ServiceProxy('detect_object', DetectObject)
        response = detect_object(threshold)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return "Error: Service call failed."

if _name_ == "_main_":
    rospy.init_node('detect_object_client')

    # Specify the threshold distance
    threshold = 30.0  # Adjust as needed

    rospy.loginfo(f"Requesting detection with threshold: {threshold} cm")
    result = detect_object_client(threshold)
    rospy.loginfo(f"Service response: {result}")
