#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import detectobject

def detect_object_client(threshold):
    """
    Call the detectobject service with a threshold value.
    """
    rospy.wait_for_service('detectobject')
    try:
        detect_object = rospy.ServiceProxy('detectobject', detectobject)
        response = detect_object(threshold)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return "Error: Service call failed."

if __name__ == "__main__":
    rospy.init_node('detectobject_client')

    # Specify the threshold distance
    threshold = 30.0  # Adjust as needed

    rospy.loginfo(f"Requesting detection with threshold: {threshold} cm")
    result = detect_object_client(threshold)
    rospy.loginfo(f"Server response: {result}")
