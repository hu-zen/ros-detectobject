#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import detectobject

def detect_object_client(threshold):
    rospy.wait_for_service('detect_object')
    try:
        # Menghubungkan dengan service detect_object
        detect_object = rospy.ServiceProxy('detect_object', detectobject)
        # Memanggil service dan mengirimkan threshold
        response = detect_object(threshold)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return "Error: Service call failed."

if __name__ == "__main__":
    rospy.init_node('detectobject_client')

    # Tentukan threshold untuk deteksi
    threshold = 30.0  # Dalam cm

    rospy.loginfo(f"Requesting detection with threshold: {threshold} cm")
    result = detect_object_client(threshold)
    rospy.loginfo(f"Server response: {result}")
