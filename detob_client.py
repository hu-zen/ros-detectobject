#!/usr/bin/env python3
import rospy
from your_package_name.srv import detectobject

def read_distance_client(threshold=100.0):
    """
    Calls the 'read_distance' service to get the sensor distance
    and determines if an object is detected based on the threshold.
    """
    rospy.wait_for_service('read_distance')
    try:
        read_distance = rospy.ServiceProxy('read_distance', detectobject)
        response = read_distance()
        if response.distance >= 0:  # Check for valid distance
            if response.distance < threshold:
                rospy.loginfo(f"Object Detected! Distance: {response.distance:.1f} cm")
            else:
                rospy.loginfo(f"No Object. Distance: {response.distance:.1f} cm")
        else:
            rospy.logwarn("Failed to read distance.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('distance_client')
    threshold = rospy.get_param('~threshold', 100.0)  # Allow threshold to be set via parameters
    rospy.loginfo(f"Using detection threshold: {threshold} cm")
    while not rospy.is_shutdown():
        read_distance_client(threshold)
        rospy.sleep(1)  # Call the service every second
