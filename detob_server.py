#!/usr/bin/env python3
import rospy
from your_package_name.srv import detectobject, detectobjectResponse
import serial

def read_sensor_cm(port='/dev/ttyUSB0', baudrate=57600):
    """
    Reads data from MaxBotix HRUSB sensor and returns distance in cm.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
        if raw_data.startswith('R') and raw_data[1:].isdigit():
            mm_distance = int(raw_data[1:])  # Extract number after 'R'
            cm_distance = mm_distance / 10.0  # Convert to cm
            return cm_distance
        else:
            return None  # Invalid data
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
        return None

def handle_read_distance(req):
    """
    Handles the service request to read the distance from the sensor.
    """
    distance = read_sensor_cm()
    if distance is not None:
        rospy.loginfo(f"Distance measured: {distance:.1f} cm")
        return detectobjectResponse(distance)
    else:
        rospy.logwarn("Failed to read distance.")
        return detectobjectResponse(-1.0)  # Return -1.0 to indicate an error

def distance_server():
    """
    Initializes the distance reading server node.
    """
    rospy.init_node('distance_server')
    service = rospy.Service('read_distance', detectobject, handle_read_distance)
    rospy.loginfo("Distance server ready.")
    rospy.spin()

if __name__ == "__main__":
    distance_server()
