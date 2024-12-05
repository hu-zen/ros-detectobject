#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import DetectObject, DetectObjectResponse
import serial

def handle_detect_object(req):
    """
    Handle service requests to detect objects using the proximity sensor.
    """
    try:
        # Connect to the sensor (adjust port as needed)
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)  # Ensure the sensor port is correct
        ser.write(b'R')  # Send trigger if required
        raw_data = ser.readline().decode('utf-8').strip()  # Read sensor data
        ser.close()

        # Convert raw data to float
        distance = float(raw_data)
        rospy.loginfo(f"Distance: {distance} cm, Threshold: {req.threshold} cm")

        # Determine detection result
        result = "Object detected!" if distance <= req.threshold else "No object detected."
        return DetectObjectResponse(result)

    except serial.SerialException as e:
        rospy.logerr(f"Error connecting to sensor: {e}")
        return DetectObjectResponse("Error: Unable to connect to sensor.")
    except ValueError:
        rospy.logerr(f"Invalid data received from sensor: {raw_data}")
        return DetectObjectResponse("Error: Invalid sensor data.")
    except Exception as ex:
        rospy.logerr(f"Unexpected error: {ex}")
        return DetectObjectResponse("Error: Unexpected issue.")

def detect_object_server():
    """
    Initialize the detect object server.
    """
    rospy.init_node('detect_object_server')
    service = rospy.Service('detect_object', DetectObject, handle_detect_object)
    rospy.loginfo("Detect Object Server is ready.")
    rospy.spin()

if _name_ == "_main_":
    detect_object_server()
