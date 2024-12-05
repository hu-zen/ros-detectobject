#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import detectobject, detectobjectResponse
import serial

def handle_detect_object(req):
    try:
        # Hubungkan ke sensor dan baca data
        ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        raw_data = ser.readline().decode('utf-8').strip()
        rospy.loginfo(f"Raw data from sensor: {raw_data}")
        ser.close()

        # Cek dan konversi data
        if raw_data.startswith("R") and len(raw_data) == 5:
            distance = float(raw_data[1:]) / 10.0  # Convert from mm to cm
            rospy.loginfo(f"Distance: {distance} cm, Threshold: {req.threshold} cm")

            # Tentukan apakah ada objek berdasarkan threshold
            result = "ada objek" if distance <= req.threshold else "tidak ada objek"
            return detectobjectResponse(result)
        else:
            rospy.logwarn(f"Invalid data format: {raw_data}")
            return detectobjectResponse("Error: Invalid sensor data format.")
    except Exception as e:
        rospy.logerr(f"Error reading sensor: {e}")
        return detectobjectResponse("Error: Unable to read sensor data.")

def detect_object_server():
    rospy.init_node('detectobject_server')
    service = rospy.Service('detect_object', detectobject, handle_detect_object)
    rospy.loginfo("Detect Object Server is ready.")
    rospy.spin()

if __name__ == "__main__":
    detect_object_server()
