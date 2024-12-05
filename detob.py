import serial
import time

def read_sensor_data(port='/dev/ttyUSB0', baudrate=9600):
    """
    Reads raw data from the sensor and processes it.
    """
    try:
        # Connect to the sensor
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2,  # Timeout for reading
            rtscts=False,
            dsrdtr=False
        )
        print(f"Connected to sensor at {port}")

        time.sleep(2)  # Allow sensor to stabilize

        while True:
            # Optional: Send trigger if required by your sensor
            ser.write(b'R')  # Send "R" to request data from the sensor
            time.sleep(0.1)  # Give the sensor time to respond

            # Read raw data from the sensor
            raw_data = ser.readline().decode('utf-8').strip()

            if raw_data:
                print("Raw Data:", raw_data)
                process_sensor_data(raw_data)  # Process the raw data
            else:
                print("No data received. Check sensor connection.")
    except serial.SerialException as e:
        print(f"Error connecting to sensor: {e}")
    except Exception as ex:
        print(f"An error occurred: {ex}")

def process_sensor_data(raw_data, threshold=30.0):
    """
    Processes the raw sensor data to determine if an object is detected.

    Args:
        raw_data (str): The raw data string from the sensor.
        threshold (float): The distance threshold for object detection.
    """
    try:
        # Clean up raw data and handle potential extra characters
        clean_data = raw_data.replace('R', '').strip()  # Remove any "R" prefix or suffix
        distance = float(clean_data)  # Convert to float

        print(f"Distance: {distance} cm")

        # Determine if an object is within the threshold distance
        if distance <= threshold:
            print("Object detected!")
        else:
            print("No object detected.")
    except ValueError:
        print(f"Invalid data received: {raw_data}")

if _name_ == "_main_":
    read_sensor_data()
