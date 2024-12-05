import serial

def read_sensor():
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)  # Pastikan baudrate sesuai
    while True:
        raw_data = ser.readline().decode('utf-8').strip()
        print(f"Raw Data: {raw_data}")

if __name__ == "__main__":
    read_sensor()
