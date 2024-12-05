import serial

def read_sensor():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)  # Pastikan baudrate sesuai
        print(f"Connected to sensor at {ser.port}")
        while True:
            raw_data = ser.readline().decode('utf-8').strip()
            if raw_data and raw_data.startswith('R'):  # Pastikan data dimulai dengan 'R'
                distance_mm = int(raw_data[1:])  # Ambil nilai setelah 'R' dan ubah ke integer
                distance_cm = distance_mm / 10.0  # Konversi ke cm
                print(f"Distance: {distance_cm} cm")
            else:
                print(f"Invalid Data: {raw_data}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    read_sensor()
