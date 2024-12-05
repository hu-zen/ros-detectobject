import serial

def read_sensor_raw(port='/dev/ttyUSB0', baudrate=57600):
    """
    Membaca data mentah dari sensor MaxBotix MB1403 HRUSB.
    """
    try:
        # Hubungkan ke port serial
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to sensor at {port} with baudrate {baudrate}")

        while True:
            # Membaca data mentah dari sensor
            raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
            if raw_data:
                print(f"Raw Data: {raw_data}")  # Tampilkan data mentah
            else:
                print("Raw Data: (kosong)")  # Jika data kosong
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    # Jalankan fungsi untuk membaca data
    read_sensor_raw()
