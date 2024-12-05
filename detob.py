import serial

def read_sensor_cm(port='/dev/ttyUSB0', baudrate=57600, threshold=100):
    """
    Membaca data dari sensor MaxBotix HRUSB dan mengonversinya ke jarak dalam cm.
    Mengidentifikasi apakah ada objek berdasarkan threshold jarak.
    """
    try:
        # Hubungkan ke port serial
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to sensor at {port} with baudrate {baudrate}")

        while True:
            # Membaca data mentah dari sensor
            raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if raw_data.startswith('R') and raw_data[1:].isdigit():
                # Ekstrak angka dari data valid dan konversi ke cm
                mm_distance = int(raw_data[1:])  # Ambil angka setelah 'R'
                cm_distance = mm_distance / 10.0  # Konversi ke cm

                # Deteksi objek berdasarkan threshold
                if cm_distance < threshold:
                    print(f"Object Detected! Distance: {cm_distance:.1f} cm")
                else:
                    print(f"No Object. Distance: {cm_distance:.1f} cm")
            elif raw_data:  # Data ada tetapi tidak valid
                print(f"Invalid Data: {raw_data}")
            else:
                print("Raw Data: (kosong)")  # Jika data kosong
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if _name_ == "_main_":
    read_sensor_cm()
