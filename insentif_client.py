#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import CalculateArea

def hitung_insentif(diameter, harga_per_m2):
    """Fungsi ini meminta luas tanah dari server dan menghitung insentif berdasarkan luas."""
    rospy.loginfo("Menunggu server luas tanah...")
    rospy.wait_for_service('hitung_luas')  # Menunggu layanan server tersedia
    
    try:
        # Membuat proxy untuk memanggil layanan
        hitung_luas = rospy.ServiceProxy('hitung_luas', CalculateArea)
        
        rospy.loginfo(f"Mengirim diameter: {diameter} meter ke server...")
        # Mengirim permintaan dengan diameter
        response = hitung_luas(diameter)
        
        # Mendapatkan luas tanah dari respons
        luas_tanah = response.area
        rospy.loginfo(f"Luas tanah yang diterima: {luas_tanah:.2f} m²")
        
        # Menghitung insentif berdasarkan luas tanah
        insentif = luas_tanah * harga_per_m2
        rospy.loginfo(f"Insentif yang dihitung: Rp {insentif:,.2f}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Gagal memanggil layanan: {e}")

if _name_ == "_main_":
    rospy.init_node('insentif_client_node')  # Inisialisasi node klien ROS
    rospy.loginfo("Client menghitung insentif telah dimulai.")
    
    # Contoh data yang ingin dihitung
    diameter = 360  # Diameter tanah dalam meter
    harga_per_m2 = 150000  # Harga insentif per m² dalam Rupiah
    
    hitung_insentif(diameter, harga_per_m2)
