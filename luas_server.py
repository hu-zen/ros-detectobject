#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import CalculateArea, CalculateAreaResponse

def hitung_luas_server(request):
    """Fungsi ini menghitung luas tanah berbentuk lingkaran dari diameter yang diberikan."""
    diameter = request.diameter
    
    if diameter <= 0:
        rospy.logwarn("Diameter tidak valid! Harus lebih besar dari 0.")
        return CalculateAreaResponse(area=0.0)

    # Menghitung luas lingkaran menggunakan rumus πr²
    jari_jari = diameter / 2
    luas_total = 3.14159 * (jari_jari ** 2)  # Rumus luas lingkaran

    rospy.loginfo(f"Diameter diterima: {diameter} meter")
    rospy.loginfo(f"Luas tanah dihitung: {luas_total:.2f} m²")
    
    # Mengembalikan luas tanah ke klien
    return CalculateAreaResponse(area=luas_total)

def luas_server():
    """Inisialisasi node ROS dan layanan untuk menghitung luas tanah."""
    rospy.init_node('luas_server_node')  # Inisialisasi node server ROS
    rospy.Service('hitung_luas', CalculateArea, hitung_luas_server)  # Membuat layanan ROS
    rospy.loginfo("Server untuk menghitung luas tanah siap.")
    rospy.spin()  # Menunggu permintaan dari klien

if _name_ == "_main_":
    try:
        luas_server()
    except rospy.ROSInterruptException:
        pass
