#!/usr/bin/env python3
import rospy
import subprocess
import os
import signal
from std_msgs.msg import String

# Variabel global untuk menyimpan proses yang sedang berjalan
gazebo_process = None
navigation_process = None

def stop_simulation():
    """Menghentikan semua proses simulasi yang sedang berjalan."""
    global gazebo_process, navigation_process

    # Hentikan navigasi terlebih dahulu
    if navigation_process:
        rospy.loginfo("Menghentikan proses navigasi...")
        os.killpg(os.getpgid(navigation_process.pid), signal.SIGINT)
        navigation_process.wait()
        navigation_process = None

    # Kemudian hentikan Gazebo
    if gazebo_process:
        rospy.loginfo("Menghentikan proses Gazebo...")
        os.killpg(os.getpgid(gazebo_process.pid), signal.SIGINT)
        gazebo_process.wait()
        gazebo_process = None
    rospy.loginfo("Semua simulasi telah dihentikan.")

def start_simulation(map_name):
    """Memulai Gazebo dan stack navigasi."""
    global gazebo_process, navigation_process

    stop_simulation() # Selalu hentikan yang lama sebelum memulai yang baru
    rospy.sleep(2) # Beri jeda singkat

    ros_env = os.environ.copy()

    try:
        # --- 1. Jalankan Gazebo ---
        # Perintah ini akan meluncurkan dunia virtual dan model robot TurtleBot3
        gazebo_command = ["roslaunch", "turtlebot3_gazebo", "turtlebot3_world.launch"]
        rospy.loginfo("Manajer: Menjalankan Gazebo...")
        gazebo_process = subprocess.Popen(gazebo_command, env=ros_env, preexec_fn=os.setsid)

        # --- Beri waktu yang cukup agar Gazebo siap ---
        rospy.loginfo("Manajer: Menunggu Gazebo siap selama 10 detik...")
        rospy.sleep(10)

        # --- 2. Jalankan Navigasi (termasuk Rviz dan map_server) ---
        # Perintah ini akan memuat peta, AMCL, move_base, dan Rviz
        nav_command = ["roslaunch", "turtlebot3_navigation", "turtlebot3_navigation.launch"]
        rospy.loginfo("Manajer: Menjalankan Navigasi...")
        navigation_process = subprocess.Popen(nav_command, env=ros_env, preexec_fn=os.setsid)

        rospy.loginfo("Manajer: Simulasi Gazebo dan Navigasi telah dieksekusi.")

    except Exception as e:
        rospy.logerr(f"Gagal menjalankan roslaunch: {e}")

def callback(msg):
    rospy.loginfo(f"Manajer: Menerima permintaan untuk peta '{msg.data}'")
    start_simulation(msg.data)

def main():
    rospy.init_node('sim_manager_node')
    rospy.Subscriber('/gui/select_map', String, callback)
    rospy.loginfo("Manajer Simulasi (Gazebo) siap.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        stop_simulation()
