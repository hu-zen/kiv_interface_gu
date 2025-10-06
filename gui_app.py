
#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import mainthread
import numpy as np
import signal # Import modul signal

class MapWidget(Image):
    @mainthread
    def update_map_texture(self, map_data, width, height):
        if self.texture is None or self.texture.size != (width, height):
            self.texture = Texture.create(size=(width, height), colorfmt='luminance')
        
        map_data_flipped = np.flipud(map_data)
        self.texture.blit_buffer(map_data_flipped.tobytes(), colorfmt='luminance', bufferfmt='ubyte')
        self.canvas.ask_update()

class GuiApp(App):
    def __init__(self, **kwargs):
        super(GuiApp, self).__init__(**kwargs)
        self.map_publisher = rospy.Publisher('/gui/select_map', String, queue_size=10, latch=True)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.on_map_received)
        rospy.loginfo("GUI Kivy: Publisher & Subscriber siap.")
        self.map_widget = None

    def build(self):
        self.title = "Waiter-Bot Control"
        layout = BoxLayout(orientation='vertical')
        btn = Button(text="Jalankan Simulasi & Tampilkan Peta", size_hint_y=0.1)
        btn.bind(on_press=self.run_simulation)
        self.map_widget = MapWidget()
        
        layout.add_widget(btn)
        layout.add_widget(self.map_widget)
        return layout

    def run_simulation(self, instance):
        self.map_publisher.publish("turtlebot_world")
        print("GUI: Mengirim perintah 'turtlebot_world'")

    def on_map_received(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.uint8)

        data[data == -1] = 127 
        data[data == 0] = 255
        data[data == 100] = 0
        
        self.map_widget.update_map_texture(data, width, height)
        print("GUI: Peta diterima dan digambar.")

    def on_stop(self):
        # --- PERUBAHAN PENTING ---
        # Saat jendela Kivy ditutup, kirim sinyal shutdown ke ROS
        rospy.loginfo("Jendela Kivy ditutup, mengirim sinyal shutdown ke ROS...")
        rospy.signal_shutdown("Aplikasi Kivy ditutup oleh pengguna")

def run_kivy_app_in_thread():
    GuiApp().run()

if __name__ == '__main__':
    rospy.init_node('gui_node', anonymous=True, disable_signals=True)
    rospy.loginfo("Node GUI utama telah diinisialisasi.")
    
    kivy_thread = threading.Thread(target=run_kivy_app_in_thread)
    kivy_thread.daemon = True
    kivy_thread.start()
    
    try:
        # Loop ini menjaga agar program utama (ROS) tetap berjalan
        # dan bisa menangkap Ctrl+C
        while kivy_thread.is_alive() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    except KeyboardInterrupt:
        # --- PERUBAHAN PENTING ---
        # Jika pengguna menekan Ctrl+C di terminal, kirim sinyal shutdown
        rospy.loginfo("Ctrl+C terdeteksi, mengirim sinyal shutdown ke ROS...")
        rospy.signal_shutdown("Ctrl+C ditekan")
