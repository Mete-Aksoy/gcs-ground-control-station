# =====================================================
# Ground Control Station (GCS)
# Author: Mete Aksoy
# Year: 2025
# All rights reserved.
# Unauthorized use or redistribution is prohibited.
# =====================================================

import sys
import time
import socket
import asyncio
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QLineEdit, QComboBox, QPushButton, QTextEdit, QGridLayout,
                             QTabWidget, QTableWidget, QTableWidgetItem, QMenu, QAction, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, QUrl, QObject, pyqtSlot, QThread, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
import pymavlink.mavutil as mavutil
import serial.tools.list_ports
import datetime
import numpy as np
import socket
import cv2
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

class VideoReceiverTCP(QThread):
    frame_received = pyqtSignal(QPixmap)
    fps_updated = pyqtSignal(float)

    def __init__(self, ip='0.0.0.0', port=5050):
        super().__init__()
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.last_time = time.time()
        try:
            self.sock.bind((self.ip, self.port))
            self.sock.listen(5)
            print(f"[INFO] Video alımı için bekleniyor: {self.ip}:{self.port}")
        except Exception as e:
            print(f"[HATA] Soket bağlantı hatası: {e}")
            self.sock.close()
            raise

    def run(self):
        try:
            conn, addr = self.sock.accept()
            print(f"[BAŞARILI] Video bağlantısı alındı: {addr}")
            buffer = b''

            while True:
                while len(buffer) < 4:
                    data = conn.recv(4096)
                    if not data:
                        conn.close()
                        return
                    buffer += data

                frame_len = int.from_bytes(buffer[:4], byteorder='big')
                buffer = buffer[4:]

                while len(buffer) < frame_len:
                    data = conn.recv(4096)
                    if not data:
                        conn.close()
                        break
                    buffer += data

                if len(buffer) < frame_len:
                    break

                frame_data = buffer[:frame_len]
                buffer = buffer[frame_len:]

                nparr = np.frombuffer(frame_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is None:
                    continue

                rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimg)

                current_time = time.time()
                fps = 1.0 / (current_time - self.last_time) if current_time > self.last_time else 0
                self.last_time = current_time

                self.frame_received.emit(pixmap)
                self.fps_updated.emit(fps)

        except Exception as e:
            print(f"[HATA] Video işleme hatası: {e}")
        finally:
            self.sock.close()
            print("[BİLGİ] Video soketi kapatıldı")            
                     
class HeartbeatMonitor(QThread):
    def __init__(self, uav_type, connection, telemetry, system_status, parent=None):
        super().__init__(parent)
        self.uav_type = uav_type
        self.connection = connection
        self.telemetry = telemetry
        self.system_status = system_status
        self.running = True
        self.mode_map = parent.rotary_id_to_mode if uav_type == "Sol Panel" else parent.fixed_id_to_mode
        self.last_armed = False
        self.last_custom_mode = -1
        self.last_mode_time = 0
        self.debounce_duration = 1
        self.parent = parent

    def run(self):
        while self.running and self.connection:
            try:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and msg.get_srcSystem() == self.connection.target_system:
                    self.system_status[0] = msg.system_status
                    new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode = self.mode_map.get(msg.custom_mode, "Unknown")
                    if new_armed != self.last_armed:
                        self.parent.log_message(self.uav_type, f"🟢 Motor status: {'ARM' if new_armed else 'DISARM'}")
                        self.last_armed = new_armed
                        self.telemetry["Armed"] = new_armed
                    if msg.custom_mode != self.last_custom_mode and (time.time() - self.last_mode_time) > self.debounce_duration:
                        self.parent.log_message(self.uav_type, f"🟢 Flight mode updated: {mode}")
                        self.telemetry["Flight Mode"] = mode
                        self.last_custom_mode = msg.custom_mode
                        self.last_mode_time = time.time()
                        if self.uav_type == "Sol Panel":
                            self.parent.left_flight_mode_updated = True
                        else:
                            self.parent.right_flight_mode_updated = True
                    self.parent.update_telemetry(self.uav_type, self.telemetry)
            except Exception as e:
                self.parent.log_message(self.uav_type, f"🔴 HEARTBEAT monitoring error: {e}")
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.wait()

class WebChannelHandler(QObject):
    def __init__(self, parent, uav_type, screen_type):
        super().__init__(parent)
        self.uav_type = uav_type
        self.screen_type = screen_type
        self.parent = parent

    @pyqtSlot(float, float)
    def add_waypoint(self, lat, lng):
        if self.screen_type == "PLAN":
            self.parent.add_waypoint(lat, lng, self.uav_type)

    @pyqtSlot(float, float)
    def show_context_menu(self, lat, lng):
        if self.screen_type == "PLAN":
            self.parent.show_context_menu(lat, lng, self.uav_type)
  
class GCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GCS")
        self.setGeometry(100, 100, 1200, 800)
        self.showMaximized()

        self.is_dark_mode = True

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        top_bar = QWidget()
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.addStretch()
        self.mode_btn = QPushButton("Aydınlık Mod")
        self.mode_btn.clicked.connect(self.toggle_mode)
        top_bar_layout.addWidget(self.mode_btn)
        self.main_layout.addWidget(top_bar)

        self.screens = QTabWidget()
        self.screens.setStyleSheet("""
            QTabWidget::pane {
                border: 2px solid #800080;
            }
            QTabBar::tab {
                background-color: #2E2E2E;
                color: #00FF00;
                font-weight: bold;
                font-size: 18px;
                padding: 10px 30px;
                border: 1px solid #800080;
            }
            QTabBar::tab:selected {
                background-color: #555555;
                color: #00FF00;
            }
        """)
        self.main_layout.addWidget(self.screens)

        self.data_screen = QWidget()
        self.data_layout = QHBoxLayout(self.data_screen)
        self.screens.addTab(self.data_screen, "DATA")

        self.plan_screen = QWidget()
        self.plan_layout = QHBoxLayout(self.plan_screen)
        self.screens.addTab(self.plan_screen, "PLAN")

        self.left_data_panel = QWidget()
        self.right_data_panel = QWidget()
        self.left_data_layout = QVBoxLayout(self.left_data_panel)
        self.right_data_layout = QVBoxLayout(self.right_data_panel)
        self.data_layout.addWidget(self.left_data_panel, stretch=1)

        self.yolo_panel = QWidget()
        self.yolo_layout = QVBoxLayout(self.yolo_panel)
        self.yolo_label = QLabel("YOLO Camera Output")
        self.yolo_label.setAlignment(Qt.AlignCenter)
        self.yolo_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #FFFFFF; background-color: #2E2E2E; border: 2px solid #800080;")
        self.yolo_layout.addWidget(self.yolo_label)
        self.yolo_placeholder = QLabel()
        self.yolo_placeholder.setStyleSheet("background-color: #000000; border: 2px solid #800080;")
        self.yolo_placeholder.setScaledContents(True)
        self.yolo_layout.addWidget(self.yolo_placeholder)
        self.data_layout.addWidget(self.yolo_panel, stretch=1)

        self.data_layout.addWidget(self.right_data_panel, stretch=1)

        self.left_plan_panel = QWidget()
        self.right_plan_panel = QWidget()
        self.left_plan_layout = QVBoxLayout(self.left_plan_panel)
        self.right_plan_layout = QVBoxLayout(self.right_plan_panel)
        self.plan_layout.addWidget(self.left_plan_panel)
        self.plan_layout.addWidget(self.right_plan_panel)

        self.create_connection_panel(self.left_data_layout, "Sol Panel", "14550", "5762")
        self.create_connection_panel(self.right_data_layout, "Sağ Panel", "14560", "5772")

        self.left_data_map = self.create_visualization_panel(self.left_data_layout, "Sol Panel", "DATA")
        self.right_data_map = self.create_visualization_panel(self.right_data_layout, "Sağ Panel", "DATA")

        self.left_data_tabs = self.create_data_tabs(self.left_data_layout, "Sol Panel")
        self.right_data_tabs = self.create_data_tabs(self.right_data_layout, "Sağ Panel")
        self.create_telemetry_control_panel(self.left_data_layout, "Sol Panel")
        self.create_telemetry_control_panel(self.right_data_layout, "Sağ Panel")

        self.left_plan_map = self.create_visualization_panel(self.left_plan_layout, "Sol Panel", "PLAN")
        self.right_plan_map = self.create_visualization_panel(self.right_plan_layout, "Sağ Panel", "PLAN")

        self.left_plan_table = self.create_plan_table(self.left_plan_layout, "Sol Panel")
        self.right_plan_table = self.create_plan_table(self.right_plan_layout, "Sağ Panel")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)

        self.left_connection = None
        self.right_connection = None

        self.left_telemetry = {
            "Altitude": 0.0, "Ground Speed": 0.0, "Dist to WP": 0.0, 
            "Vertical Speed": 0.0, "Yaw": 0.0, "Dist to MAV": 0.0,
            "Latitude": 0.0, "Longitude": 0.0, "GPS Direction": 0.0,
            "Armed": False, "Flight Mode": "Bilinmiyor"
        }
        self.right_telemetry = {
            "Altitude": 0.0, "Ground Speed": 0.0, "Dist to WP": 0.0, 
            "Vertical Speed": 0.0, "Yaw": 0.0, "Dist to MAV": 0.0,
            "Latitude": 0.0, "Longitude": 0.0, "GPS Direction": 0.0,
            "Armed": False, "Flight Mode": "Bilinmiyor"
        }

        self.left_gps_connected = False
        self.right_gps_connected = False

        self.left_gps_waiting_logged = False
        self.right_gps_waiting_logged = False
        self.left_data_updating_logged = False
        self.right_data_updating_logged = False
        self.left_gps_timeout_counter = 0
        self.right_gps_timeout_counter = 0
        self.GPS_TIMEOUT_LIMIT = 600

        self.left_arm_status_updated = False
        self.right_arm_status_updated = False
        self.left_flight_mode_updated = False
        self.right_flight_mode_updated = False

        self.left_waypoints = []
        self.right_waypoints = []
        self.left_current_waypoint = 0
        self.right_current_waypoint = 0

        self.last_left_mode = "Bilinmiyor"
        self.last_right_mode = "Bilinmiyor"
        self.last_left_armed = False
        self.last_right_armed = False
        self.last_left_custom_mode = -1
        self.last_right_custom_mode = -1

        self.last_left_mode_time = 0
        self.last_right_mode_time = 0
        self.left_system_status = [-1]
        self.right_system_status = [-1]

        self.debounce_duration = 1

        self.left_manual_disconnect = False
        self.right_manual_disconnect = False

        self.left_connection_params = None
        self.right_connection_params = None

        self.left_system_ready = False
        self.right_system_ready = False

        self.left_heartbeat_monitor = None
        self.right_heartbeat_monitor = None

        self.rotary_id_to_mode = {
            0: "STABILIZE", 1: "MANUAL", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 13: "FBWA", 21: "RTL"
        }
        self.fixed_id_to_mode = {
            0: "MANUAL", 1: "ACRO", 2: "STABILIZE", 5: "FBWA", 11: "RTL", 12: "FBWB", 13: "AUTOTUNE",
            15: "AUTO", 16: "LOITER", 17: "CRUISE", 18: "TAKEOFF", 19: "AVOID_ADSB", 20: "GUIDED",
            21: "INITIALISING", 22: "QSTABILIZE", 23: "QHOVER", 24: "QLOITER", 25: "QLAND",
            26: "QRTL", 27: "QAUTOTUNE", 28: "QACRO", 29: "THERMAL", 30: "LOITER to QLAND"
        }

        self.update_ui_mode()
        self.video_receiver = VideoReceiverTCP()
        self.video_receiver.frame_received.connect(self.yolo_placeholder.setPixmap)
        self.video_receiver.start()

    def run_async_task(self, coro):
        def thread_task():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(coro)
            finally:
                loop.close()
        threading.Thread(target=thread_task, daemon=True).start()

    async def wait_for_ack_async(self, connection, command_id, timeout=7):
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.05)
            if msg and msg.command == command_id:
                return msg
            elif msg:
                self.log_message("Debug", f"🔵 Alınan ACK yanlış: command={msg.command}, beklenen={command_id}")
            await asyncio.sleep(0.05)
        return None

    async def auto_mode_verification(self, uav_type, connection):
        self.log_message(uav_type, "🔵 Verifying AUTO mode via HEARTBEAT (custom_mode == 3, system_status >= MAV_STATE_ACTIVE)...")
        start_time = time.time()
        mode_count = 0
        system_status_verified = False
        while time.time() - start_time < 5:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == connection.target_system:
                if msg.custom_mode == 3:
                    mode_count += 1
                    if msg.system_status >= mavutil.mavlink.MAV_STATE_ACTIVE:
                        system_status_verified = True
                        self.log_message(uav_type, f"🟢 System status verified: {msg.system_status} (>= MAV_STATE_ACTIVE)")
                    if mode_count >= 3 and system_status_verified:
                        self.log_message(uav_type, "🟢 AUTO mode verified via HEARTBEAT (custom_mode == 3, 3 consecutive confirmations, system_status active)")
                        return True
                else:
                    mode_count = 0
                    self.log_message(uav_type, f"🟡 HEARTBEAT custom_mode={msg.custom_mode} (expected 3), resetting count...")
            else:
                self.log_message(uav_type, "🟡 No HEARTBEAT received in this cycle, continuing...")
            await asyncio.sleep(0.1)
        self.log_message(uav_type, "🔴 AUTO mode not verified via HEARTBEAT within 5 seconds.")
        return False

    def toggle_mode(self):
        self.is_dark_mode = not self.is_dark_mode
        self.update_ui_mode()
        if hasattr(self, 'video_receiver'):
            self.video_receiver.terminate()
            self.video_receiver.wait()
        self.video_receiver = VideoReceiverTCP()
        self.video_receiver.frame_received.connect(self.update_yolo_view)
        self.video_receiver.start()

    def update_yolo_view(self, pixmap):
        self.yolo_placeholder.setPixmap(pixmap)

    def update_ui_mode(self):
        if self.is_dark_mode:
            self.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF;")
            self.mode_btn.setText("Aydınlık Mod")
            self.mode_btn.setStyleSheet("background-color: #555555; color: #FFFFFF; border: 1px solid #800080;")
            for widget in self.left_data_panel.findChildren(QWidget) + self.right_data_panel.findChildren(QWidget) + \
                          self.left_plan_panel.findChildren(QWidget) + self.right_plan_panel.findChildren(QWidget):
                if isinstance(widget, (QLabel, QTextEdit, QTabWidget, QTableWidget)):
                    widget.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF; border: 2px solid #800080;")
                elif isinstance(widget, (QComboBox, QLineEdit)):
                    widget.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF; border: 1px solid #800080;")
                elif isinstance(widget, QPushButton):
                    widget.setStyleSheet("background-color: #555555; color: #FFFFFF; border: 1px solid #800080;")
        else:
            self.setStyleSheet("background-color: #FFFFFF; color: #000000;")
            self.mode_btn.setText("Karanlık Mod")
            self.mode_btn.setStyleSheet("background-color: #CCCCCC; color: #000000; border: 1px solid #800080;")
            for widget in self.left_data_panel.findChildren(QWidget) + self.right_data_panel.findChildren(QWidget) + \
                          self.left_plan_panel.findChildren(QWidget) + self.right_plan_panel.findChildren(QWidget):
                if isinstance(widget, (QLabel, QLineEdit, QComboBox, QTextEdit, QTabWidget, QTableWidget)):
                    widget.setStyleSheet("background-color: #FFFFFF; color: #000000; border: 1px solid #800080;")
                elif isinstance(widget, QPushButton):
                    widget.setStyleSheet("background-color: #CCCCCC; color: #000000; border: 1px solid #800080;")

    def create_connection_panel(self, layout, uav_type, default_port, default_remote_port):
        connection_frame = QWidget()
        connection_layout = QHBoxLayout(connection_frame)

        connection_type = QComboBox()
        connection_type.addItems(["UDP", "TCP", "Seri Port"])
        connection_type.setCurrentText("TCP")
        connection_type.currentTextChanged.connect(lambda text: self.update_connection_fields(text, connection_frame, uav_type))

        port_label = QLabel("Port:")
        port_edit = QLineEdit(default_port)
        port_edit.setEnabled(True)

        host_label = QLabel("Host IP:")
        host_edit = QLineEdit("127.0.0.1")
        host_edit.setEnabled(True)

        remote_port_label = QLabel("Remote Port:")
        remote_port_edit = QLineEdit(default_remote_port)
        remote_port_edit.setEnabled(True)

        com_port_label = QLabel("COM Port:")
        com_port_combo = QComboBox()
        available_ports = [port.device for port in serial.tools.list_ports.comports()]
        com_port_combo.addItems(available_ports if available_ports else ["Port Bulunamadı"])
        com_port_combo.setCurrentText("COM3" if "COM3" in available_ports else available_ports[0] if available_ports else "Port Bulunamadı")

        baud_label = QLabel("Baud Hızı:")
        baud_edit = QLineEdit("115200")
        baud_edit.setEnabled(True)

        connect_btn = QPushButton("BAĞLAN")
        connect_btn.clicked.connect(lambda: self.connect_uav(uav_type, host_edit.text(), port_edit.text(), connection_type.currentText(), remote_port_edit.text(), com_port_combo.currentText(), baud_edit.text()))
        disconnect_btn = QPushButton("KES")
        disconnect_btn.clicked.connect(lambda: self.disconnect_uav(uav_type))

        connection_layout.addWidget(connection_type)
        connection_layout.addWidget(port_label)
        connection_layout.addWidget(port_edit)
        connection_layout.addWidget(host_label)
        connection_layout.addWidget(host_edit)
        connection_layout.addWidget(remote_port_label)
        connection_layout.addWidget(remote_port_edit)
        connection_layout.addWidget(com_port_label)
        connection_layout.addWidget(com_port_combo)
        connection_layout.addWidget(baud_label)
        connection_layout.addWidget(baud_edit)
        connection_layout.addWidget(connect_btn)
        connection_layout.addWidget(disconnect_btn)

        self.update_connection_fields("TCP", connection_frame, uav_type)

        connection_frame.setStyleSheet("border: 2px solid #800080; padding: 5px;")
        layout.addWidget(connection_frame)

    def update_connection_fields(self, text, frame, uav_type):
        for widget in frame.children():
            if isinstance(widget, QLabel) and widget.text() in ["Port:"]:
                widget.setVisible(text == "UDP")
            elif isinstance(widget, QLineEdit) and widget.text() in ["14550", "14560"]:
                widget.setVisible(text == "UDP")
            elif isinstance(widget, QLabel) and widget.text() in ["Host IP:", "Remote Port:"]:
                widget.setVisible(text == "TCP")
            elif isinstance(widget, QLineEdit) and widget.text() in ["127.0.0.1", "5762", "5772"]:
                widget.setVisible(text == "TCP")
            elif isinstance(widget, QLabel) and widget.text() in ["COM Port:", "Baud Hızı:"]:
                widget.setVisible(text == "Seri Port")
            elif isinstance(widget, QComboBox) and widget.currentText().startswith("COM"):
                widget.setVisible(text == "Seri Port")
            elif isinstance(widget, QLineEdit) and widget.text() == "115200":
                widget.setVisible(text == "Seri Port")

    def connect_uav(self, uav_type, host, port, protocol, remote_port, com_port, baud_rate):
        connection_params = {
            "host": host,
            "port": port,
            "protocol": protocol,
            "remote_port": remote_port,
            "com_port": com_port,
            "baud_rate": baud_rate
        }
        if uav_type == "Sol Panel":
            self.left_connection_params = connection_params
            self.left_system_ready = False
        else:
            self.right_connection_params = connection_params
            self.right_system_ready = False

        try:
            self.log_message(uav_type, "🟡 Bağlantı kuruluyor...", is_processing=True)

            if protocol == "UDP":
                if not port:
                    raise ValueError("UDP için Port gereklidir.")
                resolved_host = socket.getaddrinfo(host or "127.0.0.1", None, socket.AF_INET)[0][4][0]
                test_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                test_socket.bind(('', int(port)))
                self.log_message(uav_type, f"🔵 UDP bağlantı: {resolved_host}:{port}")
                test_socket.close()
                connection_string = f"udpin:{resolved_host}:{port}"
            elif protocol == "TCP":
                if not remote_port:
                    raise ValueError("TCP için Remote Port gereklidir.")
                if not host:
                    raise ValueError("TCP için Host IP gereklidir.")
                resolved_host = socket.getaddrinfo(host, None, socket.AF_INET)[0][4][0]
                test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                test_socket.settimeout(5)
                test_socket.connect((resolved_host, int(remote_port)))
                self.log_message(uav_type, f"🔵 TCP bağlantı: {resolved_host}:{remote_port}")
                test_socket.close()
                connection_string = f"tcp:{resolved_host}:{remote_port}"
            elif protocol == "Seri Port":
                if not com_port or com_port == "Port Bulunamadı":
                    raise ValueError("Geçerli bir COM port seçmelisiniz.")
                if not baud_rate:
                    raise ValueError("Baud Hızı gereklidir.")
                connection_string = com_port
                baud_rate = int(baud_rate)
                self.log_message(uav_type, f"🔵 Seri Port bağlantı: {com_port}, Baud: {baud_rate}")
            else:
                raise ValueError("Geçersiz protokol türü.")

            max_retries = 3
            for attempt in range(max_retries):
                try:
                    if uav_type == "Sol Panel":
                        self.left_connection = mavutil.mavlink_connection(
                            connection_string, 
                            baud=baud_rate if protocol == "Seri Port" else None, 
                            source_system=254, 
                            source_component=1,
                            autoreconnect=True, 
                            dialect="ardupilotmega"
                        )
                        self.log_message(uav_type, "🔵 Heartbeat bekleniyor...")
                        self.left_connection.wait_heartbeat(timeout=5)
                        self.log_message(uav_type, f"🟢 Bağlantı başarılı! ({protocol})")
                        self.left_gps_waiting_logged = False
                        self.left_data_updating_logged = False
                        self.left_gps_timeout_counter = 0
                        self.left_arm_status_updated = False
                        self.left_flight_mode_updated = False
                        self.left_manual_disconnect = False
                        self.wait_for_system_ready(uav_type, 5)
                        self.left_heartbeat_monitor = HeartbeatMonitor(uav_type, self.left_connection, self.left_telemetry, self.left_system_status, self)
                        self.left_heartbeat_monitor.start()
                        break
                    elif uav_type == "Sağ Panel":
                        self.right_connection = mavutil.mavlink_connection(
                            connection_string, 
                            baud=baud_rate if protocol == "Seri Port" else None, 
                            source_system=254, 
                            source_component=1,
                            autoreconnect=True, 
                            dialect="ardupilotmega"
                        )
                        self.log_message(uav_type, "🔵 Heartbeat bekleniyor...")
                        self.right_connection.wait_heartbeat(timeout=5)
                        self.log_message(uav_type, f"🟢 Bağlantı başarılı! ({protocol})")
                        self.right_gps_waiting_logged = False
                        self.right_data_updating_logged = False
                        self.right_gps_timeout_counter = 0
                        self.right_arm_status_updated = False
                        self.right_flight_mode_updated = False
                        self.right_manual_disconnect = False
                        self.wait_for_system_ready(uav_type, 5)
                        self.right_heartbeat_monitor = HeartbeatMonitor(uav_type, self.right_connection, self.right_telemetry, self.right_system_status, self)
                        self.right_heartbeat_monitor.start()
                        break
                except Exception as e:
                    if attempt < max_retries - 1:
                        self.log_message(uav_type, f"🔵 Bağlantı denemesi {attempt + 1}/{max_retries} başarısız, tekrar deneniyor...")
                        time.sleep(1)
                        continue
                    else:
                        raise Exception(f"Bağlantı başarısız oldu: {e}")
        except Exception as e:
            self.log_message(uav_type, f"🔴 Bağlantı hatası: {e}")
            if "getaddrinfo failed" in str(e):
                self.log_message(uav_type, "🔴 Lütfen host IP ve port/remote port değerlerini kontrol edin.")
            elif isinstance(e, ValueError):
                self.log_message(uav_type, f"🔴 {str(e)}")
            if uav_type == "Sol Panel":
                self.left_connection = None
            elif uav_type == "Sağ Panel":
                self.right_connection = None

    def wait_for_system_ready(self, uav_type, timeout):
        connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                self.log_message(uav_type, f"🔵 HEARTBEAT alındı: system_status={msg.system_status}, base_mode={msg.base_mode}, custom_mode={msg.custom_mode}")
                if msg.system_status >= mavutil.mavlink.MAV_STATE_STANDBY:
                    self.log_message(uav_type, "🟢 Sistem hazır: Komutlar gönderilebilir!")
                    if uav_type == "Sol Panel":
                        self.left_system_ready = True
                        self.left_system_status[0] = msg.system_status
                    else:
                        self.right_system_ready = True
                        self.right_system_status[0] = msg.system_status
                    return True
            time.sleep(0.1)
        self.log_message(uav_type, "🔴 Sistem stabil hale gelemedi!")
        if uav_type == "Sol Panel":
            self.left_system_ready = False
            self.left_system_status[0] = -1
        else:
            self.right_system_ready = False
            self.right_system_status[0] = -1
        return False

    def disconnect_uav(self, uav_type):
        if uav_type == "Sol Panel" and self.left_connection:
            self.log_message(uav_type, "🟡 Bağlantı kesiliyor...", is_processing=True)
            if self.left_heartbeat_monitor:
                self.left_heartbeat_monitor.stop()
                self.left_heartbeat_monitor = None
            self.left_connection.close()
            self.left_connection = None
            self.left_gps_connected = False
            self.left_system_ready = False
            self.left_system_status[0] = -1
            self.left_telemetry["GPS Direction"] = 0.0
            self.left_telemetry["Armed"] = False
            self.left_telemetry["Flight Mode"] = "Bilinmiyor"
            self.left_gps_waiting_logged = False
            self.left_data_updating_logged = False
            self.left_gps_timeout_counter = 0
            self.left_arm_status_updated = False
            self.left_flight_mode_updated = False
            self.left_waypoints = []
            self.left_current_waypoint = 0
            self.left_manual_disconnect = True
            self.update_telemetry("Sol Panel", self.left_telemetry)
            self.log_message(uav_type, "🔴 Bağlantı kesildi!")
            self.update_plan_table(uav_type)
        elif uav_type == "Sağ Panel" and self.right_connection:
            self.log_message(uav_type, "🟡 Bağlantı kesiliyor...", is_processing=True)
            if self.right_heartbeat_monitor:
                self.right_heartbeat_monitor.stop()
                self.right_heartbeat_monitor = None
            self.right_connection.close()
            self.right_connection = None
            self.right_gps_connected = False
            self.right_system_ready = False
            self.right_system_status[0] = -1
            self.right_telemetry["GPS Direction"] = 0.0
            self.right_telemetry["Armed"] = False
            self.right_telemetry["Flight Mode"] = "Bilinmiyor"
            self.right_gps_waiting_logged = False
            self.right_data_updating_logged = False
            self.right_gps_timeout_counter = 0
            self.right_arm_status_updated = False
            self.right_flight_mode_updated = False
            self.right_waypoints = []
            self.right_current_waypoint = 0
            self.right_manual_disconnect = True
            self.update_telemetry("Sağ Panel", self.right_telemetry)
            self.log_message(uav_type, "🔴 Bağlantı kesildi!")
            self.update_plan_table(uav_type)

    def create_visualization_panel(self, layout, uav_type, screen_type):
        vis_container = QWidget()
        vis_layout = QVBoxLayout(vis_container)

        map_widget = QWebEngineView()
        map_widget.setMinimumHeight(300)
        map_widget.setMinimumWidth(400)

        channel = QWebChannel(map_widget.page())
        handler = WebChannelHandler(self, uav_type, screen_type)
        channel.registerObject("handler", handler)
        map_widget.page().setWebChannel(channel)

        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
            <style>
                html, body, #map { height: 100%; width: 100%; margin: 0; }
                #map { min-height: 300px; }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var mapInitialized = false;
                var map;
                var gpsMarker = null;
                var pixhawkMarker = null;
                var gpsDirectionLine = null;
                var pixhawkDirectionLine = null;
                var waypoints = [];
                var waypointLines = [];
                var waypointMarkers = [];

                function initializeMap() {
                    try {
                        map = L.map('map').setView([39.925533, 32.866287], 6);
                        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                            attribution: '© OpenStreetMap contributors',
                            maxZoom: 19
                        }).on('tileerror', function(error, tile) {
                            console.log('Tile loading error:', error, tile);
                        }).addTo(map);
                        mapInitialized = true;
                        console.log('Map initialized successfully');
                    } catch (e) {
                        console.error('Map initialization failed:', e);
                    }
                }

                function updateMarkers(gpsLat, gpsLon, pixhawkLat, pixhawkLon, pixhawkYaw, gpsDirection) {
                    if (!mapInitialized) {
                        console.error('Map not initialized for updateMarkers');
                        return;
                    }
                    if (!isFinite(gpsLat) || !isFinite(gpsLon) || !isFinite(pixhawkLat) || !isFinite(pixhawkLon)) {
                        console.error('Invalid coordinates:', gpsLat, gpsLon, pixhawkLat, pixhawkLon);
                        return;
                    }
                    try {
                        if (gpsMarker) {
                            gpsMarker.setLatLng([gpsLat, gpsLon]);
                        } else {
                            gpsMarker = L.marker([gpsLat, gpsLon], {
                                icon: L.divIcon({className: 'gps-icon'})
                            }).addTo(map);
                        }
                        if (pixhawkMarker) {
                            pixhawkMarker.setLatLng([pixhawkLat, pixhawkLon]);
                        } else {
                            pixhawkMarker = L.marker([pixhawkLat, pixhawkLon], {
                                icon: L.divIcon({className: 'pixhawk-icon'})
                            }).addTo(map);
                        }
                        map.setView([pixhawkLat, pixhawkLon], 18);
                        if (gpsDirectionLine) {
                            map.removeLayer(gpsDirectionLine);
                        }
                        var gpsLength = 0.1 * (map.getZoom() / 10);
                        var gpsEndLat = gpsLat + gpsLength * Math.cos((gpsDirection - 90) * Math.PI / 180);
                        var gpsEndLon = gpsLon + gpsLength * Math.sin((gpsDirection - 90) * Math.PI / 180);
                        gpsDirectionLine = L.polyline([[gpsLat, gpsLon], [gpsEndLat, gpsEndLon]], {
                            color: 'black',
                            weight: 2
                        }).addTo(map);
                        if (pixhawkDirectionLine) {
                            map.removeLayer(pixhawkDirectionLine);
                        }
                        var pixhawkLength = 0.1 * (map.getZoom() / 10);
                        var pixhawkEndLat = pixhawkLat + pixhawkLength * Math.cos((pixhawkYaw - 90) * Math.PI / 180);
                        var pixhawkEndLon = pixhawkLon + pixhawkLength * Math.sin((pixhawkYaw - 90) * Math.PI / 180);
                        pixhawkDirectionLine = L.polyline([[pixhawkLat, pixhawkLon], [pixhawkEndLat, pixhawkEndLon]], {
                            color: 'red',
                            weight: 2
                        }).addTo(map);
                    } catch (e) {
                        console.error('Error in updateMarkers:', e);
                    }
                }

                function addWaypoint(lat, lng) {
                    if (!mapInitialized) {
                        console.error('Map not initialized for addWaypoint');
                        return;
                    }
                    if (!isFinite(lat) || !isFinite(lng)) {
                        console.error('Invalid waypoint coordinates:', lat, lng);
                        return;
                    }
                    try {
                        waypoints.push([lat, lng]);
                        var marker = L.marker([lat, lng], {
                            icon: L.divIcon({className: 'waypoint-icon'})
                        }).addTo(map);
                        marker.bindPopup("Waypoint " + waypoints.length);
                        waypointMarkers.push(marker);
                        updateWaypointLines();
                    } catch (e) {
                        console.error('Error in addWaypoint:', e);
                    }
                }

                function updateWaypointLines() {
                    if (!mapInitialized) {
                        console.error('Map not initialized for updateWaypointLines');
                        return;
                    }
                    try {
                        waypointLines.forEach(line => map.removeLayer(line));
                        waypointLines = [];
                        for (let i = 0; i < waypoints.length - 1; i++) {
                            var line = L.polyline([waypoints[i], waypoints[i + 1]], {
                                color: 'blue',
                                weight: 2
                            }).addTo(map);
                            waypointLines.push(line);
                        }
                    } catch (e) {
                        console.error('Error in updateWaypointLines:', e);
                    }
                }

                function clearWaypoints() {
                    if (!mapInitialized) {
                        console.error('Map not initialized for clearWaypoints');
                        return;
                    }
                    try {
                        waypoints = [];
                        waypointMarkers.forEach(marker => map.removeLayer(marker));
                        waypointMarkers = [];
                        waypointLines.forEach(line => map.removeLayer(line));
                        waypointLines = [];
                    } catch (e) {
                        console.error('Error in clearWaypoints:', e);
                    }
                }

                new QWebChannel(qt.webChannelTransport, function(channel) {
                    window.handler = channel.objects.handler;
                    map.on('contextmenu', function(e) {
                        var lat = e.latlng.lat;
                        var lng = e.latlng.lng;
                        window.handler.show_context_menu(lat, lng);
                    });
                    map.on('click', function(e) {
                        var lat = e.latlng.lat;
                        var lng = e.latlng.lng;
                        window.handler.add_waypoint(lat, lng);
                    });
                });

                var style = document.createElement('style');
                style.type = 'text/css';
                style.innerHTML = '.gps-icon { background-color: black; width: 10px; height: 10px; border-radius: 50%; }' +
                                 '.pixhawk-icon { background-color: blue; width: 10px; height: 10px; border-radius: 50%; }' +
                                 '.waypoint-icon { background-color: green; width: 8px; height: 8px; border-radius: 50%; }';
                document.head.appendChild(style);

                window.onload = initializeMap;
            </script>
        </body>
        </html>
        """
        map_widget.setHtml(html)
        vis_layout.addWidget(map_widget)
        layout.addWidget(vis_container)
        return map_widget

    def create_data_tabs(self, layout, uav_type):
        tabs = QTabWidget()
        tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 2px solid #800080;
            }
            QTabBar::tab {
                background-color: #2E2E2E;
                color: #00FF00;
                font-weight: bold;
                font-size: 18px;
                padding: 10px 30px;
                border: 1px solid #800080;
            }
            QTabBar::tab:selected {
                background-color: #555555;
                color: #00FF00;
            }
        """)
        self.mavlink_logs = QTextEdit()
        self.mavlink_logs.setReadOnly(True)
        self.mavlink_logs.setPlaceholderText(f"{uav_type} MAVLink Logları (Anlık Veriler)")
        self.mavlink_logs.setStyleSheet("min-height: 100px; font-family: 'Arial'; font-size: 12px; border: 2px solid #800080;")
        self.mavlink_logs.setHtml("""
            <style>
                .loader {
                    display: inline-block;
                    width: 16px;
                    height: 16px;
                    border: 2px solid #f3f3f3;
                    border-top: 2px solid #3498db;
                    border-radius: 50%;
                    animation: spin 1s linear infinite;
                    vertical-align: middle;
                    margin-left: 5px;
                }
                @keyframes spin {
                    0% { transform: rotate(0deg); }
                    100% { transform: rotate(360deg); }
                }
            </style>
        """)
        tabs.addTab(self.mavlink_logs, "Loglar")

        button_layout = QHBoxLayout()
        save_btn = QPushButton("Kaydet")
        save_btn.setStyleSheet("background-color: #000000; color: #FFFFFF; border: 1px solid #800080;")
        save_btn.clicked.connect(lambda: self.save_logs(uav_type))
        clear_btn = QPushButton("Temizle")
        clear_btn.setStyleSheet("background-color: #000000; color: #FFFFFF; border: 1px solid #800080;")
        clear_btn.clicked.connect(lambda: self.clear_logs(uav_type))
        button_layout.addStretch()
        button_layout.addWidget(save_btn)
        button_layout.addWidget(clear_btn)

        layout.addWidget(tabs)
        layout.addLayout(button_layout)

        if uav_type == "Sol Panel":
            self.left_mavlink_logs = self.mavlink_logs
        elif uav_type == "Sağ Panel":
            self.right_mavlink_logs = self.mavlink_logs

        return tabs

    def save_logs(self, uav_type):
        logs = self.left_mavlink_logs.toPlainText() if uav_type == "Sol Panel" else self.right_mavlink_logs.toPlainText()
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{uav_type.replace(' ', '_')}_logs_{timestamp}.txt"
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(logs)
        self.log_message(uav_type, f"🟢 Loglar {filename} dosyasına kaydedildi.")

    def clear_logs(self, uav_type):
        reply = QMessageBox.question(self, 'Logları Temizle', 'Tüm logları silmek istediğinizden emin misiniz?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if uav_type == "Sol Panel":
                self.left_mavlink_logs.clear()
            else:
                self.right_mavlink_logs.clear()
            self.log_message(uav_type, "🟢 Loglar temizlendi.")

    def create_plan_table(self, layout, uav_type):
        table_container = QWidget()
        table_layout = QVBoxLayout(table_container)

        table = QTableWidget()
        table.setColumnCount(8)
        table.setHorizontalHeaderLabels(["Seq", "Command", "Lat", "Lon", "Alt (m)", "Speed (m/s)", "Delay (s)", "Delete"])
        table.setRowCount(0)
        table.setMinimumHeight(200)
        table.setColumnWidth(0, 50)
        table.setColumnWidth(1, 100)
        table.setColumnWidth(2, 100)
        table.setColumnWidth(3, 100)
        table.setColumnWidth(4, 60)
        table.setColumnWidth(5, 80)
        table.setColumnWidth(6, 60)
        table.setColumnWidth(7, 60)
        table.setStyleSheet("""
            QTableWidget::item { background-color: #FFFFFF; color: #000000; }
            QHeaderView::section { background-color: #FFFFFF; color: #000000; }
        """)
        table_layout.addWidget(table)

        button_layout = QHBoxLayout()
        write_btn = QPushButton("Write")
        write_btn.clicked.connect(lambda: self.run_async_task(self.write_waypoints(uav_type)))
        button_layout.addWidget(write_btn)
        table_layout.addLayout(button_layout)

        layout.addWidget(table_container)
        if uav_type == "Sol Panel":
            self.left_plan_table = table
        elif uav_type == "Sağ Panel":
            self.right_plan_table = table
        return table

    def log_message(self, uav_type, message, is_processing=False):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        color = "black"
        if "🟢" in message:
            color = "green"
        elif "🔴" in message:
            color = "red"
        elif "🟡" in message:
            color = "orange"
        elif "🔵" in message:
            color = "blue"

        formatted_message = f'<span style="color: #00FF00;">[{timestamp}]</span> <span style="color: {color};">{message}</span>'

        if uav_type == "Sol Panel" and hasattr(self, 'left_mavlink_logs'):
            if is_processing:
                self.left_mavlink_logs.append(formatted_message + " <span class='loader'></span>")
            else:
                current_text = self.left_mavlink_logs.toHtml()
                if "loader" in current_text:
                    current_text = current_text.replace(" <span class='loader'></span>", "")
                    self.left_mavlink_logs.setHtml(current_text)
                self.left_mavlink_logs.append(formatted_message)
        elif uav_type == "Sağ Panel" and hasattr(self, 'right_mavlink_logs'):
            if is_processing:
                self.right_mavlink_logs.append(formatted_message + " <span class='loader'></span>")
            else:
                current_text = self.right_mavlink_logs.toHtml()
                if "loader" in current_text:
                    current_text = current_text.replace(" <span class='loader'></span>", "")
                    self.right_mavlink_logs.setHtml(current_text)
                self.right_mavlink_logs.append(formatted_message)

    def add_waypoint(self, lat, lng, uav_type):
        waypoint = {"lat": lat, "lng": lng, "altitude": 30.0, "speed": 5.0, "delay": 0, "command": "Waypoint"}
        if uav_type == "Sol Panel":
            self.left_waypoints.append(waypoint)
            self.log_message(uav_type, f"NEW WAYPOINT ASSIGNED {len(self.left_waypoints)-1}/{len(self.left_waypoints)}")
            self.update_plan_table(uav_type)
            if isinstance(lat, (int, float)) and isinstance(lng, (int, float)):
                self.left_plan_map.page().runJavaScript(f"addWaypoint({lat}, {lng});")
            else:
                self.log_message(uav_type, f"🔴 Invalid waypoint coordinates: lat={lat}, lng={lng}")
        elif uav_type == "Sağ Panel":
            self.right_waypoints.append(waypoint)
            self.log_message(uav_type, f"NEW WAYPOINT ASSIGNED {len(self.right_waypoints)-1}/{len(self.right_waypoints)}")
            self.update_plan_table(uav_type)
            if isinstance(lat, (int, float)) and isinstance(lng, (int, float)):
                self.right_plan_map.page().runJavaScript(f"addWaypoint({lat}, {lng});")
            else:
                self.log_message(uav_type, f"🔴 Invalid waypoint coordinates: lat={lat}, lng={lng}")

    def show_context_menu(self, lat, lng, uav_type):
        menu = QMenu(self)
        delete_action = QAction("Delete WP", self)
        insert_action = QAction("Add WP", self)
        takeoff_action = QAction("Takeoff", self)
        rtl_action = QAction("RTL", self)
        loiter_action = QAction("Loiter", self)
        menu.addAction(delete_action)
        menu.addAction(insert_action)
        menu.addAction(takeoff_action)
        menu.addAction(rtl_action)
        menu.addAction(loiter_action)

        delete_action.triggered.connect(lambda: self.delete_last_waypoint(uav_type))
        insert_action.triggered.connect(lambda: self.add_waypoint(lat, lng, uav_type))
        takeoff_action.triggered.connect(lambda: self.add_waypoint_with_command(lat, lng, uav_type, "Takeoff"))
        rtl_action.triggered.connect(lambda: self.add_waypoint_with_command(lat, lng, uav_type, "RTL"))
        loiter_action.triggered.connect(lambda: self.add_waypoint_with_command(lat, lng, uav_type, "Loiter"))

        menu.exec_(self.mapToGlobal(self.cursor().pos()))

    def add_waypoint_with_command(self, lat, lng, uav_type, command):
        waypoint = {
            "lat": lat,
            "lng": lng,
            "altitude": 30.0,
            "speed": 5.0,
            "delay": 0,
            "command": command
        }
        if command == "Takeoff":
            gps_msg = None
            connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
            if connection:
                gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if gps_msg and gps_msg.lat != 0 and gps_msg.lon != 0:
                waypoint["lat"] = gps_msg.lat / 1e7
                waypoint["lng"] = gps_msg.lon / 1e7
                self.log_message(uav_type, f"🔵 Takeoff waypoint set to current position: lat={waypoint['lat']}, lng={waypoint['lng']}")
            else:
                self.log_message(uav_type, f"🟡 Failed to get valid GPS data for Takeoff waypoint, using default lat={lat}, lng={lng}")
            waypoint["altitude"] = 30.0
        if uav_type == "Sol Panel":
            self.left_waypoints.append(waypoint)
            self.log_message(uav_type, f"NEW WAYPOINT ASSIGNED {len(self.left_waypoints)-1}/{len(self.left_waypoints)}")
            self.update_plan_table(uav_type)
            if isinstance(waypoint["lat"], (int, float)) and isinstance(waypoint["lng"], (int, float)):
                self.left_plan_map.page().runJavaScript(f"addWaypoint({waypoint['lat']}, {waypoint['lng']});")
            else:
                self.log_message(uav_type, f"🔴 Invalid waypoint coordinates: lat={waypoint['lat']}, lng={waypoint['lng']}")
        elif uav_type == "Sağ Panel":
            self.right_waypoints.append(waypoint)
            self.log_message(uav_type, f"NEW WAYPOINT ASSIGNED {len(self.right_waypoints)-1}/{len(self.right_waypoints)}")
            self.update_plan_table(uav_type)
            if isinstance(waypoint["lat"], (int, float)) and isinstance(waypoint["lng"], (int, float)):
                self.right_plan_map.page().runJavaScript(f"addWaypoint({waypoint['lat']}, {waypoint['lng']});")
            else:
                self.log_message(uav_type, f"🔴 Invalid waypoint coordinates: lat={waypoint['lat']}, lng={waypoint['lng']}")

    def delete_last_waypoint(self, uav_type):
        waypoints = self.left_waypoints if uav_type == "Sol Panel" else self.right_waypoints
        map_widget = self.left_plan_map if uav_type == "Sol Panel" else self.right_plan_map
        if waypoints:
            waypoints.pop()
            self.update_plan_table(uav_type)
            map_widget.page().runJavaScript("clearWaypoints();")
            for wp in waypoints:
                if isinstance(wp['lat'], (int, float)) and isinstance(wp['lng'], (int, float)):
                    map_widget.page().runJavaScript(f"addWaypoint({wp['lat']}, {wp['lng']});")

    def validate_waypoints(self, waypoints, uav_type):
        for i, wp in enumerate(waypoints):
            if not isinstance(wp['lat'], (int, float)) or not isinstance(wp['lng'], (int, float)):
                self.log_message(uav_type, f"🔴 Invalid waypoint {i}: lat={wp['lat']}, lng={wp['lng']}")
                return False
            if wp['lat'] < -90 or wp['lat'] > 90 or wp['lng'] < -180 or wp['lng'] > 180:
                self.log_message(uav_type, f"🔴 Waypoint {i} out of bounds: lat={wp['lat']}, lng={wp['lng']}")
                return False
        return True

    async def write_waypoints(self, uav_type):
        connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
        waypoints = self.left_waypoints if uav_type == "Sol Panel" else self.right_waypoints
        system_ready = self.left_system_ready if uav_type == "Sol Panel" else self.right_system_ready

        if not connection:
            self.log_message(uav_type, "🔴 No connection available. Aborting waypoint upload.")
            return
        if not waypoints:
            self.log_message(uav_type, "🔴 No waypoints to upload.")
            return
        if not system_ready:
            self.log_message(uav_type, "🔴 System not ready yet, waiting for waypoint write...")
            return

        # Step 1: Validate waypoints
        if not self.validate_waypoints(waypoints, uav_type):
            self.log_message(uav_type, "🔴 Mission validation failed. Aborting upload.")
            return

        # Step 2: Pre-arm checks before mission upload
        async def pre_arm_checks(uav_type, connection):
            self.log_message(uav_type, "🔵 Performing pre-arm checks...")
            gps_msg = connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
            if not gps_msg or gps_msg.fix_type < 3:
                self.log_message(uav_type, f"🔴 Pre-arm check failed: No 3D GPS fix (fix_type={gps_msg.fix_type if gps_msg else 'unknown'})")
                return False
            sys_status = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
            if not sys_status or not (sys_status.onboard_control_sensors_present & sys_status.onboard_control_sensors_enabled & sys_status.onboard_control_sensors_health):
                self.log_message(uav_type, "🔴 Pre-arm check failed: Sensor status not suitable")
                return False
            self.log_message(uav_type, "🟢 Pre-arm checks passed")
            return True

        if not await pre_arm_checks(uav_type, connection):
            self.log_message(uav_type, "🔴 Aborting mission upload: Pre-arm checks failed.")
            return

        # Step 3: Check if first waypoint is TAKEOFF
        is_first_takeoff = waypoints[0]["command"] == "Takeoff"
        if not is_first_takeoff:
            self.log_message(uav_type, "🔴 First waypoint must be TAKEOFF for AUTO mode!")
            return
        self.log_message(uav_type, "🟢 First waypoint is TAKEOFF, will set current=1 for seq=0")

        # Step 4: Fetch home position for TAKEOFF waypoint
        self.log_message(uav_type, "🔵 Fetching home position for TAKEOFF waypoint...")
        home_msg = None
        start_time = time.time()
        timeout = 10
        while time.time() - start_time < timeout:
            home_msg = connection.recv_match(type='HOME_POSITION', blocking=True, timeout=1)
            if home_msg and home_msg.latitude != 0 and home_msg.longitude != 0:
                self.log_message(uav_type, f"🟢 Home position received: lat={home_msg.latitude / 1e7}, lng={home_msg.longitude / 1e7}")
                break
            self.log_message(uav_type, "🟡 Waiting for valid home position...")
            time.sleep(1)
        if not home_msg or home_msg.latitude == 0 or home_msg.longitude == 0:
            self.log_message(uav_type, "🔴 Aborting mission upload: Could not retrieve valid home position. Please ensure home position is set.")
            return

        # Update TAKEOFF waypoint with home position
        waypoints[0]["lat"] = home_msg.latitude / 1e7
        waypoints[0]["lng"] = home_msg.longitude / 1e7
        self.log_message(uav_type, f"🔵 TAKEOFF waypoint updated with home position: lat={waypoints[0]['lat']}, lng={waypoints[0]['lng']}")

        # Step 5: Mission upload sequence
        total_waypoints = len(waypoints)
        max_upload_retries = 3
        mission_uploaded = False

        for upload_attempt in range(max_upload_retries):
            if mission_uploaded:
                break

            self.log_message(uav_type, f"🔵 Mission upload attempt {upload_attempt + 1}/{max_upload_retries}...")

            requested_seqs = set()

            # Step 5.1: Clear existing mission (MISSION_CLEAR_ALL)
            self.log_message(uav_type, "🔵 Sending MISSION_CLEAR_ALL...")
            connection.mav.mission_clear_all_send(
                connection.target_system,
                1
            )
            start_time = time.time()
            while time.time() - start_time < 3:
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    self.log_message(uav_type, "🟢 HEARTBEAT received after MISSION_CLEAR_ALL")
                    break
                time.sleep(0.1)
            else:
                self.log_message(uav_type, "🔴 No HEARTBEAT received after MISSION_CLEAR_ALL. Aborting mission upload...")
                return

            # Step 5.2: Send mission count (MISSION_COUNT = len(waypoints))
            self.log_message(uav_type, f"🔵 Sending MISSION_COUNT: {total_waypoints} waypoints...")
            connection.mav.mission_count_send(
                connection.target_system,
                0,
                total_waypoints
            )
            start_time = time.time()
            while time.time() - start_time < 3:
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    self.log_message(uav_type, "🟢 HEARTBEAT received after MISSION_COUNT")
                    break
                time.sleep(0.1)
            else:
                self.log_message(uav_type, "🔴 No HEARTBEAT received after MISSION_COUNT. Aborting mission upload...")
                return

            # Step 5.3: Send waypoints based on MISSION_REQUEST messages using MISSION_ITEM_INT
            max_retries = 3
            timeout_per_seq = 10

            while len(requested_seqs) < total_waypoints:
                self.log_message(uav_type, f"🔵 Waiting for MISSION_REQUEST (received {len(requested_seqs)}/{total_waypoints} waypoints)...")
                ack = None
                for retry in range(max_retries):
                    start_time = time.time()
                    while time.time() - start_time < timeout_per_seq:
                        ack = connection.recv_match(type='MISSION_REQUEST', blocking=True, timeout=1)
                        if ack:
                            break
                        time.sleep(0.1)
                    if ack:
                        break
                    self.log_message(uav_type, f"🟡 MISSION_REQUEST retry {retry + 1}/{max_retries} (timeout={timeout_per_seq}s)...")
                if not ack:
                    self.log_message(uav_type, "🔴 MISSION_REQUEST not received after retries. Aborting mission upload due to missing waypoint.")
                    return

                seq = ack.seq
                self.log_message(uav_type, f"🔵 MISSION_REQUEST received for seq: {seq}")
                requested_seqs.add(seq)

                if seq >= total_waypoints:
                    self.log_message(uav_type, f"🔴 Invalid sequence number received: {seq}. Aborting mission upload.")
                    return

                wp = waypoints[seq]
                command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF if wp["command"] == "Takeoff" else mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                frame_type = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

                current = 1 if seq == 0 else 0
                self.log_message(uav_type, f"🔵 Setting current={current} for seq={seq}")

                autocontinue_flag = 1
                if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    param1 = 15.0
                    param2 = 0.0
                    param3 = 0.0
                    param4 = 0.0
                    param7 = wp["altitude"]
                else:
                    param1 = wp["delay"]
                    param2 = 2.0
                    param3 = 0.0
                    param4 = 0.0
                    param7 = wp["altitude"]

                if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    self.takeoff_params = {"frame": frame_type, "param1": param1, "param7": param7}
                    self.log_message(uav_type, f"🟢 TAKEOFF params set: frame={frame_type}, command={command}, current={current}, autocontinue={autocontinue_flag}, param1={param1}, param7={param7}")

                self.log_message(uav_type, f"🔵 Preparing waypoint {seq}: command={command}, frame={frame_type}, "
                                        f"lat={wp['lat']}, lng={wp['lng']}, alt={param7}, current={current}, autocontinue={autocontinue_flag}")

                try:
                    connection.mav.mission_item_int_send(
                        connection.target_system,
                        0,
                        seq,
                        frame_type,
                        command,
                        current,
                        autocontinue_flag,
                        param1, param2, param3, param4,
                        int(wp["lat"] * 1e7), int(wp["lng"] * 1e7), param7
                    )
                    self.log_message(uav_type, f"🟢 Waypoint {seq} sent via MISSION_ITEM_INT")
                except Exception as e:
                    self.log_message(uav_type, f"🔴 Failed to send waypoint {seq} via MISSION_ITEM_INT: {str(e)}. Aborting mission upload.")
                    return

                time.sleep(0.2)

            # Step 6: Verify all MISSION_REQUEST messages were received
            self.log_message(uav_type, "🔵 Verifying all MISSION_REQUEST messages were received...")
            if len(requested_seqs) != total_waypoints:
                self.log_message(uav_type, f"🔴 Not all MISSION_REQUEST messages received. Expected {total_waypoints}, received {len(requested_seqs)}. Aborting mission.")
                return
            self.log_message(uav_type, "🟢 All MISSION_REQUEST messages received successfully.")

            # Step 7: Verify at least 2 waypoints were sent
            self.log_message(uav_type, "🔵 Verifying at least 2 waypoints were sent...")
            if len(requested_seqs) < 2:
                self.log_message(uav_type, "🔴 Fewer than 2 waypoints were sent. Aborting mission.")
                return
            if 0 not in requested_seqs or 1 not in requested_seqs:
                self.log_message(uav_type, "🔴 Waypoints 0 and 1 were not both sent. Aborting mission.")
                return
            self.log_message(uav_type, "🟢 At least 2 waypoints (seq 0 and seq 1) were sent successfully.")

            # Step 8: Wait for MISSION_ACK with retries
            self.log_message(uav_type, "🔵 Waiting for MISSION_ACK (timeout=10s)...")
            mission_ack = connection.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if not mission_ack:
                self.log_message(uav_type, f"🔴 MISSION_ACK not received after attempt {upload_attempt + 1}/{max_upload_retries}.")
                if upload_attempt < max_upload_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying mission upload...")
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission upload: MISSION_ACK not received after max retries. Please retry the mission upload.")
                    return
            else:
                self.log_message(uav_type, f"🟢 MISSION_ACK received: type={mission_ack.type}")
                if mission_ack.type != 0:
                    self.log_message(uav_type, f"🔴 MISSION_ACK error: type={mission_ack.type} (mission rejected). Retrying mission upload...")
                    if upload_attempt < max_upload_retries - 1:
                        continue
                    else:
                        self.log_message(uav_type, "🔴 Aborting mission upload after max retries due to MISSION_ACK error. Please retry the mission upload.")
                        return

            # Step 9: Verify MISSION_CURRENT.seq == 0 or 1
            self.log_message(uav_type, "🔵 Verifying MISSION_CURRENT.seq == 0 or 1 (requires 3 consecutive confirmations)...")
            seq_valid_count = 0
            max_seq_checks = 10
            for attempt in range(max_seq_checks):
                mission_current = connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
                if mission_current and mission_current.seq in [0, 1]:
                    seq_valid_count += 1
                    self.log_message(uav_type, f"🟢 MISSION_CURRENT.seq == {mission_current.seq} (confirmation {seq_valid_count}/3)")
                    if seq_valid_count >= 3:
                        self.log_message(uav_type, "🟢 MISSION_CURRENT verified (seq == 0 or 1 confirmed 3 times consecutively)")
                        mission_uploaded = True
                        break
                else:
                    seq_valid_count = 0
                    self.log_message(uav_type, f"🟡 MISSION_CURRENT.seq={mission_current.seq if mission_current else 'unknown'} (expected 0 or 1), resetting count...")
                time.sleep(1)

            if not mission_uploaded:
                self.log_message(uav_type, f"🔴 MISSION_CURRENT verification failed: seq not 0 or 1 after {max_seq_checks} attempts.")
                if upload_attempt < max_upload_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying mission upload...")
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission upload after max retries due to MISSION_CURRENT verification failure. Please retry the mission upload.")
                    return

        # Step 10: Set MISSION_SET_CURRENT to waypoint 0 (TAKEOFF) and verify
        self.log_message(uav_type, "🔵 Setting MISSION_SET_CURRENT to waypoint 0 (TAKEOFF) after successful waypoint upload...")
        max_set_current_retries = 3
        for attempt in range(max_set_current_retries):
            connection.mav.mission_set_current_send(
                connection.target_system,
                0,
                0
            )
            self.log_message(uav_type, f"🔵 Waiting for MISSION_SET_CURRENT COMMAND_ACK (attempt {attempt + 1}/{max_set_current_retries})...")
            ack = await self.wait_for_ack_async(connection, mavutil.mavlink.MAV_CMD_MISSION_SET_CURRENT, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log_message(uav_type, "🟢 MISSION_SET_CURRENT COMMAND_ACK accepted")
                break
            else:
                self.log_message(uav_type, "🔴 MISSION_SET_CURRENT COMMAND_ACK not received or rejected.")
                if attempt < max_set_current_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying MISSION_SET_CURRENT...")
                    time.sleep(0.5)
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission: MISSION_SET_CURRENT failed after max retries. Please retry the mission upload.")
                    return

        self.log_message(uav_type, "🔵 Verifying MISSION_CURRENT.seq == 0 after MISSION_SET_CURRENT (requires 3 consecutive confirmations)...")
        seq_zero_count = 0
        max_seq_checks = 10
        for attempt in range(max_seq_checks):
            mission_current = connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
            if mission_current and mission_current.seq == 0:
                seq_zero_count += 1
                self.log_message(uav_type, f"🟢 MISSION_CURRENT.seq == 0 (confirmation {seq_zero_count}/3)")
                if seq_zero_count >= 3:
                    self.log_message(uav_type, "🟢 MISSION_CURRENT verified (seq == 0 confirmed 3 times consecutively)")
                    break
            else:
                seq_zero_count = 0
                self.log_message(uav_type, f"🟡 MISSION_CURRENT.seq={mission_current.seq if mission_current else 'unknown'} (expected 0), resetting count...")
            time.sleep(1)
        if seq_zero_count < 3:
            self.log_message(uav_type, "🔴 Aborting mission: MISSION_CURRENT verification failed after MISSION_SET_CURRENT (seq != 0). Please retry the mission upload.")
            return

        # Step 11: ARM the vehicle
        self.log_message(uav_type, "🔵 Ensuring vehicle is armed before AUTO mode...")
        if not await pre_arm_checks(uav_type, connection):
            self.log_message(uav_type, "🔴 Aborting mission: Pre-arm checks failed before ARM.")
            return

        arm_success = False
        max_arm_retries = 3
        for attempt in range(max_arm_retries):
            start_time = time.time()
            while time.time() - start_time < 5:
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and msg.get_srcSystem() == connection.target_system:
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed:
                        self.log_message(uav_type, "🟢 Vehicle already ARMED")
                        arm_success = True
                        break
                time.sleep(0.1)
            if arm_success:
                break

            self.log_message(uav_type, f"🔵 Arming the vehicle, attempt {attempt + 1}/{max_arm_retries}...")
            connection.mav.command_long_send(
                connection.target_system,
                1,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            self.log_message(uav_type, f"🔵 Waiting for ARM COMMAND_ACK (attempt {attempt + 1}/{max_arm_retries})...")
            ack = await self.wait_for_ack_async(connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log_message(uav_type, "🟢 ARM COMMAND_ACK accepted")
            else:
                self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_arm_retries} failed: COMMAND_ACK not received or rejected.")
                if attempt < max_arm_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying ARM command...")
                    time.sleep(0.5)
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission: ARM failed after max retries. Please retry the mission upload.")
                    return

            time.sleep(0.5)

            start_time = time.time()
            while time.time() - start_time < 5:
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and msg.get_srcSystem() == connection.target_system:
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed:
                        self.log_message(uav_type, "🟢 Vehicle ARMED verified via HEARTBEAT")
                        arm_success = True
                        break
                time.sleep(0.1)
            if arm_success:
                break
        if not arm_success:
            self.log_message(uav_type, "🔴 ARM failed after max retries. Aborting mission as AUTO mode cannot be initiated without ARM.")
            return

        # Step 12: Switch to AUTO mode
        self.log_message(uav_type, "🔵 Switching to AUTO mode...")
        if not await pre_arm_checks(uav_type, connection):
            self.log_message(uav_type, "🔴 Aborting mission: Pre-arm checks failed before AUTO mode.")
            return

        mode_map = self.rotary_id_to_mode if uav_type == "Sol Panel" else self.fixed_id_to_mode
        mode = "AUTO"
        mode_id = {v: k for k, v in mode_map.items()}.get(mode)
        if mode_id is None:
            self.log_message(uav_type, "🔴 Invalid mode: AUTO. Aborting mission.")
            return

        max_mode_retries = 3
        auto_mode_success = False
        for attempt in range(max_mode_retries):
            self.log_message(uav_type, f"🔵 Sending AUTO mode command, attempt {attempt + 1}/{max_mode_retries}...")
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            # Wait for COMMAND_ACK
            self.log_message(uav_type, f"🔵 Waiting for AUTO mode COMMAND_ACK (attempt {attempt + 1}/{max_mode_retries})...")
            ack = await self.wait_for_ack_async(connection, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log_message(uav_type, "🟢 AUTO mode COMMAND_ACK accepted")
            else:
                self.log_message(uav_type, f"🔴 AUTO mode attempt {attempt + 1}/{max_mode_retries} failed: COMMAND_ACK not received or rejected.")
                if attempt < max_mode_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying AUTO mode command...")
                    time.sleep(0.5)  # Small delay before retry
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission: Failed to switch to AUTO mode after max retries. Please retry the mission upload.")
                    return

            # Add delay after AUTO mode command
            time.sleep(0.5)

            # Verify AUTO mode via HEARTBEAT
            auto_mode_verified = await self.auto_mode_verification(uav_type, connection)
            if auto_mode_verified:
                auto_mode_success = True
                break
            else:
                self.log_message(uav_type, f"🔴 AUTO mode attempt {attempt + 1}/{max_mode_retries} failed: HEARTBEAT verification failed.")
                if attempt < max_mode_retries - 1:
                    self.log_message(uav_type, "🔵 Retrying AUTO mode verification...")
                    continue
                else:
                    self.log_message(uav_type, "🔴 Aborting mission: Failed to verify AUTO mode after max retries. Please retry the mission upload.")
                    return

        self.log_message(uav_type, "🟢 Mission upload and AUTO mode sequence completed successfully. TAKEOFF will be executed in AUTO mode.")

    def update_plan_table(self, uav_type):
        waypoints = self.left_waypoints if uav_type == "Sol Panel" else self.right_waypoints
        table = self.left_plan_table if uav_type == "Sol Panel" else self.right_plan_table
        table.setRowCount(len(waypoints))
        for i, wp in enumerate(waypoints):
            table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            command_combo = QComboBox()
            command_combo.addItems(["Waypoint", "Takeoff", "Land", "RTL", "Loiter"])
            command_combo.setCurrentText(wp["command"])
            command_combo.currentTextChanged.connect(lambda text, idx=i: self.update_waypoint_command(uav_type, idx, text))
            table.setCellWidget(i, 1, command_combo)
            table.setItem(i, 2, QTableWidgetItem(f"{wp['lat']:.6f}"))
            table.setItem(i, 3, QTableWidgetItem(f"{wp['lng']:.6f}"))
            table.setItem(i, 4, QTableWidgetItem(str(wp["altitude"])))
            table.setItem(i, 5, QTableWidgetItem(str(wp["speed"])))
            table.setItem(i, 6, QTableWidgetItem(str(wp["delay"])))
            delete_btn = QPushButton("Delete")
            delete_btn.clicked.connect(lambda _, idx=i: self.delete_waypoint(uav_type, idx))
            table.setCellWidget(i, 7, delete_btn)

    def update_waypoint_command(self, uav_type, index, command):
        waypoints = self.left_waypoints if uav_type == "Sol Panel" else self.right_waypoints
        if 0 <= index < len(waypoints):
            waypoints[index]["command"] = command
            if command == "Takeoff":
                connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
                gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3) if connection else None
                if gps_msg and gps_msg.lat != 0 and gps_msg.lon != 0:
                    waypoints[index]["lat"] = gps_msg.lat / 1e7
                    waypoints[index]["lng"] = gps_msg.lon / 1e7
                else:
                    waypoints[index]["lat"] = 0
                    waypoints[index]["lng"] = 0
                waypoints[index]["altitude"] = 30.0
            self.update_plan_table(uav_type)

    def delete_waypoint(self, uav_type, index):
        waypoints = self.left_waypoints if uav_type == "Sol Panel" else self.right_waypoints
        map_widget = self.left_plan_map if uav_type == "Sol Panel" else self.right_plan_map
        if 0 <= index < len(waypoints):
            waypoints.pop(index)
            self.update_plan_table(uav_type)
            map_widget.page().runJavaScript("clearWaypoints();")
            for wp in waypoints:
                if isinstance(wp['lat'], (int, float)) and isinstance(wp['lng'], (int, float)):
                    map_widget.page().runJavaScript(f"addWaypoint({wp['lat']}, {wp['lng']});")

    def create_telemetry_control_panel(self, layout, uav_type):
        control_frame = QWidget()
        control_layout = QHBoxLayout(control_frame)

        flight_mode_label = QLabel("Flight Mode: Unknown")
        flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px;")
        control_layout.addWidget(flight_mode_label)

        flight_mode_combo = QComboBox()
        if uav_type == "Sol Panel":
            flight_mode_combo.addItems(["STABILIZE", "AUTO", "GUIDED", "RTL", "LOITER", "FBWA", "MANUAL"])
        else:
            flight_mode_combo.addItems([
                "MANUAL", "STABILIZE", "ACRO", "FBWA", "FBWB", "CRUISE", "AUTOTUNE", "AUTO", "RTL", "LOITER", 
                "TAKEOFF", "AVOID_ADSB", "GUIDED", "INITIALISING", "QSTABILIZE", "QHOVER", "QLOITER", 
                "QLAND", "QRTL", "QAUTOTUNE", "QACRO", "THERMAL", "LOITER to QLAND"
            ])
        control_layout.addWidget(flight_mode_combo)

        flight_mode_confirm_btn = QPushButton("✔")
        flight_mode_confirm_btn.clicked.connect(lambda: self.confirm_flight_mode(uav_type, text=flight_mode_combo.currentText()))
        control_layout.addWidget(flight_mode_confirm_btn)

        arm_status_label = QLabel("Motor Status: DISARM")
        arm_status_label.setStyleSheet("background-color: red; color: white; padding: 5px;")
        control_layout.addWidget(arm_status_label)

        arm_combo = QComboBox()
        arm_combo.addItems(["ARM", "DISARM"])
        control_layout.addWidget(arm_combo)

        arm_confirm_btn = QPushButton("✔")
        arm_confirm_btn.clicked.connect(lambda: self.confirm_arm_status(uav_type, arm_combo.currentText()))
        control_layout.addWidget(arm_confirm_btn)

        if uav_type == "Sol Panel":
            self.left_flight_mode_label = flight_mode_label
            self.left_arm_status_label = arm_status_label
            self.left_flight_mode_combo = flight_mode_combo
            self.left_arm_combo = arm_combo
        elif uav_type == "Sağ Panel":
            self.right_flight_mode_label = flight_mode_label
            self.right_arm_status_label = arm_status_label
            self.right_flight_mode_combo = flight_mode_combo
            self.right_arm_combo = arm_combo

        layout.addWidget(control_frame)

        telemetry_frame = QWidget()
        telemetry_layout = QGridLayout(telemetry_frame)

        if not hasattr(self, 'telemetry_labels'):
            self.telemetry_labels = {}
        telemetry_data = [
            ("Altitude (m)", QColor("purple")),
            ("Ground Speed (kph)", QColor("orange")),
            ("WP Distance (m)", QColor("red")),
            ("Vertical Speed (kph)", QColor("yellow")),
            ("Yaw (degree)", QColor("green")),
            ("MAV Distance", QColor("blue"))
        ]

        row = 0
        for label_text, color in telemetry_data:
            label_widget = QLabel(f"{label_text}:")
            value_widget = QLabel("0.000")
            value_widget.setStyleSheet(f"color: {color.name()};")
            self.telemetry_labels[(uav_type, label_text)] = value_widget
            telemetry_layout.addWidget(label_widget, row, 0)
            telemetry_layout.addWidget(value_widget, row, 1)
            row += 1

        layout.addWidget(telemetry_frame)

    async def send_mode_change(self, uav_type, connection, mode, mode_id):
        try:
            telemetry = self.left_telemetry if uav_type == "Sol Panel" else self.right_telemetry
            if not telemetry["Armed"]:
                self.log_message(uav_type, f"🔴 Mode change to {mode} rejected: System is DISARMED!")
                return False

            self.log_message(uav_type, f"🔵 Attempting mode change: {mode}")
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            self.log_message(uav_type, f"🔵 Command sent for {mode} mode, waiting for HEARTBEAT verification...")
            start_time = time.time()
            mode_count = 0
            mode_map = self.rotary_id_to_mode if uav_type == "Sol Panel" else self.fixed_id_to_mode
            while time.time() - start_time < 5:
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and msg.get_srcSystem() == connection.target_system:
                    current_mode = mode_map.get(msg.custom_mode, "Unknown")
                    if current_mode == mode:
                        mode_count += 1
                        if mode_count >= 3:
                            self.log_message(uav_type, f"🟢 {mode} mode verified via HEARTBEAT (3 consecutive confirmations)")
                            return True
                    else:
                        mode_count = 0
                time.sleep(0.1)
            self.log_message(uav_type, f"🔴 {mode} mode not verified via HEARTBEAT.")
            return False
        except Exception as e:
            self.log_message(uav_type, f"🔴 Mode change error: {str(e)}")
            return False

    def confirm_flight_mode(self, uav_type, text):
        connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
        telemetry = self.left_telemetry if uav_type == "Sol Panel" else self.right_telemetry
        mode_map = self.rotary_id_to_mode if uav_type == "Sol Panel" else self.fixed_id_to_mode
        mode = text

        if not connection:
            self.log_message(uav_type, "🔴 Mode change rejected: No connection!")
            return

        if telemetry["Flight Mode"] == mode:
            self.log_message(uav_type, f"🔵 Mode already {mode}, no change needed.")
            return

        mode_id = {v: k for k, v in mode_map.items()}.get(mode)
        if mode_id is None:
            self.log_message(uav_type, f"🔴 Invalid mode: {mode}")
            return

        system_ready = self.left_system_ready if uav_type == "Sol Panel" else self.right_system_ready
        system_status = self.left_system_status[0] if uav_type == "Sol Panel" else self.right_system_status[0]
        if not system_ready or system_status < mavutil.mavlink.MAV_STATE_STANDBY:
            self.log_message(uav_type, "🔴 System not ready yet, waiting for mode change...")
            return

        try:
            mode_mapping = mavutil.mode_mapping_bynumber(connection.mav_type)
            if mode not in mode_mapping.values():
                self.log_message(uav_type, f"🔴 Mode change rejected: {mode} not supported for this vehicle!")
                return

            success = self.run_async_task(self.send_mode_change(uav_type, connection, mode, mode_id))
            if not success:
                self.log_message(uav_type, f"🔴 Mode change to {mode} failed.")
        except Exception as e:
            self.log_message(uav_type, f"🔴 Mode change error: {str(e)}")

    def confirm_arm_status(self, uav_type, status):
        connection = self.left_connection if uav_type == "Sol Panel" else self.right_connection
        telemetry = self.left_telemetry if uav_type == "Sol Panel" else self.right_telemetry
        arm_combo = self.left_arm_combo if uav_type == "Sol Panel" else self.right_arm_combo

        if not connection:
            self.log_message(uav_type, "🔴 Motor status change rejected: No connection!")
            return

        new_armed = (status == "ARM")
        if telemetry["Armed"] == new_armed:
            self.log_message(uav_type, f"🔵 Motor status already {status}, no change needed.")
            return

        system_ready = self.left_system_ready if uav_type == "Sol Panel" else self.right_system_ready
        system_status = self.left_system_status[0] if uav_type == "Sol Panel" else self.right_system_status[0]
        if not system_ready or system_status < mavutil.mavlink.MAV_STATE_STANDBY:
            self.log_message(uav_type, "🔴 System not ready yet, waiting for motor status change...")
            return

        max_retries = 3
        for attempt in range(max_retries):
            try:
                if new_armed:
                    gps_msg = connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
                    if not gps_msg or gps_msg.fix_type < 3:
                        self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: No 3D GPS fix (fix_type={gps_msg.fix_type if gps_msg else 'unknown'})!")
                        continue

                    sys_status = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
                    if sys_status:
                        if not (sys_status.onboard_control_sensors_present & sys_status.onboard_control_sensors_enabled & sys_status.onboard_control_sensors_health):
                            self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: Sensor status not suitable!")
                            continue
                    else:
                        self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: SYS_STATUS not received!")
                        continue

                connection.mav.command_long_send(
                    connection.target_system,
                    1,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1 if new_armed else 0, 0, 0, 0, 0, 0, 0
                )
                self.log_message(uav_type, f"🔵 Motor status command sent: {status}")
                ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.log_message(uav_type, f"🟢 Command accepted (ACK): {status}")
                else:
                    self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: result={ack.result if ack else 'no ACK received'}")
                    continue

                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                    if msg and msg.get_srcSystem() == connection.target_system:
                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        if armed == new_armed:
                            self.log_message(uav_type, f"🟢 Motor status verified via HEARTBEAT: {'ARM' if new_armed else 'DISARM'}")
                            telemetry["Armed"] = new_armed
                            arm_combo.setCurrentText(status)
                            self.update_telemetry(uav_type, telemetry)
                            return
                    time.sleep(0.1)
                self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: Motor status not verified via HEARTBEAT.")
            except Exception as e:
                self.log_message(uav_type, f"🔴 ARM attempt {attempt + 1}/{max_retries} failed: {str(e)}")
        self.log_message(uav_type, f"🔴 Motor status change to {status} failed after {max_retries} attempts.")

    def update_telemetry(self, uav_type, telemetry):
        for (type_, label), widget in list(self.telemetry_labels.items()):
            if type_ == uav_type:
                if "Altitude" in label:
                    widget.setText(f"{telemetry['Altitude']:.3f}")
                elif "Ground Speed" in label:
                    widget.setText(f"{telemetry['Ground Speed']:.3f}")
                elif "WP Distance" in label:
                    widget.setText(f"{telemetry['Dist to WP']:.3f}")
                elif "Vertical Speed" in label:
                    widget.setText(f"{telemetry['Vertical Speed']:.3f}")
                elif "Yaw" in label:
                    widget.setText(f"{telemetry['Yaw']:.3f}")
                elif "MAV Distance" in label:
                    widget.setText(f"{telemetry['Dist to MAV']:.3f}")

        flight_mode_label = self.left_flight_mode_label if uav_type == "Sol Panel" else self.right_flight_mode_label
        arm_status_label = self.left_arm_status_label if uav_type == "Sol Panel" else self.right_arm_status_label

        flight_mode_label.setText(f"Flight Mode: {telemetry['Flight Mode']}")
        arm_status_label.setText(f"Motor Status: {'ARM' if telemetry['Armed'] else 'DISARM'}")
        arm_status_label.setStyleSheet(
            "background-color: green; color: white; padding: 5px;" if telemetry["Armed"]
            else "background-color: red; color: white; padding: 5px;"
        )

        if uav_type == "Sol Panel":
            if telemetry["Flight Mode"] == "STABILIZE":
                flight_mode_label.setStyleSheet("background-color: yellow; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "AUTO":
                flight_mode_label.setStyleSheet("background-color: green; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "GUIDED":
                flight_mode_label.setStyleSheet("background-color: blue; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "RTL":
                flight_mode_label.setStyleSheet("background-color: purple; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "LOITER":
                flight_mode_label.setStyleSheet("background-color: orange; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "FBWA":
                flight_mode_label.setStyleSheet("background-color: lightblue; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "MANUAL":
                flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px;")
            else:
                flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px;")
            if not self.left_arm_status_updated:
                self.log_message(uav_type, f"🟢 Motor status: {'ARM' if telemetry['Armed'] else 'DISARM'}")
                self.left_arm_status_updated = True
        else:
            if telemetry["Flight Mode"] == "MANUAL":
                flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "STABILIZE":
                flight_mode_label.setStyleSheet("background-color: yellow; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "AUTO":
                flight_mode_label.setStyleSheet("background-color: green; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "GUIDED":
                flight_mode_label.setStyleSheet("background-color: blue; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "RTL":
                flight_mode_label.setStyleSheet("background-color: purple; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "LOITER":
                flight_mode_label.setStyleSheet("background-color: orange; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "FBWA":
                flight_mode_label.setStyleSheet("background-color: lightblue; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "ACRO":
                flight_mode_label.setStyleSheet("background-color: magenta; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "FBWB":
                flight_mode_label.setStyleSheet("background-color: teal; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "CRUISE":
                flight_mode_label.setStyleSheet("background-color: olive; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "AUTOTUNE":
                flight_mode_label.setStyleSheet("background-color: brown; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "TAKEOFF":
                flight_mode_label.setStyleSheet("background-color: indigo; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "AVOID_ADSB":
                flight_mode_label.setStyleSheet("background-color: violet; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "INITIALISING":
                flight_mode_label.setStyleSheet("background-color: coral; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "QSTABILIZE":
                flight_mode_label.setStyleSheet("background-color: gold; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "QHOVER":
                flight_mode_label.setStyleSheet("background-color: lime; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "QLOITER":
                flight_mode_label.setStyleSheet("background-color: maroon; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "QLAND":
                flight_mode_label.setStyleSheet("background-color: navy; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "QRTL":
                flight_mode_label.setStyleSheet("background-color: orchid; color: white; padding: 5px;")
            elif telemetry["Flight Mode"] == "QAUTOTUNE":
                flight_mode_label.setStyleSheet("background-color: pink; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "QACRO":
                flight_mode_label.setStyleSheet("background-color: silver; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "THERMAL":
                flight_mode_label.setStyleSheet("background-color: turquoise; color: black; padding: 5px;")
            elif telemetry["Flight Mode"] == "LOITER to QLAND":
                flight_mode_label.setStyleSheet("background-color: salmon; color: white; padding: 5px;")
            else:
                flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px;")
            if not self.right_arm_status_updated:
                self.log_message(uav_type, f"🟢 Motor status: {'ARM' if telemetry['Armed'] else 'DISARM'}")
                self.right_arm_status_updated = True

    def update_data(self):
        if not hasattr(self, 'last_left_system_status'):
            self.last_left_system_status = -1
            self.last_right_system_status = -1
            self.last_left_gps_connected = False
            self.last_right_gps_connected = False
            self.last_left_gps_timeout_logged = False
            self.last_right_gps_timeout_logged = False

        if self.left_connection:
            try:
                msg_received = False
                while True:
                    msg = self.left_connection.recv_match(blocking=False)
                    if not msg:
                        break
                    msg_received = True
                    msg_type = msg.get_type()
                    if msg_type == 'GLOBAL_POSITION_INT':
                        self.left_telemetry["Altitude"] = msg.relative_alt / 1000.0
                        self.left_telemetry["Yaw"] = msg.hdg / 100.0
                        self.left_telemetry["Latitude"] = msg.lat / 1e7
                        self.left_telemetry["Longitude"] = msg.lon / 1e7
                        if (self.left_data_map and 
                            isinstance(self.left_telemetry["Latitude"], (int, float)) and 
                            isinstance(self.left_telemetry["Longitude"], (int, float)) and 
                            self.left_telemetry["Latitude"] != 0 and 
                            self.left_telemetry["Longitude"] != 0 and 
                            isinstance(self.left_telemetry["Yaw"], (int, float)) and 
                            isinstance(self.left_telemetry["GPS Direction"], (int, float))):
                            self.left_data_map.page().runJavaScript(
                                f"updateMarkers({self.left_telemetry['Latitude']}, {self.left_telemetry['Longitude']}, "
                                f"{self.left_telemetry['Latitude']}, {self.left_telemetry['Longitude']}, "
                                f"{self.left_telemetry['Yaw']}, {self.left_telemetry['GPS Direction']});"
                            )
                        if (self.left_plan_map and 
                            isinstance(self.left_telemetry["Latitude"], (int, float)) and 
                            isinstance(self.left_telemetry["Longitude"], (int, float)) and 
                            self.left_telemetry["Latitude"] != 0 and 
                            self.left_telemetry["Longitude"] != 0 and 
                            isinstance(self.left_telemetry["Yaw"], (int, float)) and 
                            isinstance(self.left_telemetry["GPS Direction"], (int, float))):
                            self.left_plan_map.page().runJavaScript(
                                f"updateMarkers({self.left_telemetry['Latitude']}, {self.left_telemetry['Longitude']}, "
                                f"{self.left_telemetry['Latitude']}, {self.left_telemetry['Longitude']}, "
                                f"{self.left_telemetry['Yaw']}, {self.left_telemetry['GPS Direction']});"
                            )
                        if not self.left_gps_connected and self.last_left_gps_connected != self.left_gps_connected:
                            self.log_message("Sol Panel", "🟢 GPS data received.")
                            self.last_left_gps_connected = True
                            self.left_gps_connected = True
                            self.left_gps_waiting_logged = False
                            self.left_gps_timeout_counter = 0
                    elif msg_type == 'VFR_HUD':
                        self.left_telemetry["Ground Speed"] = msg.groundspeed * 3.6
                        self.left_telemetry["Vertical Speed"] = msg.climb * 3.6
                        if self.left_telemetry["Ground Speed"] > 0:
                            self.left_telemetry["GPS Direction"] = self.left_telemetry["Yaw"]
                if not msg_received and self.left_system_status[0] == 0 and self.left_system_status[0] != self.last_left_system_status:
                    self.log_message("Sol Panel", "🔴 System not ready, waiting for stability...")
                    self.last_left_system_status = self.left_system_status[0]
                    time.sleep(1)
                    return
                elif msg_received and self.left_system_status[0] >= mavutil.mavlink.MAV_STATE_STANDBY and not self.left_system_ready:
                    self.left_system_ready = True
                    self.log_message("Sol Panel", "🟢 System ready: Commands can be sent!")
                    self.last_left_system_status = self.left_system_status[0]
                elif msg_received and self.left_system_status[0] < mavutil.mavlink.MAV_STATE_STANDBY and self.left_system_ready and self.left_system_status[0] != self.last_left_system_status:
                    self.left_system_ready = False
                    self.log_message("Sol Panel", f"🔴 System status not suitable (system_status={self.left_system_status[0]}), waiting for stability...")
                    self.last_left_system_status = self.left_system_status[0]
                    return
            except Exception as e:
                self.log_message("Sol Panel", f"🔴 Data update error: {e}")
            if not self.left_gps_connected:
                if not self.left_gps_waiting_logged and not self.last_left_gps_timeout_logged:
                    self.log_message("Sol Panel", "🟡 Waiting for GPS data...")
                    self.left_gps_waiting_logged = True
                self.left_gps_timeout_counter += 1
                if self.left_gps_timeout_counter >= self.GPS_TIMEOUT_LIMIT and not self.last_left_gps_timeout_logged:
                    self.log_message("Sol Panel", "🔴 GPS data not received after 30 seconds!")
                    self.last_left_gps_timeout_logged = True
                    self.left_gps_waiting_logged = False
                    self.left_gps_timeout_counter = 0

        if self.right_connection:
            try:
                msg_received = False
                while True:
                    msg = self.right_connection.recv_match(blocking=False)
                    if not msg:
                        break
                    msg_received = True
                    msg_type = msg.get_type()
                    if msg_type == 'GLOBAL_POSITION_INT':
                        self.right_telemetry["Altitude"] = msg.relative_alt / 1000.0
                        self.right_telemetry["Yaw"] = msg.hdg / 100.0
                        self.right_telemetry["Latitude"] = msg.lat / 1e7
                        self.right_telemetry["Longitude"] = msg.lon / 1e7
                        if (self.right_data_map and 
                            isinstance(self.right_telemetry["Latitude"], (int, float)) and 
                            isinstance(self.right_telemetry["Longitude"], (int, float)) and 
                            self.right_telemetry["Latitude"] != 0 and 
                            self.right_telemetry["Longitude"] != 0 and 
                            isinstance(self.right_telemetry["Yaw"], (int, float)) and 
                            isinstance(self.right_telemetry["GPS Direction"], (int, float))):
                            self.right_data_map.page().runJavaScript(
                                f"updateMarkers({self.right_telemetry['Latitude']}, {self.right_telemetry['Longitude']}, "
                                f"{self.right_telemetry['Latitude']}, {self.right_telemetry['Longitude']}, "
                                f"{self.right_telemetry['Yaw']}, {self.right_telemetry['GPS Direction']});"
                            )
                        if (self.right_plan_map and 
                            isinstance(self.right_telemetry["Latitude"], (int, float)) and 
                            isinstance(self.right_telemetry["Longitude"], (int, float)) and 
                            self.right_telemetry["Latitude"] != 0 and 
                            self.right_telemetry["Longitude"] != 0 and 
                            isinstance(self.right_telemetry["Yaw"], (int, float)) and 
                            isinstance(self.right_telemetry["GPS Direction"], (int, float))):
                            self.right_plan_map.page().runJavaScript(
                                f"updateMarkers({self.right_telemetry['Latitude']}, {self.right_telemetry['Longitude']}, "
                                f"{self.right_telemetry['Latitude']}, {self.right_telemetry['Longitude']}, "
                                f"{self.right_telemetry['Yaw']}, {self.right_telemetry['GPS Direction']});"
                            )
                        if not self.right_gps_connected and self.last_right_gps_connected != self.right_gps_connected:
                            self.log_message("Sağ Panel", "🟢 GPS data received.")
                            self.last_right_gps_connected = True
                            self.right_gps_connected = True
                            self.right_gps_waiting_logged = False
                            self.right_gps_timeout_counter = 0
                    elif msg_type == 'VFR_HUD':
                        self.right_telemetry["Ground Speed"] = msg.groundspeed * 3.6
                        self.right_telemetry["Vertical Speed"] = msg.climb * 3.6
                        if self.right_telemetry["Ground Speed"] > 0:
                            self.right_telemetry["GPS Direction"] = self.right_telemetry["Yaw"]
                if not msg_received and self.right_system_status[0] == 0 and self.right_system_status[0] != self.last_right_system_status:
                    self.log_message("Sağ Panel", "🔴 System not ready, waiting for stability...")
                    self.last_right_system_status = self.right_system_status[0]
                    time.sleep(1)
                    return
                elif msg_received and self.right_system_status[0] >= mavutil.mavlink.MAV_STATE_STANDBY and not self.right_system_ready:
                    self.right_system_ready = True
                    self.log_message("Sağ Panel", "🟢 System ready: Commands can be sent!")
                    self.last_right_system_status = self.right_system_status[0]
                elif msg_received and self.right_system_status[0] < mavutil.mavlink.MAV_STATE_STANDBY and self.right_system_ready and self.right_system_status[0] != self.last_right_system_status:
                    self.right_system_ready = False
                    self.log_message("Sağ Panel", f"🔴 System status not suitable (system_status={self.right_system_status[0]}), waiting for stability...")
                    self.last_right_system_status = self.right_system_status[0]
                    return
            except Exception as e:
                self.log_message("Sağ Panel", f"🔴 Data update error: {e}")
            if not self.right_gps_connected:
                if not self.right_gps_waiting_logged and not self.last_right_gps_timeout_logged:
                    self.log_message("Sağ Panel", "🟡 Waiting for GPS data...")
                    self.right_gps_waiting_logged = True
                self.right_gps_timeout_counter += 1
                if self.right_gps_timeout_counter >= self.GPS_TIMEOUT_LIMIT and not self.last_right_gps_timeout_logged:
                    self.log_message("Sağ Panel", "🔴 GPS data not received after 30 seconds!")
                    self.last_right_gps_timeout_logged = True
                    self.right_gps_waiting_logged = False
                    self.right_gps_timeout_counter = 0

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GCS()
    window.show()
    sys.exit(app.exec_())