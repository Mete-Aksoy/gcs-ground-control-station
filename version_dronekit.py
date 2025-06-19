# =====================================================
# Ground Control Station (GCS)
# Author: Mete Aksoy
# Year: 2025
# All rights reserved.
# Unauthorized use or redistribution is prohibited.
# =====================================================

import sys
import time
import logging
import traceback
import socket
import struct
import cv2
import numpy as np
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QComboBox, QPushButton, QTextEdit, QGridLayout,
                             QTabWidget, QTableWidget, QTableWidgetItem, QCheckBox, QAbstractScrollArea, QFileDialog,
                             QDialog, QDoubleSpinBox, QFormLayout)
from PyQt5.QtCore import Qt, QTimer, QUrl, pyqtSlot, QObject, QThread, pyqtSignal
from PyQt5.QtGui import QTextCursor, QColor, QImage, QPixmap
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
import serial.tools.list_ports
from ultralytics import YOLO

# Logging configuration
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(logging.Formatter('%(levelname)s:%(name)s:%(message)s'))
logger.addHandler(console_handler)

# Supported flight modes by vehicle type
SUPPORTED_MODES = {
    "Copter": ["STABILIZE", "GUIDED", "AUTO", "LOITER", "RTL", "LAND"],
    "Plane": ["MANUAL", "FBWA", "FBWB", "AUTO", "RTL", "LOITER", "CRUISE"],
}

class TakeoffDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Takeoff Parametreleri")
        layout = QFormLayout(self)

        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(1, 100)
        self.altitude_spin.setValue(20)
        self.altitude_spin.setSuffix(" m")
        layout.addRow("Yükseklik:", self.altitude_spin)

        self.pitch_spin = QDoubleSpinBox()
        self.pitch_spin.setRange(0, 45)
        self.pitch_spin.setValue(15)
        self.pitch_spin.setSuffix(" °")
        layout.addRow("Pitch:", self.pitch_spin)

        self.airspeed_spin = QDoubleSpinBox()
        self.airspeed_spin.setRange(0, 50)
        self.airspeed_spin.setValue(13)
        self.airspeed_spin.setSuffix(" m/s")
        layout.addRow("Airspeed:", self.airspeed_spin)

        buttons = QHBoxLayout()
        ok_btn = QPushButton("Ekle")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("İptal")
        cancel_btn.clicked.connect(self.reject)
        buttons.addWidget(ok_btn)
        buttons.addWidget(cancel_btn)
        layout.addRow(buttons)

    def get_values(self):
        return self.altitude_spin.value(), self.pitch_spin.value(), self.airspeed_spin.value()

class WebChannelHandler(QObject):
    """Harita ile iletişim için WebChannel handler"""
    waypoint_signal = pyqtSignal(str, float, float, str)

    def __init__(self, parent, uav_type, screen_type):
        super().__init__(parent)
        self.uav_type = uav_type
        self.screen_type = screen_type
        self.parent = parent

    @pyqtSlot(str)
    def pycmd(self, command):
        try:
            if command.startswith("waypoint:"):
                _, latlon = command.split(":")
                lat, lon = map(float, latlon.split(","))
                self.waypoint_signal.emit(self.uav_type, lat, lon, "WAYPOINT")
            elif command.startswith("removeWaypoint:"):
                _, seq = command.split(":")
                self.parent.remove_waypoint_from_js(self.uav_type, int(seq))
        except Exception as e:
            logger.error(f"Komut işleme hatası: {str(e)}")

    @pyqtSlot(str, float, float)
    def addCommandFromContextMenu(self, command, lat, lon):
        self.parent.add_waypoint(self.uav_type, lat, lon, command)

class ConnectThread(QThread):
    """Bağlantı işlemlerini ayrı thread'de yapan sınıf"""
    connection_success_with_string = pyqtSignal(str, object, str)
    connection_error = pyqtSignal(str, str)

    def __init__(self, uav_type, connection_string, baud, parent=None):
        super().__init__(parent)
        self.uav_type = uav_type
        self.connection_string = connection_string
        self.baud = baud

    def run(self):
        start_time = time.perf_counter()
        try:
            if self.baud is not None:
                vehicle = connect(self.connection_string, baud=self.baud, wait_ready=False, source_system=255)
            else:
                vehicle = connect(self.connection_string, wait_ready=False, source_system=255)
            end_time = time.perf_counter()
            print(f"[{self.uav_type}] Bağlantı süresi: {end_time - start_time:.2f} saniye")
            self.connection_success_with_string.emit(self.uav_type, vehicle, self.connection_string)
        except Exception as e:
            end_time = time.perf_counter()
            print(f"[{self.uav_type}] Bağlantı hatası, süre: {end_time - start_time:.2f} saniye")
            self.connection_error.emit(self.uav_type, str(e))

class TelemetryThread(QThread):
    """Telemetri verilerini okuyan thread"""
    telemetry_updated = pyqtSignal(str, dict)
    log_signal = pyqtSignal(str, str, str)

    def __init__(self, vehicle, uav_type, parent):
        super().__init__(parent)
        self.vehicle = vehicle
        self.uav_type = uav_type
        self.parent = parent
        self.running = True

    def run(self):
        self.running = True
        self.log_signal.emit(self.uav_type, "🟢 Telemetri thread başlatıldı.", "SUCCESS")
        try:
            while self.running:
                try:
                    loc = self.vehicle.location.global_frame
                    alt = loc.alt or 0.0
                    lat = loc.lat or 0.0
                    lon = loc.lon or 0.0
                    gps = self.vehicle.gps_0
                    att = self.vehicle.attitude
                    groundspeed = self.vehicle.groundspeed or 0.0
                    vertical_speed = None
                    if self.vehicle.velocity is not None:
                        vertical_speed = self.vehicle.velocity[2] or 0.0
                    else:
                        self.log_signal.emit(self.uav_type, "🔵 vehicle.velocity None döndü.", "INFO")
                    bat = self.vehicle.battery
                    battery_voltage = bat.voltage if bat and bat.voltage else 0.0
                    battery_remaining = bat.level if bat and bat.level is not None else None
                    if battery_remaining is None and battery_voltage > 0:
                        # 3S batarya için yaklaşık hesaplama
                        percentage = min(max((battery_voltage - 10.5) / (12.6 - 10.5), 0), 1) * 100
                        battery_remaining = percentage
                    flight_mode = self.vehicle.mode.name or "Unknown"
                    arm_status = "ARM" if self.vehicle.armed else "DISARM"

                    telemetry = {
                        "Latitude": lat,
                        "Longitude": lon,
                        "Altitude (m)": alt,
                        "Yaw (deg)": att.yaw if hasattr(att, 'yaw') else 0.0,
                        "Ground Speed (kph)": groundspeed * 3.6,
                        "Vertical Speed (kph)": vertical_speed * 3.6 if vertical_speed is not None else 0.0,
                        "Battery (V)": battery_voltage,
                        "Battery (%)": battery_remaining,
                        "Flight Mode": flight_mode,
                        "Arm Status": arm_status
                    }
                    self.telemetry_updated.emit(self.uav_type, telemetry)
                    time.sleep(0.25)  # 4Hz güncelleme
                except Exception as e:
                    self.log_signal.emit(self.uav_type, f"🔴 Telemetri döngü hatası: {str(e)}", "ERROR")
                    traceback.print_exc()
                time.sleep(0.25)
            self.log_signal.emit(self.uav_type, "🔴 Telemetri thread durduruldu.", "INFO")
        except Exception as e:
            self.log_signal.emit(self.uav_type, f"🔴 Telemetri başlatma hatası: {str(e)}", "ERROR")
            traceback.print_exc()
            self.running = False

    def stop(self):
        self.running = False

class ImageProcessingThread(QThread):
    """YOLOv8 görüntü işleme thread'i"""
    image_updated = pyqtSignal(QPixmap)
    detections_signal = pyqtSignal(list)
    fps_signal = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.model = YOLO("yolov8m.engine")
        self.previous_detections = set()
        self.frame_counter = 0
        # İlk "ısınma" için boş bir görüntü gönder
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model.predict(dummy, device='cuda', verbose=False)

    def run(self):
        HOST = '0.0.0.0'
        PORT = 5050
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print("[INFO] GCS TCP alımı başlatıldı:", HOST, PORT)
        conn, addr = server_socket.accept()
        print("[BAĞLANTI] Jetson bağlandı:", addr)

        data_buffer = b''
        frame_times = []

        while self.running:
            try:
                # Veri boyutunu oku
                while len(data_buffer) < 4:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionAbortedError("Veri alınamadı.")
                    data_buffer += packet
                msg_size = struct.unpack(">L", data_buffer[:4])[0]
                data_buffer = data_buffer[4:]

                # Mesajı tamamla
                while len(data_buffer) < msg_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionAbortedError("Veri alınamadı.")
                    data_buffer += packet
                frame_data = data_buffer[:msg_size]
                data_buffer = data_buffer[msg_size:]

                # Görüntüyü çözüp YOLOv8 ile işle
                image = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    print("Görüntü çözülemedi.")
                    continue

                start = time.time()
                results = self.model.predict(image, device='cuda', verbose=False)
                end = time.time()

                # FPS hesapla
                fps = 1 / (end - start)
                frame_times.append(end - start)
                self.fps_signal.emit(fps)

                # 100 frame'de bir ortalama FPS yaz
                if len(frame_times) == 100:
                    avg_fps = 1 / (sum(frame_times) / len(frame_times))
                    print(f"[100 FRAME ORTALAMA FPS] {avg_fps:.2f}")
                    frame_times = []

                # Bounding box'ları çiz
                annotated = results[0].plot()

                # QLabel için dönüştür
                rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                bytes_per_line = ch * w
                image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(image)
                self.image_updated.emit(pixmap)

                # Nesne tespitlerini işleme
                self.frame_counter += 1
                if self.frame_counter >= 100:
                    detected_objects = set()
                    for result in results:
                        for box in result.boxes:
                            class_id = int(box.cls[0])
                            class_name = self.model.names[class_id]
                            detected_objects.add(class_name)

                    new_objects = detected_objects - self.previous_detections
                    self.previous_detections |= new_objects  # Güncelle

                    if new_objects:
                        self.detections_signal.emit(list(new_objects))

                    self.frame_counter = 0  # Sıfırla

            except Exception as e:
                print(f"[HATA] Görüntü işleme hatası: {str(e)}")
                break

        conn.close()
        server_socket.close()

    def stop(self):
        self.running = False

class DualUAVGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # Ekran çözünürlüğüne uygun güvenli geometri ayarı
        desktop = QApplication.primaryScreen().availableGeometry()
        width = min(1920, desktop.width())
        height = min(1001, desktop.height() - 50)  # Görev çubuğu için pay bırak
        self.setGeometry(0, 0, width, height)
        self.setWindowTitle("Çift İHA Kontrol Paneli")
        self.start_time = time.time()
        self.is_dark_mode = True
        self.vehicle_left = None  # Sol panel için Copter
        self.vehicle_right = None  # Sağ panel için Plane
        self.vehicle_type_left = None
        self.vehicle_type_right = None
        self.telemetry_data = {"Sol Panel": {}, "Sağ Panel": {}}
        self.last_position = {"Sol Panel": (0, 0), "Sağ Panel": (0, 0)}
        self.last_heartbeat = {"Sol Panel": 0, "Sağ Panel": 0}
        self.telemetry_threads = {"Sol Panel": None, "Sağ Panel": None}
        self.last_telemetry_check = {"Sol Panel": 0, "Sağ Panel": 0}
        self.last_disconnect_log = {"Sol Panel": 0, "Sağ Panel": 0}
        self.waypoints = {"Sol Panel": [], "Sağ Panel": []}
        self.initial_altitude = {"Sol Panel": None, "Sağ Panel": None}
        self.targets = {"Sol Panel": [], "Sağ Panel": []}
        self.enemy_targets = {"Sol Panel": [], "Sağ Panel": []}
        self.mission_tables = {"Sol Panel": None, "Sağ Panel": None}
        self.plan_maps = {"Sol Panel": None, "Sağ Panel": None}
        self.threads = []  # Tüm thread'leri takip için

        self.mode_colors = {
            "STABILIZE": "yellow",
            "GUIDED": "blue",
            "AUTO": "green",
            "LOITER": "purple",
            "LAND": "orange",
            "RTL": "red",
            "POSHOLD": "cyan",
            "MANUAL": "pink",
            "FBWA": "brown",
            "FBWB": "teal",
            "CRUISE": "lime",
            "UNKNOWN": "gray"
        }

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.sarge_label = QLabel("SARGE")
        self.sarge_label.setAlignment(Qt.AlignCenter)
        self.sarge_label.setStyleSheet("font-size: 32px; font-weight: bold;")
        self.main_layout.addWidget(self.sarge_label)

        top_bar = QWidget()
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.addStretch()
        self.mode_btn = QPushButton("Light Mode")
        self.mode_btn.clicked.connect(self.toggle_mode)
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(6)
        shadow.setOffset(0, 2)
        shadow.setColor(QColor(0, 0, 0, 76))
        self.mode_btn.setGraphicsEffect(shadow)
        top_bar_layout.addWidget(self.mode_btn)
        self.main_layout.addWidget(top_bar)

        self.screens = QTabWidget()
        self.screens.setStyleSheet("border: 2px solid #800080;")
        self.main_layout.addWidget(self.screens)

        self.data_screen = QWidget()
        self.data_layout = QHBoxLayout(self.data_screen)
        self.screens.addTab(self.data_screen, "DATA")

        self.plan_screen = QWidget()
        self.plan_layout = QHBoxLayout(self.plan_screen)
        self.screens.addTab(self.plan_screen, "PLAN")

        self.target_screen = QWidget()
        self.target_layout = QVBoxLayout(self.target_screen)
        self.screens.addTab(self.target_screen, "Hedefler")

        self.left_data_panel = QWidget()
        self.right_data_panel = QWidget()
        self.center_data_panel = QWidget()
        self.left_data_layout = QVBoxLayout(self.left_data_panel)
        self.right_data_layout = QVBoxLayout(self.right_data_panel)
        self.center_data_layout = QVBoxLayout(self.center_data_panel)
        self.data_layout.addWidget(self.left_data_panel)
        self.data_layout.addWidget(self.center_data_panel)
        self.data_layout.addWidget(self.right_data_panel)

        self.left_plan_panel = QWidget()
        self.right_plan_panel = QWidget()
        self.left_plan_layout = QVBoxLayout(self.left_plan_panel)
        self.right_plan_layout = QVBoxLayout(self.right_plan_panel)
        self.plan_layout.addWidget(self.left_plan_panel)
        self.plan_layout.addWidget(self.right_plan_panel)

        self.target_panel = QWidget()
        self.target_layout_widget = QVBoxLayout(self.target_panel)
        self.target_layout.addWidget(self.target_panel)

        self.fps_label = QLabel("FPS: 0.00")
        self.fps_label.setStyleSheet("color: white; font-weight: bold;")
        self.center_data_layout.addWidget(self.fps_label)

        self.yolo_view_1 = QLabel("YOLOv8 Video Paneli")
        self.yolo_view_1.setFixedSize(480, 360)
        self.yolo_view_1.setStyleSheet("background-color: black; color: white; border: 1px solid white")
        self.yolo_view_2 = QTableWidget()
        self.yolo_view_2.setFixedSize(480, 360)
        self.yolo_view_2.setColumnCount(3)
        self.yolo_view_2.setHorizontalHeaderLabels(["Nesne", "Hedef", "İmha"])
        self.center_data_layout.addWidget(self.yolo_view_1)
        self.center_data_layout.addStretch()
        self.center_data_layout.addWidget(self.yolo_view_2)

        self.create_connection_panel(self.left_data_layout, "Sol Panel", "5765", "14550")
        self.create_connection_panel(self.right_data_layout, "Sağ Panel", "5775", "14550")

        self.left_data_map = self.create_visualization_panel(self.left_data_layout, "Sol Panel", "DATA")
        self.right_data_map = self.create_visualization_panel(self.right_data_layout, "Sağ Panel", "DATA")
        self.plan_maps["Sol Panel"] = self.create_visualization_panel(self.left_plan_layout, "Sol Panel", "PLAN")
        self.plan_maps["Sağ Panel"] = self.create_visualization_panel(self.right_plan_layout, "Sağ Panel", "PLAN")

        self.create_log_and_buttons(self.left_data_layout, "Sol Panel")
        self.create_log_and_buttons(self.right_data_layout, "Sağ Panel")
        self.create_telemetry_control_panel(self.left_data_layout, "Sol Panel")
        self.create_telemetry_control_panel(self.right_data_layout, "Sağ Panel")

        self.mission_tables["Sol Panel"] = self.create_plan_table(self.left_plan_layout, "Sol Panel")
        self.mission_tables["Sağ Panel"] = self.create_plan_table(self.right_plan_layout, "Sağ Panel")
        self.create_target_table(self.target_layout_widget, "Hedefler")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        QTimer.singleShot(1000, self.timer.start)
        QTimer.singleShot(0, self.update_ui_mode)

        # Görüntü işleme thread'ini başlat
        self.image_thread = ImageProcessingThread(self)
        self.image_thread.image_updated.connect(self.update_yolo_view)
        self.image_thread.detections_signal.connect(self.handle_detections)
        self.image_thread.fps_signal.connect(self.update_fps)
        self.image_thread.start()
        self.threads.append(self.image_thread)

    def closeEvent(self, event):
        """Uygulama kapanırken tüm thread'leri düzgünce kapat"""
        for thread in self.threads:
            if thread.isRunning():
                thread.quit()
                thread.wait()
        if self.vehicle_left:
            self.vehicle_left.close()
        if self.vehicle_right:
            self.vehicle_right.close()
        event.accept()

    @pyqtSlot(QPixmap)
    def update_yolo_view(self, pixmap):
        self.yolo_view_1.setPixmap(pixmap)

    @pyqtSlot(float)
    def update_fps(self, fps):
        self.fps_label.setText(f"FPS: {fps:.2f}")

    @pyqtSlot(list)
    def handle_detections(self, objects):
        for obj in objects:
            snapshot = "snapshot.jpg"  # ileride görüntüden alınacak
            self.add_target("Sol Panel", obj, snapshot, is_enemy=False)

    def toggle_mode(self):
        self.is_dark_mode = not self.is_dark_mode
        self.update_ui_mode()

    def update_ui_mode(self):
        if not self.isVisible():
            return
        button_style = """
            QPushButton {
                border-radius: 12px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #a670e0;
                color: white;
            }
            QPushButton:pressed {
                background-color: #8a2be2;
            }
        """
        if self.is_dark_mode:
            self.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF;")
            self.mode_btn.setText("Light Mode")
            self.mode_btn.setStyleSheet(button_style + "background-color: #555555; color: #FFFFFF; border: 1px solid #800080;")
            self.sarge_label.setStyleSheet(
                "font-size: 32px; font-weight: bold; background-color: #2E2E2E; color: #FFFFFF;"
                "qproperty-text: '<span style=\"color:#800080;\">S</span><span style=\"color:#FFFFFF;\">A</span>"
                "<span style=\"color:#800080;\">R</span><span style=\"color:#FFFFFF;\">G</span>"
                "<span style=\"color:#800080;\">E</span>';"
            )
            self.screens.setStyleSheet("""
                QTabWidget::pane { background-color: #2E2E2E; border: 2px solid #800080; }
                QTabBar::tab { background-color: #2E2E2E; color: #000000; border: 1px solid #800080; padding: 5px; }
                QTabBar::tab:selected { background-color: #555555; color: #FFFFFF; }
            """)
            self.yolo_view_2.setStyleSheet("""
                QTableWidget { background-color: #2E2E2E; color: white; border: 1px solid #800080; }
                QHeaderView::section { background-color: #2E2E2E; color: white; border: 1px solid #800080; }
            """)
            for widget in (self.left_data_panel.findChildren(QWidget) + self.right_data_panel.findChildren(QWidget) +
                          self.left_plan_panel.findChildren(QWidget) + self.right_plan_panel.findChildren(QWidget) +
                          self.target_panel.findChildren(QWidget)):
                if isinstance(widget, (QLabel, QTextEdit, QTabWidget)):
                    widget.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF; border: 2px solid #800080;")
                elif isinstance(widget, (QComboBox, QLineEdit)):
                    widget.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF; border: 1px solid #800080;")
                elif isinstance(widget, QPushButton):
                    widget.setStyleSheet(button_style + "background-color: #555555; color: #FFFFFF; border: 1px solid #800080;")
                    shadow = QGraphicsDropShadowEffect()
                    shadow.setBlurRadius(6)
                    shadow.setOffset(0, 2)
                    shadow.setColor(QColor(0, 0, 0, 76))
                    widget.setGraphicsEffect(shadow)
                elif isinstance(widget, QCheckBox):
                    widget.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF;")
                elif isinstance(widget, QTableWidget):
                    widget.setStyleSheet("""
                        QTableWidget { background-color: #2E2E2E; color: white; border: 1px solid #800080; }
                        QHeaderView::section { background-color: #2E2E2E; color: white; }
                    """)
        else:
            self.setStyleSheet("background-color: #FFFFFF; color: #000000;")
            self.mode_btn.setText("Dark Mode")
            self.mode_btn.setStyleSheet(button_style + "background-color: #CCCCCC; color: #000000; border: 1px solid #800080;")
            self.sarge_label.setStyleSheet(
                "font-size: 32px; font-weight: bold; background-color: #FFFFFF; color: #000000;"
                "qproperty-text: '<span style=\"color:#800080;\">S</span><span style=\"color:#000000;\">A</span>"
                "<span style=\"color:#800080;\">R</span><span style=\"color:#000000;\">G</span>"
                "<span style=\"color:#800080;\">E</span>';"
            )
            self.screens.setStyleSheet("""
                QTabWidget::pane { background-color: #FFFFFF; border: 2px solid #800080; }
                QTabBar::tab { background-color: #FFFFFF; color: #000000; border: 1px solid #800080; padding: 5px; }
                QTabBar::tab:selected { background-color: #CCCCCC; color: #000000; }
            """)
            self.yolo_view_2.setStyleSheet("""
                QTableWidget { background-color: #FFFFFF; color: black; border: 1px solid #800080; }
                QHeaderView::section { background-color: #FFFFFF; color: black; border: 1px solid #800080; }
            """)
            for widget in (self.left_data_panel.findChildren(QWidget) + self.right_data_panel.findChildren(QWidget) +
                          self.left_plan_panel.findChildren(QWidget) + self.right_plan_panel.findChildren(QWidget) +
                          self.target_panel.findChildren(QWidget)):
                if isinstance(widget, (QLabel, QLineEdit, QComboBox, QTextEdit, QTabWidget)):
                    widget.setStyleSheet("background-color: #FFFFFF; color: #000000; border: 1px solid #800080;")
                elif isinstance(widget, QPushButton):
                    widget.setStyleSheet(button_style + "background-color: #CCCCCC; color: #000000; border: 1px solid #800080;")
                    shadow = QGraphicsDropShadowEffect()
                    shadow.setBlurRadius(6)
                    shadow.setOffset(0, 2)
                    shadow.setColor(QColor(0, 0, 0, 76))
                    widget.setGraphicsEffect(shadow)
                elif isinstance(widget, QCheckBox):
                    widget.setStyleSheet("background-color: #FFFFFF; color: #000000;")
                elif isinstance(widget, QTableWidget):
                    widget.setStyleSheet("""
                        QTableWidget { background-color: #FFFFFF; color: black; border: 1px solid #800080; }
                        QHeaderView::section { background-color: #FFFFFF; color: black; }
                    """)
        # Flight mode and arm status labels' border
        for label in [self.left_flight_mode_label, self.right_flight_mode_label, 
                     self.left_arm_status_label, self.right_arm_status_label]:
            if label:
                current_style = label.styleSheet()
                new_style = current_style + " border: 2px solid #800080;"
                label.setStyleSheet(new_style)

    def create_connection_panel(self, layout, uav_type, default_port, default_remote_port):
        connection_frame = QWidget()
        connection_frame.setFixedHeight(50)
        connection_layout = QHBoxLayout(connection_frame)

        connection_type = QComboBox()
        connection_type.addItems(["TCP", "UDP", "Seri Port"])
        connection_type.setCurrentText("TCP")

        port_label = QLabel("Port:")
        port_input = QLineEdit(default_port)

        host_label = QLabel("Host IP:")
        host_input = QLineEdit("127.0.0.1")

        com_port_label = QLabel("COM Port:")
        com_port_combo = QComboBox()
        available_ports = get_available_ports()
        com_port_combo.addItems(available_ports if available_ports else ["Port Bulunamadı"])
        com_port_combo.setCurrentText("COM3" if "COM3" in available_ports else (available_ports[0] if available_ports else "Port Bulunamadı"))

        baud_label = QLabel("Baud Rate:")
        baud_input = QComboBox()
        baud_input.addItems(["57600", "111100", "115200"])
        baud_input.setCurrentText("57600")

        connect_btn = QPushButton("BAĞLAN")
        connect_btn.clicked.connect(lambda: self.connect_uav(uav_type, host_input.text(), port_input.text(), connection_type.currentText(), "", com_port_combo.currentText(), baud_input.currentText()))
        disconnect_btn = QPushButton("KES")
        disconnect_btn.clicked.connect(lambda: self.disconnect_uav(uav_type))

        connection_layout.addWidget(connection_type)
        connection_layout.addWidget(port_label)
        connection_layout.addWidget(port_input)
        connection_layout.addWidget(host_label)
        connection_layout.addWidget(host_input)
        connection_layout.addWidget(com_port_label)
        connection_layout.addWidget(com_port_combo)
        connection_layout.addWidget(baud_label)
        connection_layout.addWidget(baud_input)
        connection_layout.addWidget(connect_btn)
        connection_layout.addWidget(disconnect_btn)

        def update_connection_fields(text):
            port_label.setVisible(text in ["TCP", "UDP"])
            port_input.setVisible(text in ["TCP", "UDP"])
            host_label.setVisible(text == "TCP")
            host_input.setVisible(text == "TCP")
            com_port_label.setVisible(text == "Seri Port")
            com_port_combo.setVisible(text == "Seri Port")
            baud_label.setVisible(text == "Seri Port")
            baud_input.setVisible(text == "Seri Port")
            if text == "UDP":
                port_input.setText("14550")
            elif text == "TCP":
                port_input.setText("5765" if uav_type == "Sol Panel" else "5775")
            elif text == "Seri Port":
                port_input.setText("")

        connection_type.currentTextChanged.connect(update_connection_fields)
        update_connection_fields("TCP")
        connection_frame.setStyleSheet("border: 2px solid #800080; padding: 5px;")
        layout.addWidget(connection_frame)

    def create_visualization_panel(self, layout, uav_type, screen_type):
        vis_container = QWidget()
        vis_layout = QVBoxLayout(vis_container)

        map_widget = QWebEngineView()
        if screen_type == "PLAN":
            map_widget.setFixedSize(800, 500)
        else:
            map_widget.setFixedSize(600, 400)
        map_widget.setMinimumWidth(600)
        map_widget.setMinimumHeight(300)

        channel = QWebChannel(map_widget.page())
        handler = WebChannelHandler(self, uav_type, screen_type)
        channel.registerObject("handler", handler)
        map_widget.page().setWebChannel(channel)
        handler.waypoint_signal.connect(self.add_waypoint)

        initial_lat = self.telemetry_data.get(uav_type, {}).get("Latitude", 39.925533)
        initial_lon = self.telemetry_data.get(uav_type, {}).get("Longitude", 32.866287)
        initial_zoom = 18 if screen_type == "PLAN" and initial_lat != 0 and initial_lon != 0 else 6

        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
            <style>
                html, body, #map {{ height: 100%; width: 100%; margin: 0; }}
                .leaflet-popup-content {{ padding: 5px; font-size: 12px; }}
                .leaflet-popup-content div:hover {{ background-color: #f0f0f0; }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <button id="droneBtn" onclick="zoomToDrone()" style="position:absolute;bottom:10px;right:10px;z-index:1000;background:#800080;color:white;border-radius:5px;">📍 Drone</button>
            <script>
                var map = L.map('map').setView([{initial_lat}, {initial_lon}], {initial_zoom});
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    attribution: '© OpenStreetMap contributors'
                }}).addTo(map);
                var pixhawkMarker = null;
                var pixhawkDirectionLine = null;
                var waypointMarkers = [];
                var markerMap = {{}};
                var shouldFollow = false;

                window.onresize = function() {{
                    var droneButton = document.getElementById('droneBtn');
                    droneButton.style.bottom = "10px";
                    droneButton.style.right = "10px";
                }};

                window.updateMarkers = function(pixhawkLat, pixhawkLon, pixhawkYaw) {{
                    console.log("Updating markers: ", pixhawkLat, pixhawkLon, pixhawkYaw);
                    if (pixhawkLat === 0 || pixhawkLon === 0) {{
                        console.log("Sıfır koordinatlar, marker güncellenmedi.");
                        return;
                    }}
                    if (!pixhawkMarker) {{
                        console.log("Yeni marker oluşturuluyor.");
                        pixhawkMarker = L.marker([pixhawkLat, pixhawkLon], {{icon: L.divIcon({{className: 'pixhawk-icon'}})}}).addTo(map);
                    }} else {{
                        console.log("Mevcut marker güncelleniyor.");
                        pixhawkMarker.setLatLng([pixhawkLat, pixhawkLon]);
                    }}
                    if (shouldFollow) {{
                        map.panTo([pixhawkLat, pixhawkLon]);
                    }}
                    if (pixhawkDirectionLine) {{
                        map.removeLayer(pixhawkDirectionLine);
                    }}
                    var pixhawkLength = 0.1 * (map.getZoom() / 10);
                    var pixhawkEndLat = pixhawkLat + pixhawkLength * Math.cos((pixhawkYaw - 90) * Math.PI / 180);
                    var pixhawkEndLon = pixhawkLon + pixhawkLength * Math.sin((pixhawkYaw - 90) * Math.PI / 180);
                    pixhawkDirectionLine = L.polyline([[pixhawkLat, pixhawkLon], [pixhawkEndLat, pixhawkEndLon]], {{
                        color: 'red',
                        weight: 2
                    }}).addTo(map);
                }};

                window.addWaypointMarker = function(lat, lon, label, seq) {{
                    var marker = L.marker([lat, lon], {{icon: L.divIcon({{className: 'waypoint-icon'}})}}).addTo(map);
                    marker.customId = seq;
                    marker.bindTooltip(label, {{permanent: true, direction: 'top'}}).openTooltip();
                    waypointMarkers.push(marker);
                    markerMap[seq] = marker;
                }};

                window.removeWaypointMarker = function(seq) {{
                    if (markerMap[seq]) {{
                        map.removeLayer(markerMap[seq]);
                        delete markerMap[seq];
                        for (var i = 0; i < waypointMarkers.length; i++) {{
                            if (waypointMarkers[i].customId === seq) {{
                                waypointMarkers.splice(i, 1);
                                break;
                            }}
                        }}
                    }}
                }};

                window.removeMarker = function(seq) {{
                    if (markerMap[seq]) {{
                        map.removeLayer(markerMap[seq]);
                        delete markerMap[seq];
                    }}
                }};

                function zoomToDrone() {{
                    if (pixhawkMarker) {{
                        shouldFollow = true;
                        map.setView(pixhawkMarker.getLatLng(), 18);
                    }}
                }}

                map.on('zoomstart', function() {{
                    shouldFollow = false;
                }});

                function add_waypoint(lat, lon) {{
                    var marker = L.marker([lat, lon], {{icon: L.divIcon({{className: 'waypoint-icon'}})}}).addTo(map);
                    new QWebChannel(qt.webChannelTransport, function(channel) {{
                        channel.objects.handler.pycmd('waypoint:' + lat + ',' + lon);
                    }});
                    lastClickedLatLng = {{ lat: lat, lon: lon }};
                }}

                function sendCommand(command, lat, lon) {{
                    if (command === 'TAKEOFF') {{
                        new QWebChannel(qt.webChannelTransport, function(channel) {{
                            channel.objects.handler.addCommandFromContextMenu(command, lat, lon);
                        }});
                    }} else {{
                        new QWebChannel(qt.webChannelTransport, function(channel) {{
                            channel.objects.handler.addCommandFromContextMenu(command, lat, lon);
                        }});
                    }}
                    map.closePopup();
                }}

                map.on('click', function(e) {{
                    var lat = e.latlng.lat;
                    var lon = e.latlng.lng;
                    add_waypoint(lat, lon);
                }});

                map.on('contextmenu', function(e) {{
                    var lat = e.latlng.lat;
                    var lon = e.ltlng.lng;
                    var popup = L.popup()
                        .setLatLng([lat, lon])
                        .setContent(`
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('TAKEOFF', ${{lat}}, ${{lon}})">Takeoff</div>
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('WAYPOINT', ${{lat}}, ${{lon}})">Waypoint</div>
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('LOITER_TURNS', ${{lat}}, ${{lon}})">Loiter Turns</div>
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('DO_CHANGE_SPEED', ${{lat}}, ${{lon}})">Change Speed</div>
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('RTL', ${{lat}}, ${{lon}})">RTL</div>
                            <div style="cursor:pointer;padding:10px;font-size:12px;" onclick="sendCommand('LAND', ${{lat}}, ${{lon}})">Land</div>
                        `)
                        .openOn(map);
                }});

                var style = document.createElement('style');
                style.type = 'text/css';
                style.innerHTML = '.pixhawk-icon {{ width: 10px; height: 10px; background-color: blue; border-radius: 50%; }} .waypoint-icon {{ width: 8px; height: 8px; background-color: green; border-radius: 50%; }}';
                document.getElementsByTagName('head')[0].appendChild(style);
            </script>
        </body>
        </html>
        """
        map_widget.setHtml(html)
        vis_layout.addWidget(map_widget)
        layout.addWidget(vis_container)
        return map_widget

    def create_log_and_buttons(self, layout, uav_type):
        log_container = QWidget()
        log_layout = QVBoxLayout(log_container)

        mavlink_logs = QTextEdit()
        mavlink_logs.setReadOnly(True)
        mavlink_logs.setPlaceholderText(f"{uav_type} MAVLink Logları (Anlık Veriler)")
        mavlink_logs.setStyleSheet("min-height: 100px; font-family: 'Arial'; font-size: 12px; border: 2px solid #800080;")
        log_layout.addWidget(mavlink_logs)

        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setAlignment(Qt.AlignRight | Qt.AlignBottom)
        takeoff_btn = QPushButton("Takeoff")
        takeoff_alt_input = QLineEdit()
        takeoff_alt_input.setPlaceholderText("m")
        takeoff_alt_input.setFixedWidth(50)
        takeoff_btn.clicked.connect(lambda: self.direct_takeoff(uav_type, float(takeoff_alt_input.text()) if takeoff_alt_input.text() else 10.0))
        land_btn = QPushButton("Land")
        land_btn.clicked.connect(lambda: self.land(uav_type))
        rtl_btn = QPushButton("RTL")
        rtl_btn.clicked.connect(lambda: self.rtl(uav_type))
        clear_log_btn = QPushButton("Temizle")
        clear_log_btn.clicked.connect(lambda: mavlink_logs.clear())
        export_log_btn = QPushButton("Kaydet")
        export_log_btn.clicked.connect(lambda: self.export_log(mavlink_logs.toPlainText(), uav_type))
        button_layout.addWidget(takeoff_btn)
        button_layout.addWidget(takeoff_alt_input)
        button_layout.addWidget(land_btn)
        button_layout.addWidget(rtl_btn)
        button_layout.addWidget(clear_log_btn)
        button_layout.addWidget(export_log_btn)
        log_layout.addWidget(button_container)

        layout.addWidget(log_container)

        if uav_type == "Sol Panel":
            self.left_mavlink_logs = mavlink_logs
        elif uav_type == "Sağ Panel":
            self.right_mavlink_logs = mavlink_logs

    def export_log(self, log_text, uav_type):
        file_name, _ = QFileDialog.getSaveFileName(self, "Log'u Kaydet", f"log_{uav_type}_{time.strftime('%Y%m%d_%H%M%S')}.txt", "Text Files (*.txt)")
        if file_name:
            with open(file_name, 'w', encoding='utf-8') as file:
                file.write(log_text)

    def make_delete_func(self, btn, table, uav_type):
        def delete():
            index = table.indexAt(btn.pos())
            if index.isValid():
                self.remove_waypoint(uav_type, index.row())
        return delete

    def create_plan_table(self, layout, uav_type):
        table_container = QWidget()
        table_layout = QVBoxLayout(table_container)

        table = QTableWidget()
        table.setColumnCount(8)
        table.setHorizontalHeaderLabels(["Seq", "Command", "Lat", "Long", "Alt (m)", "Speed (m/s)", "Delay (s)", "Delete"])
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
        table_layout.addWidget(table)

        button_layout = QHBoxLayout()
        write_btn = QPushButton("Write")
        write_btn.clicked.connect(lambda: self.write_mission(uav_type))
        write_btn.setEnabled(False)
        start_mission_btn = QPushButton("Start Mission")
        start_mission_btn.clicked.connect(lambda: self.start_mission(uav_type))
        start_mission_btn.setEnabled(False)
        button_layout.addWidget(write_btn)
        button_layout.addWidget(start_mission_btn)
        table_layout.addLayout(button_layout)

        layout.addWidget(table_container)
        if uav_type == "Sol Panel":
            self.left_write_btn = write_btn
            self.left_do_action_btn = start_mission_btn
        elif uav_type == "Sağ Panel":
            self.right_write_btn = write_btn
            self.right_do_action_btn = start_mission_btn
        return table

    def create_target_table(self, layout, uav_type):
        table_container = QWidget()
        table_layout = QVBoxLayout(table_container)

        table = QTableWidget()
        table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        table.horizontalHeader().setStretchLastSection(True)
        table.setColumnCount(6)
        table.setHorizontalHeaderLabels(["Nesne", "Resim (Snapshot)", "Hedef", "Sol WP", "Sağ WP", "İmha"])
        table.setRowCount(0)
        table.setMinimumHeight(200)
        table.setColumnWidth(0, 100)
        table.setColumnWidth(1, 150)
        table.setColumnWidth(2, 80)
        table.setColumnWidth(3, 80)
        table.setColumnWidth(4, 80)
        table.setColumnWidth(5, 80)
        table_layout.addWidget(table)
        table_layout.setAlignment(Qt.AlignCenter)

        layout.addWidget(table_container)
        self.target_table = table

    def create_telemetry_control_panel(self, layout, uav_type):
        control_frame = QWidget()
        control_layout = QHBoxLayout(control_frame)

        flight_mode_label = QLabel("Uçuş Modu: Unknown")
        flight_mode_label.setStyleSheet("background-color: gray; color: white; padding: 5px; border: 2px solid #800080;")
        control_layout.addWidget(flight_mode_label)

        flight_mode_combo = QComboBox()
        # Mod menüsünü araç tipine göre ayarla
        if uav_type == "Sol Panel":
            modes = SUPPORTED_MODES["Copter"]  # Sol panel Copter
        else:
            modes = SUPPORTED_MODES["Plane"]  # Sağ panel Plane
        flight_mode_combo.addItems(modes)
        flight_mode_combo.setEnabled(False)
        control_layout.addWidget(flight_mode_combo)

        flight_mode_confirm_btn = QPushButton("✓")
        flight_mode_confirm_btn.clicked.connect(lambda: self.set_flight_mode(uav_type, flight_mode_combo.currentText()))
        control_layout.addWidget(flight_mode_confirm_btn)

        arm_status_label = QLabel("Motor Durumu: DISARM")
        arm_status_label.setStyleSheet("background-color: red; color: white; padding: 5px; border: 2px solid #800080;")
        control_layout.addWidget(arm_status_label)

        arm_combo = QComboBox()
        arm_combo.addItems(["ARM", "DISARM"])
        arm_combo.setEnabled(False)
        control_layout.addWidget(arm_combo)

        arm_confirm_btn = QPushButton("✓")
        arm_confirm_btn.clicked.connect(lambda: self.set_arm_status(uav_type, arm_combo.currentText() == "ARM"))
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
            ("Dist to WP (m)", QColor("red")),
            ("Vertical Speed (kph)", QColor("yellow")),
            ("Yaw (deg)", QColor("green")),
            ("Dist to MAV", QColor("blue")),
            ("Battery", QColor("cyan")),
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

    def connect_uav(self, uav_type, host, port, protocol, remote_port, com_port, baud_rate):
        if (uav_type == "Sol Panel" and self.vehicle_left is not None) or (uav_type == "Sağ Panel" and self.vehicle_right is not None):
            self.log_message(uav_type, "Zaten bağlı!", "WARNING")
            return

        try:
            if protocol == "TCP":
                connection_string = f"tcp:{host}:{port}"
                self.log_message(uav_type, f"Bağlantı string: {connection_string}", "INFO")
            elif protocol == "UDP":
                connection_string = f"udp:0.0.0.0:{port}"
                self.log_message(uav_type, f"Bağlantı string: {connection_string}", "INFO")
            else:  # Seri Port
                connection_string = f"{com_port}"

            self.log_message(uav_type, f"{connection_string} adresine bağlanıyor...", "INFO")
            baud = int(baud_rate) if protocol == "Seri Port" else None
            connect_thread = ConnectThread(uav_type, connection_string, baud, self)
            connect_thread.connection_success_with_string.connect(self.on_connection_success)
            connect_thread.connection_error.connect(self.on_connection_error)
            connect_thread.start()
            self.threads.append(connect_thread)

        except Exception as e:
            self.log_message(uav_type, f"Bağlantı hatası: {str(e)}", "ERROR")

    @pyqtSlot(str, object, str)
    def on_connection_success(self, uav_type, vehicle, connection_string):
        if uav_type == "Sol Panel":
            self.vehicle_left = vehicle
            self.vehicle_type_left = "Copter" if vehicle._vehicle_type == 2 else "Unknown"
        elif uav_type == "Sağ Panel":
            self.vehicle_right = vehicle
            self.vehicle_type_right = "Plane" if vehicle._vehicle_type == 1 else "Unknown"

        self.log_message(uav_type, f"{connection_string} adresine bağlanıldı. Araç tipi: {self.vehicle_type_left if uav_type == 'Sol Panel' else self.vehicle_type_right}", "SUCCESS")

        print(f"Latitude: {vehicle.location.global_frame.lat}")
        print(f"Longitude: {vehicle.location.global_frame.lon}")
        print(f"Heading: {vehicle.heading}")

        telemetry_thread = TelemetryThread(vehicle, uav_type, self)
        self.telemetry_threads[uav_type] = telemetry_thread
        telemetry_thread.telemetry_updated.connect(self.update_telemetry)
        telemetry_thread.log_signal.connect(self.log_message_from_thread)
        telemetry_thread.start()
        self.threads.append(telemetry_thread)

        if uav_type == "Sol Panel":
            self.left_flight_mode_combo.setEnabled(True)
            self.left_arm_combo.setEnabled(True)
            self.left_write_btn.setEnabled(True)
            self.left_do_action_btn.setEnabled(True)
        elif uav_type == "Sağ Panel":
            self.right_flight_mode_combo.setEnabled(True)
            self.right_arm_combo.setEnabled(True)
            self.right_write_btn.setEnabled(True)
            self.right_do_action_btn.setEnabled(True)

    @pyqtSlot(str, str)
    def on_connection_error(self, uav_type, error_message):
        self.log_message(uav_type, f"Bağlantı hatası: {error_message}", "ERROR")

    def disconnect_uav(self, uav_type):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            if self.telemetry_threads.get(uav_type):
                self.telemetry_threads[uav_type].stop()
                self.telemetry_threads[uav_type].wait()
                self.threads.remove(self.telemetry_threads[uav_type])
                self.telemetry_threads[uav_type] = None
            vehicle.close()
            if uav_type == "Sol Panel":
                self.vehicle_left = None
                self.vehicle_type_left = None
            else:
                self.vehicle_right = None
                self.vehicle_type_right = None
            self.telemetry_data[uav_type] = {}
            self.initial_altitude[uav_type] = None
            self.targets[uav_type].clear()
            self.enemy_targets[uav_type].clear()
            self.log_message(uav_type, "Bağlantı kesiliyor...", "INFO")
            self.log_message(uav_type, "Bağlantı kesildi!", "ERROR")

            if uav_type == "Sol Panel":
                self.left_flight_mode_combo.setEnabled(False)
                self.left_arm_combo.setEnabled(False)
                self.left_write_btn.setEnabled(False)
                self.left_do_action_btn.setEnabled(False)
            elif uav_type == "Sağ Panel":
                self.right_flight_mode_combo.setEnabled(False)
                self.right_arm_combo.setEnabled(False)
                self.right_write_btn.setEnabled(False)
                self.right_do_action_btn.setEnabled(False)

    @pyqtSlot(str, dict)
    def update_telemetry(self, uav_type, telemetry):
        if self.initial_altitude[uav_type] is None and telemetry.get("Altitude (m)", 0) != 0:
            self.initial_altitude[uav_type] = telemetry["Altitude (m)"]
        if self.initial_altitude[uav_type] is not None:
            telemetry["Altitude (m)"] = telemetry["Altitude (m)"] - self.initial_altitude[uav_type]
        self.telemetry_data[uav_type] = telemetry
        QTimer.singleShot(0, self.update_data)

    def set_flight_mode(self, uav_type, mode):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        vehicle_type = self.vehicle_type_left if uav_type == "Sol Panel" else self.vehicle_type_right
        if vehicle:
            try:
                mode_upper = mode.upper()
                if mode_upper not in SUPPORTED_MODES.get(vehicle_type, []):
                    self.log_message(uav_type, f"⚠ Uçuş modu {mode_upper}, {vehicle_type} için desteklenmiyor", "WARNING")
                    return

                vehicle.mode = VehicleMode(mode_upper)
                for _ in range(10):  # 2 saniyeye kadar bekle
                    time.sleep(0.2)
                    if vehicle.mode.name == mode_upper:
                        break
                if vehicle.mode.name != mode_upper:
                    self.log_message(uav_type, f"❌ Uçuş modu {mode_upper} olarak ayarlanamadı.", "ERROR")
                else:
                    self.log_message(uav_type, f"Uçuş modu {mode_upper} olarak ayarlandı.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Uçuş modu ayarlama hatası: {str(e)}", "ERROR")

    def set_arm_status(self, uav_type, arm):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                vehicle.armed = arm
                for _ in range(5):
                    time.sleep(0.5)
                    if vehicle.armed == arm:
                        break
                if vehicle.armed != arm:
                    self.log_message(uav_type, f"❌ Motor durumu {'ARM' if arm else 'DISARM'} olarak ayarlanamadı.", "ERROR")
                else:
                    self.log_message(uav_type, f"Motor durumu {'ARM' if arm else 'DISARM'} olarak ayarlandı.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Motor durumu ayarlama hatası: {str(e)}", "ERROR")

    def direct_takeoff(self, uav_type, altitude):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                vehicle.mode = VehicleMode("GUIDED")
                self.set_arm_status(uav_type, True)
                vehicle.simple_takeoff(altitude)
                self.log_message(uav_type, f"Takeoff komutu {altitude} metre irtifa için gönderildi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Takeoff hatası: {str(e)}", "ERROR")

    def takeoff(self, uav_type, altitude):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                vehicle.mode = VehicleMode("GUIDED")
                self.set_arm_status(uav_type, True)
                vehicle.simple_takeoff(altitude)
                self.log_message(uav_type, f"Takeoff komutu {altitude} metre irtifa için gönderildi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Takeoff hatası: {str(e)}", "ERROR")

    def land(self, uav_type):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                vehicle.mode = VehicleMode("LAND")
                self.log_message(uav_type, "Land komutu gönderildi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Land hatası: {str(e)}", "ERROR")

    def rtl(self, uav_type):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                vehicle.mode = VehicleMode("RTL")
                self.log_message(uav_type, "RTL komutu gönderildi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"RTL hatası: {str(e)}", "ERROR")

    @pyqtSlot(str, float, float, str)
    def add_waypoint(self, uav_type, lat, lon, command_type):
        table = self.mission_tables[uav_type]
        map_widget = self.plan_maps[uav_type]
        row_count = table.rowCount()
        
        if command_type == "TAKEOFF":
            dialog = TakeoffDialog(self)
            if dialog.exec_():
                altitude, pitch, airspeed = dialog.get_values()
                table.insertRow(row_count)
                seq = row_count + 1
                label = f"{seq}"
                table.setItem(row_count, 0, QTableWidgetItem(str(seq)))
                table.setItem(row_count, 1, QTableWidgetItem("TAKEOFF"))
                table.setItem(row_count, 2, QTableWidgetItem(f"{lat:.6f}"))
                table.setItem(row_count, 3, QTableWidgetItem(f"{lon:.6f}"))
                table.setItem(row_count, 4, QTableWidgetItem(f"{altitude:.2f}"))
                table.setItem(row_count, 5, QTableWidgetItem(f"{airspeed:.2f}"))
                table.setItem(row_count, 6, QTableWidgetItem("0.00"))
                delete_btn = QPushButton("Sil")
                delete_btn.clicked.connect(self.make_delete_func(delete_btn, table, uav_type))
                table.setCellWidget(row_count, 7, delete_btn)
                self.waypoints[uav_type].append({
                    "seq": seq,
                    "command": "TAKEOFF",
                    "lat": lat,
                    "lon": lon,
                    "alt": altitude,
                    "pitch": pitch,
                    "airspeed": airspeed,
                    "speed": airspeed,
                    "delay": 0.0
                })
                self.log_message(uav_type, f"TAKEOFF ({lat:.6f}, {lon:.6f}, {altitude}m) eklendi.", "SUCCESS")
                js_command = f"addWaypointMarker({lat}, {lon}, '{label}', {seq})"
                map_widget.page().runJavaScript(js_command)

                # DO_CHANGE_SPEED ekle
                table.insertRow(row_count + 1)
                seq += 1
                table.setItem(row_count + 1, 0, QTableWidgetItem(str(seq)))
                table.setItem(row_count + 1, 1, QTableWidgetItem("DO_CHANGE_SPEED"))
                table.setItem(row_count + 1, 2, QTableWidgetItem("0.000000"))
                table.setItem(row_count + 1, 3, QTableWidgetItem("0.000000"))
                table.setItem(row_count + 1, 4, QTableWidgetItem("0.00"))
                table.setItem(row_count + 1, 5, QTableWidgetItem(f"{airspeed:.2f}"))
                table.setItem(row_count + 1, 6, QTableWidgetItem("0.00"))
                delete_btn = QPushButton("Sil")
                delete_btn.clicked.connect(self.make_delete_func(delete_btn, table, uav_type))
                table.setCellWidget(row_count + 1, 7, delete_btn)
                self.waypoints[uav_type].append({
                    "seq": seq,
                    "command": "DO_CHANGE_SPEED",
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "speed": airspeed,
                    "delay": 0.0
                })
                self.log_message(uav_type, f"DO_CHANGE_SPEED ({airspeed} m/s) eklendi.", "SUCCESS")
        else:
            table.insertRow(row_count)
            if command_type == "LOITER_TURNS":
                command = "LOITER_TURNS"
                alt = 10.0
                speed = 5.0
                delay = 0.0
            elif command_type == "DO_CHANGE_SPEED":
                command = "DO_CHANGE_SPEED"
                alt = 0.0
                speed = 15.0
                delay = 0.0
            elif command_type == "RTL":
                command = "RTL"
                alt = 0.0
                speed = 5.0
                delay = 0.0
            elif command_type == "LAND":
                command = "LAND"
                alt = 0.0
                speed = 5.0
                delay = 0.0
            else:
                command = "WAYPOINT"
                alt = 10.0
                speed = 5.0
                delay = 0.0

            seq = row_count + 1
            label = f"{seq}"

            table.setItem(row_count, 0, QTableWidgetItem(str(seq)))
            table.setItem(row_count, 1, QTableWidgetItem(command))
            table.setItem(row_count, 2, QTableWidgetItem(f"{lat:.6f}"))
            table.setItem(row_count, 3, QTableWidgetItem(f"{lon:.6f}"))
            table.setItem(row_count, 4, QTableWidgetItem(f"{alt:.2f}"))
            table.setItem(row_count, 5, QTableWidgetItem(f"{speed:.2f}"))
            table.setItem(row_count, 6, QTableWidgetItem(f"{delay:.2f}"))

            delete_btn = QPushButton("Sil")
            delete_btn.clicked.connect(self.make_delete_func(delete_btn, table, uav_type))
            table.setCellWidget(row_count, 7, delete_btn)

            self.waypoints[uav_type].append({
                "seq": seq,
                "command": command,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "speed": speed,
                "delay": delay
            })
            self.log_message(uav_type, f"{command} ({lat:.6f}, {lon:.6f}) eklendi.", "SUCCESS")

            js_command = f"addWaypointMarker({lat}, {lon}, '{label}', {seq})"
            map_widget.page().runJavaScript(js_command)

    def remove_waypoint_from_js(self, uav_type, seq):
        for i, wp in enumerate(self.waypoints[uav_type]):
            if wp["seq"] == seq:
                self.waypoints[uav_type].pop(i)
                break
        self.log_message(uav_type, f"Waypoint {seq} silindi.", "SUCCESS")
        js_command = f"removeWaypointMarker({seq})"
        map_widget = self.plan_maps[uav_type]
        map_widget.page().runJavaScript(js_command)

    def remove_waypoint(self, uav_type, row):
        table = self.mission_tables[uav_type]
        item = table.item(row, 0)
        if item is None:
            self.log_message(uav_type, f"Silinmeye çalışılan satır ({row}) bulunamadı.", "ERROR")
            return
        seq = int(item.text())
        table.removeRow(row)
        self.remove_waypoint_from_js(uav_type, seq)
        self.plan_maps[uav_type].page().runJavaScript(f"removeMarker({seq})")
        self.log_message(uav_type, f"Waypoint {seq} silindi.", "SUCCESS")

    def write_mission(self, uav_type):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                cmds = vehicle.commands
                cmds.clear()
                for wp in self.waypoints[uav_type]:
                    if wp["command"] == "TAKEOFF":
                        cmd = Command(0, 0, 0, 3, 22, 0, 0, wp.get("pitch", 15), 0, 0, 0, wp["lat"], wp["lon"], wp["alt"])
                    elif wp["command"] == "RTL":
                        cmd = Command(0, 0, 0, 3, 20, 0, 0, 0, 0, 0, 0, wp["lat"], wp["lon"], wp["alt"])
                    elif wp["command"] == "LAND":
                        cmd = Command(0, 0, 0, 3, 21, 0, 0, 0, 0, 0, 0, wp["lat"], wp["lon"], wp["alt"])
                    elif wp["command"] == "LOITER_TURNS":
                        cmd = Command(0, 0, 0, 3, 18, 0, 0, 3, 100, 0, 0, wp["lat"], wp["lon"], wp["alt"])
                    elif wp["command"] == "DO_CHANGE_SPEED":
                        cmd = Command(0, 0, 0, 3, 178, 0, 0, 1, wp["speed"], -1, 0, 0, 0, 0)
                    else:
                        cmd = Command(0, 0, 0, 3, 16, 0, 0, 0, 0, 0, 0, wp["lat"], wp["lon"], wp["alt"])
                    cmds.add(cmd)
                cmds.upload()
                self.log_message(uav_type, "Waypointlar yüklendi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"Waypoint yükleme hatası: {str(e)}", "ERROR")

    def start_mission(self, uav_type):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if vehicle:
            try:
                self.write_mission(uav_type)
                self.set_arm_status(uav_type, True)
                vehicle.mode = VehicleMode("AUTO")
                self.log_message(uav_type, "AUTO moduna geçildi.", "SUCCESS")
            except Exception as e:
                self.log_message(uav_type, f"AUTO modu hatası: {str(e)}", "ERROR")

    def add_target(self, uav_type, obj_type, snapshot, is_enemy=False, lat=None, lon=None):
        if hasattr(self, 'target_table'):
            table = self.target_table
            row_count = table.rowCount()
            table.insertRow(row_count)

            obj_item = QTableWidgetItem(obj_type)
            snapshot_item = QTableWidgetItem(snapshot)
            enemy_check = QCheckBox()
            enemy_check.setChecked(is_enemy)
            enemy_check.stateChanged.connect(lambda state, r=row_count, u=uav_type: self.update_enemy_targets(u, r, state == Qt.Checked))
            left_wp_btn = QPushButton("Sol WP")
            left_wp_btn.clicked.connect(lambda r=row_count: self.add_wp_from_target(uav_type, r, "Sol"))
            right_wp_btn = QPushButton("Sağ WP")
            right_wp_btn.clicked.connect(lambda r=row_count: self.add_wp_from_target(uav_type, r, "Sağ"))
            destroy_btn = QPushButton("İmha")
            destroy_btn.clicked.connect(lambda r=row_count: self.destroy_target(uav_type, r))
            destroy_btn.setEnabled(is_enemy)

            # Disable buttons if lat/lon is None
            has_location = lat is not None and lon is not None
            left_wp_btn.setEnabled(has_location)
            right_wp_btn.setEnabled(has_location)
            destroy_btn.setEnabled(is_enemy and has_location)

            table.setItem(row_count, 0, obj_item)
            table.setItem(row_count, 1, snapshot_item)
            table.setCellWidget(row_count, 2, enemy_check)
            table.setCellWidget(row_count, 3, left_wp_btn)
            table.setCellWidget(row_count, 4, right_wp_btn)
            table.setCellWidget(row_count, 5, destroy_btn)

            self.targets[uav_type].append({"obj_type": obj_type, "snapshot": snapshot, "is_enemy": is_enemy, "lat": lat, "lon": lon, "row": row_count})
            if is_enemy:
                self.enemy_targets[uav_type].append({"obj_type": obj_type, "snapshot": snapshot, "lat": lat, "lon": lon, "row": row_count})
            self.update_enemy_panel(uav_type)

    def update_enemy_targets(self, uav_type, row, is_enemy):
        if hasattr(self, 'target_table'):
            target = self.targets[uav_type][row]
            target["is_enemy"] = is_enemy
            has_location = target["lat"] is not None and target["lon"] is not None
            self.target_table.cellWidget(row, 5).setEnabled(is_enemy and has_location)
            if is_enemy and target not in self.enemy_targets[uav_type]:
                self.enemy_targets[uav_type].append(target)
            elif not is_enemy and target in self.enemy_targets[uav_type]:
                self.enemy_targets[uav_type].remove(target)
            self.update_enemy_panel(uav_type)

    def add_wp_from_target(self, uav_type, row, panel):
        if hasattr(self, 'target_table'):
            target = self.targets[uav_type][row]
            if target["lat"] and target["lon"]:
                target_uav = uav_type if panel == "Sol" else ("Sağ Panel" if panel == "Sağ" else uav_type)
                self.add_waypoint(target_uav, target["lat"], target["lon"], "WAYPOINT")
                self.log_message(uav_type, f"Hedef waypoint {panel} paneline eklendi: ({target['lat']:.6f}, {target['lon']:.6f})", "SUCCESS")

    def destroy_target(self, uav_type, row):
        vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
        if hasattr(self, 'target_table') and vehicle:
            target = self.targets[uav_type][row]
            if target["is_enemy"]:
                self.log_message(uav_type, f"Hedef imha edildi: {target['obj_type']}", "SUCCESS")
                target["destroyed"] = True
                self.target_table.removeRow(row)
                self.targets[uav_type].pop(row)
                self.enemy_targets[uav_type] = [t for t in self.enemy_targets[uav_type] if t["row"] != row]
                self.update_enemy_panel(uav_type)
            else:
                self.log_message(uav_type, "Sadece düşman hedefler imha edilebilir!", "WARNING")

    def update_enemy_panel(self, uav_type):
        if hasattr(self, 'yolo_view_2'):
            self.yolo_view_2.setRowCount(0)
            for index, target in enumerate(self.enemy_targets[uav_type]):
                self.yolo_view_2.insertRow(index)
                self.yolo_view_2.setItem(index, 0, QTableWidgetItem(target["obj_type"]))
                self.yolo_view_2.setItem(index, 1, QTableWidgetItem("Evet"))
                destroy_btn = QPushButton("İmha")
                destroy_btn.setEnabled(target["lat"] is not None and target["lon"] is not None)
                self.yolo_view_2.setCellWidget(index, 2, destroy_btn)

    def update_data(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        for uav_type in ["Sol Panel", "Sağ Panel"]:
            vehicle = self.vehicle_left if uav_type == "Sol Panel" else self.vehicle_right
            if not vehicle:
                continue
            telemetry = self.telemetry_data.get(uav_type, {})
            if not telemetry:
                continue
            for key, value in telemetry.items():
                if key == "Flight Mode":
                    label = self.left_flight_mode_label if uav_type == "Sol Panel" else self.right_flight_mode_label
                    color = self.mode_colors.get(value, "gray")
                    label.setStyleSheet(f"background-color: {color}; color: white; padding: 5px; border: 2px solid #800080;")
                    label.setText(f"Uçuş Modu: {value}")
                elif key == "Arm Status":
                    label = self.left_arm_status_label if uav_type == "Sol Panel" else self.right_arm_status_label
                    label.setText(f"Motor Durumu: {value}")
                    label.setStyleSheet(f"background-color: {'green' if value == 'ARM' else 'red'}; color: white; padding: 5px; border: 2px solid #800080;")
                elif key == "Battery":
                    label_key = (uav_type, "Battery")
                    if label_key in self.telemetry_labels:
                        battery_voltage = telemetry.get("Battery (V)", 0.0)
                        battery_remaining = telemetry.get("Battery (%)", None)
                        self.telemetry_labels[label_key].setText(f"{battery_voltage:.1f}V ({battery_remaining:.0f}%)")
                else:
                    label_key = (uav_type, key)
                    if label_key in self.telemetry_labels and value is not None:
                        self.telemetry_labels[label_key].setText(f"{value:.2f}" if isinstance(value, (int, float)) else str(value))
            try:
                lat = telemetry.get("Latitude", 0)
                lon = telemetry.get("Longitude", 0)
                yaw = telemetry.get("Yaw (deg)", 0)
                if lat != 0 and lon != 0 and (lat, lon) != self.last_position[uav_type]:
                    self.last_position[uav_type] = (lat, lon)
                    js_command = f"updateMarkers({lat}, {lon}, {yaw})"
                    if uav_type == "Sol Panel":
                        self.left_data_map.page().runJavaScript(js_command)
                        self.plan_maps["Sol Panel"].page().runJavaScript(js_command)
                    elif uav_type == "Sağ Panel":
                        self.right_data_map.page().runJavaScript(js_command)
                        self.plan_maps["Sağ Panel"].page().runJavaScript(js_command)
            except Exception as e:
                print("Harita hatası:", str(e))
                self.log_message(uav_type, f"Harita güncelleme hatası: {str(e)}", "ERROR")

            if current_time - self.last_telemetry_check[uav_type] >= 5:
                self.last_telemetry_check[uav_type] = current_time
                if telemetry.get("Flight Mode", "Unknown") != "Unknown":
                    pass
                else:
                    self.log_message(uav_type, "Telemetri verisi alınamadı", "WARNING")

    @pyqtSlot(str, str, str)
    def log_message_from_thread(self, uav_type, message, log_type):
        self.log_message(uav_type, message, log_type)

    def log_message(self, uav_type, message, log_type="INFO"):
        try:
            log_widget = self.left_mavlink_logs if uav_type == "Sol Panel" else self.right_mavlink_logs
            if log_widget.document().blockCount() > 200:
                log_widget.clear()
            colors = {
                "INFO": "cyan",
                "WARNING": "yellow",
                "ERROR": "red",
                "SUCCESS": "#00cc66"  # Daha açık yeşil
            }
            log_text = f"<font color='{colors[log_type]}'>[{time.strftime('%H:%M:%S')}] {message}</font>"
            cursor = QTextCursor(log_widget.document())
            cursor.movePosition(QTextCursor.End)
            log_widget.setTextCursor(cursor)
            log_widget.insertHtml(log_text + "<br>")
            log_widget.verticalScrollBar().setValue(log_widget.verticalScrollBar().maximum())
        except Exception as e:
            print(f"Loglama hatası: {str(e)}")

def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

if __name__ == "__main__":
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    gui = DualUAVGUI()
    gui.show()
    sys.exit(app.exec_())