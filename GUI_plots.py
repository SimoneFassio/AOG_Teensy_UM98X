import sys
import socket
import threading
from collections import deque
from PyQt5 import QtWidgets, QtCore, QtNetwork, QtGui
from PyQt5.QtCore import pyqtSignal, QObject, QSettings
from PyQt5.QtNetwork import QUdpSocket
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.animation as animation
import select

class UdpWorker(QObject):
    new_plot_data = pyqtSignal(str)
    new_debug_data = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def setup_sockets(self):
        self.plot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.plot_sock.bind(('0.0.0.0', 6968))
        self.plot_sock.settimeout(0.001)
        
        self.debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.debug_sock.bind(('0.0.0.0', 6969))
        self.debug_sock.settimeout(0.001)
        
    def run(self):
        self.setup_sockets()
        socks = [self.plot_sock, self.debug_sock]
        
        while self.running:
            # Use select to wait for data on any socket (with 1ms timeout)
            readable, _, _ = select.select(socks, [], [], 0.001)
            
            for sock in readable:
                try:
                    data, _ = sock.recvfrom(1024)
                    if data:
                        if sock is self.plot_sock:
                            self.new_plot_data.emit(data.decode())
                        elif sock is self.debug_sock:
                            self.new_debug_data.emit(data.decode())
                except Exception as e:
                    print(f"Socket error: {e}")

class RealTimePlot(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        
        self.udp_sender = QtNetwork.QUdpSocket()
        self.teensy_ip = '192.168.5.255'
        self.command_port = 9696
        
        self.initUI()
        self.setup_threading()
        self.data = {
            'steerAngleSens': deque(maxlen=200),
            'insWheelAngle': deque(maxlen=200),
            'keyaEncoder': deque(maxlen=200),
            'KalmanWheelAngle': deque(maxlen=200),
            'timestamps': deque(maxlen=200),
            'angleVariance': 0.0
        }
        self.visibility = {
            'steerAngleSens': True,
            'insWheelAngle': True,
            'keyaEncoder': True,
            'KalmanWheelAngle': True
        }
        self.request_states = {
            'GPS': False,
            'EXPERIMENT': False,
            'KEYA': False,
            'WAS': False,
            'INFO': False
        }
        self.config = {
            'using2serialGPS': True,
            'usingWT61': False,
            'interval_INS': 0.1,
            'useKalmanForSensor': True,
            'minSpeedKalman_m/s': 0.5,
            'secondsVarBuf': 3.0,
            'KalmanR': 0.3,
            'KalmanQ': 0.0001
        }
        self.settings = QSettings("AGOPENGPS_", "AOG_Teensy_Config")
        # Only load settings at startup; don't send them via UDP automatically.
        self.load_settings()
  

    def initUI(self):
        self.setWindowTitle('Real-Time Steering Data Monitor')
        self.setGeometry(100, 100, 1200, 800)

        self.fig = plt.Figure()
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        
        self.debug_terminal = QtWidgets.QTextEdit()
        self.debug_terminal.setReadOnly(True)
        
        # Enable touch screen scrolling features
        self.debug_terminal.viewport().setAttribute(QtCore.Qt.WA_AcceptTouchEvents, True)
        # Enable kinetic/momentum scrolling
        self.debug_terminal.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        # Make it easier to grab and scroll with fingers
        scrollbar = self.debug_terminal.verticalScrollBar()
        scrollbar.setStyleSheet("""
            QScrollBar:vertical {
                width: 18px;
            }
        """)
        
        # Auto-scroll toggle for debug terminal
        self.autoscroll_checkbox = QtWidgets.QCheckBox("Auto-scroll debug")
        self.autoscroll_checkbox.setChecked(True)
        
        # Visible Signals group
        self.checkbox_group = QtWidgets.QGroupBox("Visible Signals")
        self.steer_check = QtWidgets.QCheckBox("Steer Angle Sensor")
        self.ins_check = QtWidgets.QCheckBox("INS Wheel Angle")
        self.kalman_check = QtWidgets.QCheckBox("Kalman Wheel Angle")
        
        self.steer_check.setChecked(True)
        self.ins_check.setChecked(True)
        self.kalman_check.setChecked(True)
        
        self.steer_check.stateChanged.connect(lambda: self.toggle_visibility('steerAngleSens'))
        self.ins_check.stateChanged.connect(lambda: self.toggle_visibility('insWheelAngle'))
        self.kalman_check.stateChanged.connect(lambda: self.toggle_visibility('KalmanWheelAngle'))
        
        checkbox_layout = QtWidgets.QVBoxLayout()
        checkbox_layout.addWidget(self.steer_check)
        checkbox_layout.addWidget(self.ins_check)
        checkbox_layout.addWidget(self.kalman_check)
        self.checkbox_group.setLayout(checkbox_layout)
        
        # Requested Info group
        self.request_group = QtWidgets.QGroupBox("Requested Info")
        self.gps_check = QtWidgets.QCheckBox("GPS")
        self.experiment_check = QtWidgets.QCheckBox("EXPERIMENT")
        self.was_check = QtWidgets.QCheckBox("WAS")
        self.info_check = QtWidgets.QCheckBox("INFO")

        self.gps_check.stateChanged.connect(lambda: self.send_request('GPS', self.gps_check.isChecked()))
        self.experiment_check.stateChanged.connect(lambda: self.send_request('EXPERIMENT', self.experiment_check.isChecked()))
        self.was_check.stateChanged.connect(lambda: self.send_request('WAS', self.was_check.isChecked()))
        self.info_check.stateChanged.connect(lambda: self.send_request('INFO', self.info_check.isChecked()))
        
        # Configuration group
        self.config_group = QtWidgets.QGroupBox("Configuration Parameters")
        config_layout = QtWidgets.QGridLayout()

        self.using2serialGPS_check = QtWidgets.QCheckBox("Using 2 Serial GPS")
        self.usingWT61_check = QtWidgets.QCheckBox("Using WT61")
        self.interval_INS_spin = QtWidgets.QDoubleSpinBox()
        self.useKalmanForSensor_check = QtWidgets.QCheckBox("Use Kalman for Sensor")
        self.minSpeedKalman_spin = QtWidgets.QDoubleSpinBox()
        self.secondsVarBuf_spin = QtWidgets.QDoubleSpinBox()
        self.kalmanR_spin = QtWidgets.QDoubleSpinBox()
        self.kalmanQ_spin = QtWidgets.QDoubleSpinBox()

        self.interval_INS_spin.setRange(0.05, 0.1)
        self.interval_INS_spin.setDecimals(2)
        self.interval_INS_spin.setSingleStep(0.05)
        self.minSpeedKalman_spin.setRange(0.0, 2.0)
        self.minSpeedKalman_spin.setDecimals(1)
        self.minSpeedKalman_spin.setSingleStep(0.1)
        self.secondsVarBuf_spin.setRange(1.0, 10.0)
        self.secondsVarBuf_spin.setDecimals(1)
        self.secondsVarBuf_spin.setSingleStep(0.1)
        self.kalmanR_spin.setRange(0.001, 10.0)
        self.kalmanR_spin.setDecimals(3)
        self.kalmanR_spin.setSingleStep(0.1)
        self.kalmanQ_spin.setRange(0.0001, 0.01)
        self.kalmanQ_spin.setDecimals(6)
        self.kalmanQ_spin.setSingleStep(0.0001)

        config_layout.addWidget(QtWidgets.QLabel("Serial GPS:"), 0, 0)
        config_layout.addWidget(self.using2serialGPS_check, 0, 1)
        config_layout.addWidget(QtWidgets.QLabel("WT61:"), 1, 0)
        config_layout.addWidget(self.usingWT61_check, 1, 1)
        config_layout.addWidget(QtWidgets.QLabel("INS Interval (s):"), 2, 0)
        config_layout.addWidget(self.interval_INS_spin, 2, 1)
        config_layout.addWidget(QtWidgets.QLabel("Use Kalman:"), 3, 0)
        config_layout.addWidget(self.useKalmanForSensor_check, 3, 1)
        config_layout.addWidget(QtWidgets.QLabel("Min Kalman (m/s):"), 4, 0)
        config_layout.addWidget(self.minSpeedKalman_spin, 4, 1)
        config_layout.addWidget(QtWidgets.QLabel("Var Buffer (s):"), 5, 0)
        config_layout.addWidget(self.secondsVarBuf_spin, 5, 1)
        config_layout.addWidget(QtWidgets.QLabel("Kalman R:"), 6, 0)
        config_layout.addWidget(self.kalmanR_spin, 6, 1)
        config_layout.addWidget(QtWidgets.QLabel("Kalman Q:"), 7, 0)
        config_layout.addWidget(self.kalmanQ_spin, 7, 1)
        
        # Button to load saved settings and send them via UDP
        self.reload_settings_btn = QtWidgets.QPushButton("Send Saved Settings")
        self.reload_settings_btn.clicked.connect(self.load_and_send_settings)
        config_layout.addWidget(self.reload_settings_btn, 8, 0, 1, 2)

        self.config_group.setLayout(config_layout)

        # Connect configuration widget changes to sending updates (will be active after load)
        self.using2serialGPS_check.stateChanged.connect(
            lambda: self.send_config('using2serialGPS', self.using2serialGPS_check.isChecked()))
        self.usingWT61_check.stateChanged.connect(
            lambda: self.send_config('usingWT61', self.usingWT61_check.isChecked()))
        self.interval_INS_spin.valueChanged.connect(
            lambda: self.send_config('interval_INS', self.interval_INS_spin.value()))
        self.useKalmanForSensor_check.stateChanged.connect(
            lambda: self.send_config('useKalmanForSensor', self.useKalmanForSensor_check.isChecked()))
        self.minSpeedKalman_spin.valueChanged.connect(
            lambda: self.send_config('minSpeedKalman_m/s', self.minSpeedKalman_spin.value()))
        self.secondsVarBuf_spin.valueChanged.connect(
            lambda: self.send_config('secondsVarBuf', self.secondsVarBuf_spin.value()))
        self.kalmanR_spin.valueChanged.connect(
            lambda: self.send_config('KalmanR', self.kalmanR_spin.value()))
        self.kalmanQ_spin.valueChanged.connect(
            lambda: self.send_config('KalmanQ', self.kalmanQ_spin.value()))

        request_layout = QtWidgets.QVBoxLayout()
        request_layout.addWidget(self.gps_check)
        request_layout.addWidget(self.experiment_check)
        request_layout.addWidget(self.was_check)
        request_layout.addWidget(self.info_check)
        self.request_group.setLayout(request_layout)

        main_layout = QtWidgets.QVBoxLayout()
        top_layout = QtWidgets.QHBoxLayout()
        left_top = QtWidgets.QVBoxLayout()
        left_top.addWidget(self.canvas)
        
        right_top = QtWidgets.QVBoxLayout()
        right_top.addWidget(self.checkbox_group)
        right_top.addWidget(self.request_group)
        right_top.addWidget(self.config_group)
        right_top.addStretch()
        
        top_layout.addLayout(left_top, 90)
        top_layout.addLayout(right_top, 10)
        
        bottom_layout = QtWidgets.QVBoxLayout()
        bottom_layout.addWidget(self.debug_terminal)
        bottom_layout.addWidget(self.autoscroll_checkbox)

        main_layout.addLayout(top_layout, 60)
        main_layout.addLayout(bottom_layout, 40)
        self.setLayout(main_layout)

    def toggle_visibility(self, signal_name):
        self.visibility[signal_name] = not self.visibility[signal_name]
        self.update_plot(None)

    def setup_threading(self):
        self.worker = UdpWorker()
        self.thread = QtCore.QThread()
        self.worker.moveToThread(self.thread)
        self.worker.new_plot_data.connect(self.handle_plot_data)
        self.worker.new_debug_data.connect(self.handle_debug_data)
        self.thread.started.connect(self.worker.run)
        self.thread.start()
        
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, cache_frame_data=False)

    @QtCore.pyqtSlot(str)
    def handle_plot_data(self, data):
        values = data.split(',')
        print(values)
        if len(values) == 6:
            try:
                self.data['timestamps'].append(float(values[0]))
                self.data['steerAngleSens'].append(float(values[1]))
                self.data['insWheelAngle'].append(float(values[2]))
                self.data['keyaEncoder'].append(float(values[3]))
                self.data['KalmanWheelAngle'].append(float(values[4]))
                self.data['angleVariance'] = float(values[5])
            except ValueError:
                print("parsing failed")

    @QtCore.pyqtSlot(str)
    def handle_debug_data(self, message):
        # Store current scrollbar position and check if we were at the bottom
        scrollbar = self.debug_terminal.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10 or self.autoscroll_checkbox.isChecked()
        
        # Insert the text
        cursor = self.debug_terminal.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        cursor.insertText(message)
        self.debug_terminal.setTextCursor(cursor)
        
        # Print to console
        print(message, end="")
        
        # Force UI update before scrolling
        QtWidgets.QApplication.processEvents()
        
        # Scroll to bottom if we were at bottom or auto-scroll is checked
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def update_plot(self, frame):
        self.ax.clear()
        if self.data['timestamps']:
            t = [(ts - self.data['timestamps'][0]) / 1000 for ts in self.data['timestamps']]
            
            if self.visibility['steerAngleSens']:
                self.ax.plot(t, self.data['steerAngleSens'], label='Steer Angle Sensor')
            if self.visibility['insWheelAngle']:
                self.ax.plot(t, self.data['insWheelAngle'], label='INS Wheel Angle')
            if self.visibility['KalmanWheelAngle']:
                self.ax.plot(t, self.data['KalmanWheelAngle'], label='Kalman Wheel Angle')
            
            self.ax.text(0.02, 0.95, f"Variance: {self.data['angleVariance']:.5f}",
                         transform=self.ax.transAxes, fontsize=10,
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))
            
            self.ax.legend()
            self.ax.set_xlabel('Time (s)')
            self.ax.set_ylabel('Angle (degrees)')
            self.ax.set_title('Real-Time Steering Angles')
        self.canvas.draw()
        
    def load_settings(self):
        # Load saved values or defaults
        self.config['using2serialGPS'] = self.settings.value("using2serialGPS", True, bool)
        self.config['usingWT61'] = self.settings.value("usingWT61", False, bool)
        self.config['interval_INS'] = self.settings.value("interval_INS", 0.1, float)
        self.config['useKalmanForSensor'] = self.settings.value("useKalmanForSensor", True, bool)
        self.config['minSpeedKalman_m/s'] = self.settings.value("minSpeedKalman_m/s", 0.5, float)
        self.config['secondsVarBuf'] = self.settings.value("secondsVarBuf", 3.0, float)
        self.config['KalmanR'] = self.settings.value("KalmanR", 0.3, float)
        self.config['KalmanQ'] = self.settings.value("KalmanQ", 0.0001, float)

        # Block signals so no UDP message is sent on startup.
        self.using2serialGPS_check.blockSignals(True)
        self.using2serialGPS_check.setChecked(self.config['using2serialGPS'])
        self.using2serialGPS_check.blockSignals(False)
        
        self.usingWT61_check.blockSignals(True)
        self.usingWT61_check.setChecked(self.config['usingWT61'])
        self.usingWT61_check.blockSignals(False)
        
        self.interval_INS_spin.blockSignals(True)
        self.interval_INS_spin.setValue(self.config['interval_INS'])
        self.interval_INS_spin.blockSignals(False)
        
        self.useKalmanForSensor_check.blockSignals(True)
        self.useKalmanForSensor_check.setChecked(self.config['useKalmanForSensor'])
        self.useKalmanForSensor_check.blockSignals(False)
        
        self.minSpeedKalman_spin.blockSignals(True)
        self.minSpeedKalman_spin.setValue(self.config['minSpeedKalman_m/s'])
        self.minSpeedKalman_spin.blockSignals(False)
        
        self.secondsVarBuf_spin.blockSignals(True)
        self.secondsVarBuf_spin.setValue(self.config['secondsVarBuf'])
        self.secondsVarBuf_spin.blockSignals(False)
        
        self.kalmanR_spin.blockSignals(True)
        self.kalmanR_spin.setValue(self.config['KalmanR'])
        self.kalmanR_spin.blockSignals(False)
        
        self.kalmanQ_spin.blockSignals(True)
        self.kalmanQ_spin.setValue(self.config['KalmanQ'])
        self.kalmanQ_spin.blockSignals(False)

    def send_all_configs(self):
        for key, value in self.config.items():
            self.send_config(key, value)

    def load_and_send_settings(self):
        self.load_settings()
        self.send_all_configs()
    
    def send_config(self, param, value):
        self.settings.setValue(param, value)
        # Convert boolean to integer if necessary
        if value is True:
            value = 1
        elif value is False:
            value = 0
        message = f"{param}:{value}"
        self.udp_sender.writeDatagram(message.encode(), 
                                      QtNetwork.QHostAddress(self.teensy_ip), 
                                      self.command_port)
    
    def send_request(self, command, state):
        self.request_states[command] = state
        message = f"{command}:{1 if state else 0}"
        self.udp_sender.writeDatagram(message.encode(), 
                                      QtNetwork.QHostAddress(self.teensy_ip), 
                                      self.command_port)

    def closeEvent(self, event):
        self.worker.running = False
        self.thread.quit()
        self.thread.wait()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    ex = RealTimePlot()
    ex.show()
    sys.exit(app.exec_())
