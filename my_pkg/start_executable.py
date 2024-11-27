import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QWidget
import subprocess

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control GUI")
        self.processes = {}

        # Layout
        self.layout = QVBoxLayout()

        # Buttons
        self.segment_button = QPushButton("Start Segment Human")
        self.segment_button.clicked.connect(self.run_segment_human)
        self.layout.addWidget(self.segment_button)

        self.cmd_robot_button = QPushButton("Start Command Robot")
        self.cmd_robot_button.clicked.connect(self.run_cmd_robot)
        self.layout.addWidget(self.cmd_robot_button)

        self.measure_button = QPushButton("Start Measure Distance")
        self.measure_button.clicked.connect(self.run_measure_distance)
        self.layout.addWidget(self.measure_button)

        self.stop_button = QPushButton("Stop All")
        self.stop_button.clicked.connect(self.stop_all_processes)
        self.layout.addWidget(self.stop_button)

        # Container
        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

    def run_segment_human(self):
        self.start_process("segment_human", "python3 person_segmentation.py")

    def run_cmd_robot(self):
        self.start_process("cmd_robot", "python3 cmd_robot.py")

    def run_measure_distance(self):
        self.start_process("measure_distance", "python3 measure_distance.py")

    def start_process(self, name, command):
        if name in self.processes:
            print(f"{name} is already running.")
            return

        process = subprocess.Popen(command, shell=True)
        self.processes[name] = process
        print(f"Started {name} with PID {process.pid}")

    def stop_all_processes(self):
        for name, process in self.processes.items():
            process.terminate()
            print(f"Terminated {name} with PID {process.pid}")
        self.processes.clear()

def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
