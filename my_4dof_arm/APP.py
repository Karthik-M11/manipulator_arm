import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import Int32MultiArray  # Standard ROS2 message type for arrays

class ColorSwitcher(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node  # ROS2 node reference
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Click to Change Colors')
        self.setGeometry(900, 900, 400, 600)

        self.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                         stop:0 #ffdab9, stop:1 #20b2aa);
        """)

        label1 = QLabel('TOWER A')
        label2 = QLabel('TOWER B')
        label1.setAlignment(Qt.AlignCenter)
        label2.setAlignment(Qt.AlignCenter)
        label1.setStyleSheet("font-size: 30px; font-weight: bold;")
        label2.setStyleSheet("font-size: 30px; font-weight: bold;")

        self.button_states = [1] * 10  # 10 buttons, all initially red (1)
        self.buttons = [QPushButton() for _ in range(10)]

        for idx, btn in enumerate(self.buttons):
            btn.setStyleSheet("""
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                             stop:0 #ff4d4d, stop:1 #8b0000);
                border: 2px solid black;
                border-radius: 15px ;
                padding: 15px;
                width: 300px;
                height: 80px;
            """)
            btn.setFixedSize(250, 60)
            btn.clicked.connect(lambda checked, i=idx: self.toggleColor(i))

        self.sendButton = QPushButton('Send Data')
        self.sendButton.setFixedSize(250, 60)
        self.sendButton.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                         stop:0 #ff7f50, stop:1 #6a1b9a);
            border-radius: 15px;
            color: white;
            font-size: 18px;
            font-weight: bold;
        """)
        self.sendButton.clicked.connect(self.sendData)

        gridLayout = QGridLayout()
        gridLayout.setSpacing(70)
        gridLayout.setContentsMargins(10, 10, 10, 10)
        gridLayout.addWidget(label1, 0, 0)
        gridLayout.addWidget(label2, 0, 1)
        for i in range(10):
            gridLayout.addWidget(self.buttons[i], (i // 2) + 1, i % 2)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(gridLayout)
        mainLayout.addWidget(self.sendButton, alignment=Qt.AlignCenter)
        self.setLayout(mainLayout)

    def toggleColor(self, idx):
        current_state = self.button_states[idx]
        if current_state == 1:
            self.buttons[idx].setStyleSheet("""
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                            stop:0 #000000, stop:1 #2f2f2f);
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.button_states[idx] = -1
        elif current_state == -1:
            self.buttons[idx].setStyleSheet("""
                background-color: #ffffff;
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.button_states[idx] = 0
        else:
            self.buttons[idx].setStyleSheet("""
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                         stop:0 #ff4d4d, stop:1 #8b0000);
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.button_states[idx] = 1

    def sendData(self):
        # Publish the current button states to the ROS2 topic
        msg = Int32MultiArray()
        msg.data = self.button_states
        self.ros_node.publisher.publish(msg)
        print("Published Button States:", self.button_states)

class ButtonStatePublisher(Node):
    def __init__(self):
        super().__init__('button_state_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'button_states', 10)

# def main():
#     rclpy.init()
#     ros_node = ButtonStatePublisher()
#     app = QApplication(sys.argv)
#     window = ColorSwitcher(ros_node)
    
#     # Timer to spin ROS2 in the background
#     timer = QTimer()
#     timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
#     timer.start(10)
    
#     window.show()
#     sys.exit(app.exec_())

# if __name__ == '__main__':
#     main()