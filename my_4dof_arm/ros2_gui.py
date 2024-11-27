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

        # Step 1: Define initial positions of discs (Pre-coded)
        self.color_code = {'red': 1, 'black': -1, 'white': 0}
        self.initial_positions = {
            1: [(1, self.color_code['red']), (2, self.color_code['black'])],  # Stack 1
            2: [(7, self.color_code['red']), (8, self.color_code['black'])]   # Stack 2
        }

        self.scenario_map = {  # Mapping between position and scenario
            1: 2,  # Scenario 2 when picking from 1
            2: 1,  # Scenario 1 when picking from 2
        }

    def initUI(self):
        self.setWindowTitle('Set Final Configuration')
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

        self.final_button_states = [0] * 10  # 10 buttons, user sets final arrangement (0: white initially)
        self.buttons = [QPushButton() for _ in range(10)]

        for idx, btn in enumerate(self.buttons):
            btn.setStyleSheet("""
                background-color: #ffffff;
                border: 2px solid black;
                border-radius: 15px ;
                padding: 15px;
                width: 300px;
                height: 80px;
            """)
            btn.setFixedSize(250, 60)
            btn.clicked.connect(lambda checked, i=idx: self.toggleColor(i))

        self.sendButton = QPushButton('Move Discs')
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
        current_state = self.final_button_states[idx]
        if current_state == 0:  # White to Red
            self.buttons[idx].setStyleSheet("""
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                         stop:0 #ff4d4d, stop:1 #8b0000);
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.final_button_states[idx] = 1
        elif current_state == 1:  # Red to Black
            self.buttons[idx].setStyleSheet("""
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                            stop:0 #000000, stop:1 #2f2f2f);
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.final_button_states[idx] = -1
        else:  # Black to White
            self.buttons[idx].setStyleSheet("""
                background-color: #ffffff;
                border: 2px solid black;
                border-radius: 15px;
                width: 300px;
                height: 80px;
            """)
            self.final_button_states[idx] = 0

    def sendData(self):
        # Create the final positions based on GUI button states
        final_positions = {
            1: [(i+1, self.final_button_states[i]) for i in range(5)],  # Stack 1 (positions 1 to 5)
            2: [(i+6, self.final_button_states[i+5]) for i in range(5)]  # Stack 2 (positions 6 to 10)
        }

        # Create the sequence of scenarios to execute
        scenario_sequence = []

        # Determine the sequence based on the initial and final positions
        for stack, discs in final_positions.items():
            for target_pos, target_color in discs:
                # Get the scenario based on the stack position
                scenario = self.scenario_map.get(target_pos, None)
                if scenario:
                    scenario_sequence.append(scenario)

        # Create Int32MultiArray and publish the sequence
        msg = Int32MultiArray(data=scenario_sequence)
        self.ros_node.publisher.publish(msg)
        print(f"Publishing scenario sequence: {scenario_sequence}")

    def simulateDiscMovement(self, initial_positions, final_positions):
        # Move discs to temporary positions
        print("Moving discs to temporary positions...")
        temp_positions = self.move_discs_to_temp(initial_positions, 6)  # Temp stack starts at position 6

        # Sort discs to final positions
        print("\nSorting discs to final positions...")
        self.sort_discs_to_final(temp_positions, final_positions)

    def move_discs_to_temp(self, initial, temp_stack_start):
        temp_positions = initial[2][:]  # Current positions in temporary "stack"
        for stack, discs in initial.items():
            if stack == 1:  # Only move discs from the "first stack"
                for pos, color in discs:
                    # Find first unoccupied position in temp stack (6 to 10)
                    unoccupied_positions = [p for p in range(temp_stack_start, 11) if p not in [d[0] for d in temp_positions]]
                    if unoccupied_positions:
                        target_pos = unoccupied_positions[0]  # Get the first available position
                        self.pick_disc(pos)
                        self.place_disc(target_pos)
                        temp_positions.append((target_pos, color))  # Add to temp list
        return temp_positions

    def sort_discs_to_final(self, temp_positions, final):
        for stack, discs in final.items():
            for target_pos, target_color in discs:
                for i, (temp_pos, temp_color) in enumerate(temp_positions):
                    if temp_color == target_color:  # Match color to final target position
                        self.pick_disc(temp_pos)  # Pick from temporary stack
                        self.place_disc(target_pos)  # Place in the final position
                        temp_positions.pop(i)  # Remove from temp list
                        break

    def pick_disc(self, position):
        print(f"Picking disc from Position {position}")

    def place_disc(self, position):
        print(f"Placing disc in Position {position}")

class ButtonStatePublisher(Node):
    def __init__(self):
        super().__init__('button_state_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'button_states', 10)

def main():
    rclpy.init()
    ros_node = ButtonStatePublisher()
    app = QApplication(sys.argv)
    window = ColorSwitcher(ros_node)
    
    # Timer to spin ROS2 in the background
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)
    
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
