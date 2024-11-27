import serial
import time

# Establish serial communication with ESP32
esp32 = serial.Serial('COM3', 115200, timeout=1)  

# List of predefined angle sets
scenarios = {
    1: [30, 45, 90, 100],   # Scenario 1
    2: [60, 30, 120, 200],  # Scenario 2
    3: [90, 90, 0, 0],      # Scenario 3
    4: [45, 10, 180, 50],   # Scenario 4
    5: [0, 0, 0, 0],        # Scenario 5
    6: [15, 60, 45, 150],   # Scenario 6
    7: [15, 60, 135, 150],  # Scenario 7
    8: [50, 50, 50, 100],   # Scenario 8
    9: [90, 20, 70, 250],   # Scenario 9
    10: [120, 80, 160, 300], # Scenario 10
    11: [120, 80, 160, 300], # Scenario 11
    12: [120, 80, 160, 80], # Scenario 12
}

pick = 1

# Function to send angles to ESP32
def send_angles(scenario):
    if scenario in scenarios:
        angles = scenarios[scenario]
        data = ','.join(map(str, angles)) + str(pick) + '\n'
        if pick == 1:
            pick = 0
        else:
            pick = 1
        esp32.write(data.encode())  # Send to ESP32
        time.sleep(0.1)  # Allow ESP32 to process
    else:
        print("Invalid scenario. Choose between 1 and 10.")

send_angles(3)
