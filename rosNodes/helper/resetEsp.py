import serial
import time

# Ask the user for the USB number
usb_number = input("Enter USB number (e.g., 0 for /dev/ttyUSB0): ").strip()
esp_port = f"/dev/ttyUSB{usb_number}"  # Construct the port dynamically

baud_rate = 115200

try:
    ser = serial.Serial(esp_port, baud_rate, timeout=1)
    print(f"Connected to {esp_port}")

    # Reset ESP32 using RTS & DTR in different order
    ser.setDTR(True)   # Pull DTR high
    ser.setRTS(False)  # Pull RTS low
    time.sleep(0.1)
    ser.setDTR(False)  # Release DTR
    ser.setRTS(True)   # Release RTS

    print("ESP32 Reset Command Sent!")

    # Wait and read ESP32 response
    time.sleep(2)

    ser.close()

except serial.SerialException as e:
    print(f"Error: {e}")
