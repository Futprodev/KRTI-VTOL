import RPi.GPIO as GPIO
import time

relay_pin = 11  # Physical pin 11 is same as 17 BCM

GPIO.setmode(GPIO.BOARD)  # Using BOARD pin numbering
GPIO.setup(relay_pin, GPIO.OUT)

GPIO.output(relay_pin, GPIO.LOW)  # Initialize relay as off

try:
    print("Press 'q' to turn the relay ON/OFF. Press 'Ctrl + C' to stop the script.")
    while True:
        command = input("Enter 'q' to toggle relay: ").strip().lower()
        if command == 'q':
            current_state = GPIO.input(relay_pin)
            GPIO.output(relay_pin, not current_state)
            print(f"Relay is now {'ON' if not current_state else 'OFF'}")
        else:
            print("Invalid input. Please enter 'q' to toggle relay.")

except KeyboardInterrupt:
    print("Script stopped by user.")

finally:
    GPIO.cleanup()  # Clean up GPIO settings
    print("GPIO cleaned up and script ended.")