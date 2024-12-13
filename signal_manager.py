import os
import requests
from urllib.parse import quote
import time
import serial
import re

# Configuration
OCTOPRINT_URL = "http://willow/api"  # Base URL for OctoPrint API
API_KEY = "_hoH4T0Ngo2fAJdjn_a3_J8pJnjD0uJ4iGgh96WrhkY"  # API key for OctoPrint
UPLOAD_FOLDER = "/home/willow-1/.octoprint/uploads"  # Folder where OctoPrint uploads files

# Track if we've already notified the STM32 about print completion
notified = False


def notify_stm32_of_completion():
    """
    Sends a signal to STM32 indicating that a print job is complete.
    This helps STM32 start the next process (e.g., print removal).
    """
    try:
        with serial.Serial("/dev/serial0", 115200, timeout=1) as stm:
            stm.write(b"PRINT:COMPLETE\n")  # Send completion signal
            print("[DEBUG] Sent PRINT:COMPLETE signal to STM32.")
    except serial.SerialException as e:
        # Handle errors related to serial communication
        print(f"[DEBUG] Serial communication error while notifying STM32: {e}")
    except Exception as e:
        # Handle other unexpected errors
        print(f"[DEBUG] Unexpected error while notifying STM32: {e}")


def listen_for_stm32_signal():
    """
    Listens for signals from the STM32 to coordinate actions.
    STM32 may send signals like 'PRINT:REMOVED' when ready for the next task.
    """
    try:
        with serial.Serial("/dev/serial0", 115200, timeout=1) as stm:
            last_message = None  # Track the last message to avoid redundant logging
            no_message_counter = 0  # Count how long there has been no response

            while True:
                if stm.in_waiting > 0:
                    # Reset the no-message counter when a message is received
                    no_message_counter = 0

                    # Read raw message from STM32
                    raw_message = stm.readline().decode().strip()
                    print(f"Raw message from STM32: {raw_message}")

                    # Remove non-alphanumeric characters for noise reduction
                    message = re.sub(r'[^A-Za-z: ]', '', raw_message)

                    # Normalize corrupted messages like 'tate: Idle' to 'State: Idle'
                    if "tate: Idle" in message or "Idle" in message:
                        message = "State: Idle"

                    # Log only new or significant messages
                    if message != last_message and message:
                        print(f"Received message from STM32: {message}")
                        last_message = message

                    # Check for variations of 'PRINT:REMOVED' indicating readiness
                    if re.search(r'(PRINT|RINT): ?REMOV(ED|E)?', message, re.IGNORECASE):
                        print("[DEBUG] STM32 sent a valid PRINT:REMOVED signal. Proceeding to file removal.")
                        return True
                else:
                    # Increment the no-message counter if no message is received
                    no_message_counter += 1
                    if no_message_counter >= 10:  # Trigger warning after a delay (adjust threshold as needed)
                        print("[DEBUG] No message received from STM32 for 5 seconds.")
                        no_message_counter = 0  # Reset counter

                # Add a small delay to avoid over-polling
                time.sleep(0.5)
    except serial.SerialException as e:
        # Handle serial port errors
        print(f"[DEBUG] Serial communication error: {e}")
    except Exception as e:
        # Handle unexpected errors in the signal listening function
        print(f"[DEBUG] Unexpected error in listen_for_stm32_signal: {e}")
    return False


def get_print_status():
    """
    Fetches the current print job status from OctoPrint.
    Returns the state and completion percentage.
    """
    headers = {"X-Api-Key": API_KEY}  # Add the API key to the request headers
    try:
        # Send GET request to OctoPrint's job API
        response = requests.get(f"{OCTOPRINT_URL}/job", headers=headers)
        if response.status_code == 200:
            # Parse and return the response JSON
            data = response.json()
            return data.get("state", ""), data.get("progress", {}).get("completion", 0)
        else:
            # Log if there was an error with the request
            print(f"[DEBUG] Error fetching print status: {response.status_code} - {response.text}")
            return "", 0
    except requests.RequestException as e:
        # Handle HTTP request errors
        print(f"[DEBUG] HTTP Request failed: {e}")
        return "", 0


def monitor_print_completion():
    """
    Monitors the OctoPrint API for print completion.
    Notifies the STM32 and coordinates subsequent actions.
    """
    global notified  # Track if STM32 has been notified about print completion

    while True:
        # Fetch the current print job state and completion percentage
        state, completion = get_print_status()
        print(f"[DEBUG] Printer state: {state}, Completion: {completion}%")

        if state == "Operational" and completion == 100.0:
            # Check if print job is complete
            if not notified:
                print("[DEBUG] Print job completed. Notifying STM32.")
                notify_stm32_of_completion()  # Notify STM32 about completion
                notified = True

                if listen_for_stm32_signal():  # Wait for STM32 readiness
                    print("[DEBUG] STM32 signal received. Proceeding to next steps.")
            else:
                print("[DEBUG] Already notified STM32 about the completed print.")
        elif state == "Printing":
            # Log if the printer is still printing
            print("[DEBUG] Printer is currently printing.")
        else:
            # Handle other states and reset notification if needed
            print(f"[DEBUG] Printer state: {state}, completion: {completion}%")
            notified = False

        # Add delay to avoid overloading the OctoPrint API
        time.sleep(5)


if __name__ == "__main__":
    """
    Entry point for the script.
    Monitors print jobs and coordinates actions with STM32.
    """
    try:
        monitor_print_completion()  # Start monitoring print jobs
    except KeyboardInterrupt:
        # Gracefully handle script termination
        print("[DEBUG] Script terminated by user.")
    except Exception as e:
        # Log unexpected errors
        print(f"[DEBUG] Unexpected error occurred: {e}")

