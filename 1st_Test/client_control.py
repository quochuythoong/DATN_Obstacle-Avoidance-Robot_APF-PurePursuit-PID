###############################################################################
# LIBRARIES
###############################################################################
import requests

###############################################################################
# ESP32 SERVER IP ADDRESS
###############################################################################
esp32_ip = "http://192.168.1.9"

###############################################################################
# ROBOT VELOCITY & PID QUERY
###############################################################################

# Send w1, w2 (Setpoint for Calculated Angular velocity)
def send_params(value1, value2):
    try:
        # Include both parameters in the URL
        response = requests.get(f"{esp32_ip}/?left={value1}&right={value2}")
        if response.status_code == 200:
            print(f"Sent left={value1}, right={value2} successfully")
        else:
            print(f"Failed to send params, status code:", response.status_code)
    except requests.exceptions.RequestException as e:
        pass

# Send PID parameters
def send_PID(kp, ki, kd):
    try:
        # Include both parameters in the URL
        response = requests.get(f"{esp32_ip}/?kp={kp}&ki={ki}&kd={kd}")
        if response.status_code == 200:
            print(f"Sent kp={kp}, ki={ki}, kd={kd} successfully")
        else:
            print(f"Failed to send PID, status code:", response.status_code)
    except requests.exceptions.RequestException as e:
        pass

# Enable/Disable PID
def ena_PID(value_ON_OFF):
    try:
        # Include both parameters in the URL
        response = requests.get(f"{esp32_ip}/?ena_PID={value_ON_OFF}")
        if response.status_code == 200:
            print(f"Sent ena_PID={value_ON_OFF} successfully")
        else:
            print(f"Failed to send ena_PID, status code:", response.status_code)
    except requests.exceptions.RequestException as e:
        pass