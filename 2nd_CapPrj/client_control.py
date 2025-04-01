import requests

# ESP32 server IP address
esp32_ip = "http://192.168.37.89"

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

# def send_params(value1, value2, kp, ki, kd):
#     try:
#         # Include both parameters in the URL
#         response = requests.get(f"{esp32_ip}/?left={value1}&right={value2}&kp={kp}&ki={ki}&kd={kd}")
#         if response.status_code == 200:
#             print(f"Sent left={value1}, right={value2} successfully, kp={kp}, ki={ki}, kd={kd}")
#         else:
#             print(f"Failed to send params, status code:", response.status_code)
#     except requests.exceptions.RequestException as e:
#         pass

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