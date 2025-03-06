import requests

# ESP32 server IP address
esp32_ip = "http://192.168.1.6"

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
