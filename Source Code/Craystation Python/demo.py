import requests
from bs4 import BeautifulSoup

# Replace these with your router's IP, username, and password
router_ip = '192.168.0.1'
username = 'admin'
password = 'your_password'

# Log in to the router
login_url = f'http://{router_ip}/login.cgi'
payload = {
    'username': username,
    'password': password
}

session = requests.Session()
login_response = session.post(login_url, data=payload)

# Check if login was successful
if login_response.status_code == 200:
    # Replace this URL with the correct endpoint for your router
    device_list_url = f'http://{router_ip}/connected_devices.cgi'
    device_list_response = session.get(device_list_url)
    
    # Parse the response (example with BeautifulSoup)
    soup = BeautifulSoup(device_list_response.text, 'html.parser')
    devices = soup.find_all('device')
    
    for device in devices:
        print(f"Device: {device.get_text()}")
else:
    print("Login failed")
