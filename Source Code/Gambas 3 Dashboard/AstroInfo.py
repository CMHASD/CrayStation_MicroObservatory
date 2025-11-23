import subprocess
import os
import ephem
import socket
import pygame
import netifaces                          # pip install netifaces

#from getmac import get_mac_address        # pip install getmac
from datetime import *
from suntime import *
from dateutil import *
from astropy.time import Time             # pip install astropy
from astropy.coordinates import *
from astropy import units as u
from pywifi import *

# Default values
longitude        =  0.38794
latitude         = 51.17522
siteName         = 'CrayStation Alpha - Paddock Wood, Kent'
obsSite          = EarthLocation(lat = latitude*u.deg, lon=longitude*u.deg)
timeOffset       =  0.0
sun              = Sun(latitude, longitude)
settingsPath     = '/home/keith/Gambas Files/CrayStationDashboard/'
settingsFile     = settingsPath + 'Settings.ini'
ramDiskPath      = '/mnt/ramdisk/'
sunriseFile      = ramDiskPath + 'SunRise.txt'
sunsetFile       = ramDiskPath + 'SunSet.txt'
darknessFile     = ramDiskPath + 'Darkness.txt'
moonriseFile     = ramDiskPath + 'MoonRise.txt'
moonsetFile      = ramDiskPath + 'MoonSet.txt'
moonPhaseFile    = ramDiskPath + 'MoonPhase.txt'
moonTypeFile     = ramDiskPath + 'MoonType.txt'
moonAgeFile      = ramDiskPath + 'MoonAge.txt'
ssidFile         = ramDiskPath + 'SSID.txt'
passwordFile     = ramDiskPath + 'Password.txt'
wifiStatus       = ramDiskPath + 'WiFiStatus.txt'

# Initialisation
pygame.init()
clock = pygame.time.Clock()
# ====================================================================================================================
def Create_Ramdisk(mount_point="/mnt/ramdisk", size_mb=1):
    try:
        # Create the mount point if it doesn't exist
        if not os.path.exists(mount_point):
            subprocess.run(["sudo", "mkdir", "-p", mount_point], check=True)

        # Mount the RAM disk
        subprocess.run([
            "sudo", "mount", "-t", "tmpfs", "-o", f"size={size_mb}M", "tmpfs", mount_point
        ], check=True)

        print(f"RAM disk mounted at {mount_point} with size {size_mb}MB")
    except subprocess.CalledProcessError as e:
        print(f"Error creating RAM disk: {e}")
# --------------------------------------------------------------------------------------------------------------------
def Get_Settings_Info():
    try:
        file = open(settingsFile, 'r')
        for line in file:
            if line.startswith('Longitude'):
                longitude = line[line.find('=') + 1:-1]
            elif line.startswith('Latitude'):
                latitude = line[line.find('=') + 1:-1]
            elif line.startswith('TimeOffset'):
                timeOffset = line[line.find('=') + 1:-1]
            elif line.startswith('SiteName'):
                siteName = line[line.find('=') + 1:-1]
        file.close()
    except:
        file = open(settingsFile, 'w')
        file.write('Longitude=0.38794\n')
        file.write('Latitude=51.17522\n')
        file.write('TimeOffset=0.0\n')
        file.write('SiteName=CrayStation Alpha - Paddock Wood, Kent\n')
        file.close()
        print('Created ' + settingsFile)
# --------------------------------------------------------------------------------------------------------------------
def Get_Sun_Info():
    obsSite = EarthLocation(lat = latitude*u.deg, lon=longitude*u.deg)
    sun = Sun(latitude, longitude)
    utcNow = datetime.now(timezone.utc)
    now = Time(utcNow)
    
    sunAltAzFrame =AltAz(obstime = now, location = obsSite)
    sunAltAz = get_sun(now).transform_to(sunAltAzFrame)
    sunAlt = sunAltAz.alt.deg
    
    file = open(sunriseFile, 'w')
    file.write(sun.get_sunrise_time().strftime('%H:%M') + '\n')
    file.close()
               
    file = open(sunsetFile, 'w')               
    file.write(sun.get_sunset_time().strftime('%H:%M') + '\n')
    file.close()
               
    file = open(darknessFile, 'w')
    darkness = ""
    if sunAlt > 0:      darkness = 'Daylight'
    elif sunAlt > -6:   darkness = 'Civil twilight'
    elif sunAlt > -12:  darkness = 'Nautical twilight'
    elif sunAlt > -18:  darkness = "Astronomical twilight"
    else:               darkness = 'Astronomical'
    file.write(darkness + '\n')
# --------------------------------------------------------------------------------------------------------------------
def Get_Moon_Info():
    site = ephem.Observer()
    site.lat = str(latitude)
    site.lon = str(longitude)
    site.date = datetime.now(timezone.utc)
    moon = ephem.Moon(site)
    moonrise = site.next_rising(moon)
    moonset = site.next_setting(moon)
    phase = moon.phase
    prevPhase = ephem.Moon(site.previous_transit(ephem.Moon())).phase
    age = (site.date - ephem.previous_new_moon(site.date))
    if prevPhase > phase:
      if phase < 1:     moonType = "New moon"
      elif phase <49:   moonType = "Waning crescent"
      elif phase <= 51: moonType = "First quarter"
      elif phase <= 99: moonType = "Waning gibbous"
      elif phase > 99:  moonType = "Full moon"
    else:
      if phase < 1:     moonType = "New moon"
      elif phase <49:   moonType = "Waxing crescent"
      elif phase <= 51: moonType = "Last quarter"
      elif phase <= 99: moonType = "Waxing gibbous"
      elif phase > 99:  moonType = "Full moon"
               
    file = open(moonriseFile, 'w')
    file.write(ephem.localtime(moonrise).strftime("%H:%M") + '\n')
    file.close()
    
    file = open(moonsetFile, 'w')
    file.write(ephem.localtime(moonset).strftime("%H:%M") + '\n')
    file.close()
    
    file = open(moonPhaseFile, 'w')
    file.write(str(int(phase * 10) / 10) + '%\n')
    file.close()
    
    file = open(moonAgeFile, 'w')
    file.write(str(int(age * 10) / 10) + ' days\n')
    file.close()
    
    file = open(moonTypeFile, 'w')
    file.write(moonType + '\n')
    file.close()
# --------------------------------------------------------------------------------------------------------------------
def Get_WiFi_Networks():
    output = subprocess.check_output(['nmcli', '-t', '-f', 'SSID,SIGNAL', 'device', 'wifi', 'list'])
    lines = output.decode().split('\n')
    for line in lines:
        if line.strip():
            ssid, signal = line.split(':')
            print(f"SSID: {ssid}, Signal: {signal}")
# --------------------------------------------------------------------------------------------------------------------
def Get_Connected_WiFi():
    status = 'Not connected'
    try:
        ssid = subprocess.check_output(['iwgetid', '-r']).decode().strip()
        if ssid:
            status = 'Connected to ' + ssid
        else:
            status = 'Not connected'
    except subprocess.CalledProcessError:
        status = 'Not connected'
    
    file = open(wifiStatusFile, 'w')               
    file.write(status)
    file.close()
# --------------------------------------------------------------------------------------------------------------------
def Connect_to_WiFi():
    status = ''
    try:
        os.remove(wifiStatusFile)
    except:

    try:
        file = open(ssidFile, 'r')
        ssid = file.readline()
        ssid = ssid.strip()
        file.close()
    except:
        status = status + 'No SSID provided\n'
        return

    try:
        file = open(ssidFile, 'r')
        password = file.readline()
        password = password.strip()
        file.close()
    except:
        status = status +'No password\n'
        return
    
    if status = ""
        cmd = f'nmcli dev wifi connect "{ssid}" password "{password}"'
        try:
            subprocess.run(cmd, shell=True, check=True)
            status = 'Connected'
        except subprocess.CalledProcessError:
            status = "Unable to connect"
    
    file = open(wifiStatusFile, 'w')               
    file.write(status)
    file.close()
    
    os.remove(ssidFile)
    os.remove(passwordFile)
    return status
# ====================================================================================================================
Create_Ramdisk()
Get_WiFi_Networks()
print('WiFi connection: ', Get_Connected_WiFi())

try:
    while True:
        Get_Settings_Info()
        Get_Sun_Info()
        Get_Moon_Info()
        pygame.time.wait(1000)  #  Wait 10 seconds
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
        pygame.time.wait(1000)
except KeyboardInterrupt:
    print("\nBye!")
    quit

