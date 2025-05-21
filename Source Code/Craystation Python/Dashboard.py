# Work started: 21 Dec 2024 by Keith Rickard
# /home/keith/Documents/CrayStation
version = "\n(c) 2025 Crayford Manor House Astronomical Society Dartford - Version V0.250105"
OpSys = "Win"                            # "TEST" for test mode, "Win" for Window, "RPi" for Raspberry Pi

from guizero import *
from datetime import *
from suntime import *
from dateutil import *
from astropy.time import Time             # pip install astropy
from astropy.coordinates import *
from astropy import units as u
from PIL import Image                     # pip install pillow
#from picamera import  PiCamera            # pip install picamera
import serial                             # pip install pyserial
import time
import ephem                              # pip install ephem
import subprocess
import os
import re
import socket
import netifaces                          # pip install netifaces
from getmac import get_mac_address        # pip install getmac

arduinoPresent = True
arduino = serial.Serial()
try:
  if OpSys == "Win":
    arduino = serial.Serial("COM11", 9600, timeout=1)
  elif OpSys == "RPi":
    arduino = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
except:
    arduinoPresent = False

# Create a RAM disk for the RPi to temporarily store video stream images
if os=="RPi":
  subprocess.run(['sudo', 'mkdir', '-p', '/ramdisk'])
  with open('/etc/fstab', 'a') as f:
    f.write('tmpfs /ramdisk tmpfs defaults,size=100M 0 0\n')
  subprocess.run(['sudo', 'mount', '-a'])
  output = subprocess.run(['df', '-h'], capture_output=True, text=True)
  print(output.stdout)
  ramDiskPath = '/ramdisk/'

# Site settings
siteName = "CrayStation Alpha - Paddock Wood, Kent"
longitude = 0 + 23 / 60 + 16.066 / 3600
latitude = 51 + 10 / 60 + 31.017 / 3600
obsSite = EarthLocation(lat=latitude*u.deg, lon=longitude*u.deg)
sun = Sun(latitude, longitude)

# Other settings
path = os.path.dirname(os.path.abspath(__file__))+'/'		# Folder where picture files are kept
servoAction = 131
servoHome = 90
win = 0

# Apps to be opened from dashboard
pathSeestar = "notepad.exe"
pathSkyCam = ""
pathFileManager = ""
pathStellarium = ""

# Get Default Gateway's IP address
defaultGateway = netifaces.gateways()['default'][netifaces.AF_INET][0]
netPrefix = defaultGateway[:-1]
SeeStarIPaddress = '192.168.0.74'
pinging = False

#==================================================================================================
# Look for 'SeeStar' on the network and get its IP address:
#--------------------------------------------------------------------------------------------------
def GetSeeStarIpAddr(ipAddress, lastAddress):
  global pinging
  if pinging:
    return "Busy"
  pinging = True
  deviceNo = 1
  endNo = 255
  if ipAddress == '':
    ipAddress == netPrefix + str(deviceNo)
  else:
    deviceNo = ipAddress[-1]
    if ipAddress[-3] == '.':
      deviceNo = ipAddress[-2:]
    else:
      deviceNo = ipAddress[-3:]
  
  if lastAddress == '':
    lastAddress == netPrefix + str(endNo)
  else:
    endNo = ipAddress[-1]
    if lastAddress[-3] == '.':
      endNo = lastAddress[-2:]
    else:
      endNo = lastAddress[-3:]

  for i in range(int(deviceNo), int(endNo)+1):
    ipAddress = netPrefix+ str(i)
    try:
      if OpSys == "Win":
        ping = subprocess.check_output(['ping', '-n', '1', '-w', '3', ipAddress], universal_newlines=True)
        #ping = subprocess.check_output(['ping', '-n', '1', ipAddress], universal_newlines=True)
      else: # RPi OS
        ping = subprocess.check_output(['ping', '-c', '1', '-W', '3', ipAddress], universal_newlines=True)
        #ping = subprocess.check_output(['ping', '-n', '1', ipAddress], universal_newlines=True)
      if ping.find("unreachable") == -1:
        #   print(f"{socket.gethostbyaddr(ipAddress)} MAC: {get_mac_address(ip=ipAddress)}")
        if socket.gethostbyaddr(ipAddress)[0] == "SeeStar":
          #print("SeeStar found")
          pinging = False
          return ipAddress                                  # SeeStar address found
    except:
        endNo = endNo

  pinging = False
  return ''                                                 # Seestar not found

#==================================================================================================
# Comms with the Arduino
#--------------------------------------------------------------------------------------------------
def Comms(cmd, testTxt, ArdMon):
  global arduino, arduinoPresent

  line = datetime.now().strftime("%H:%M:%S ") + '[' + cmd + ']'
  if ArdMon == True:
    lstArduino.insert(0, line)
  else:
    lstArduino0.insert(0, line)

  if OpSys == "TEST":
    return testTxt

  if not arduinoPresent:
    arduino.close()
    try:
      if OpSys == "Win":
        arduino = serial.Serial("COM11", 115200, timeout=1)
      elif OpSys == "RPi":
        arduino = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
      arduino.reset_input_buffer()
      arduinoPresent = True
    except:
      return "No Arduino"

  try:
    arduino.reset_input_buffer()
    if cmd == '?':
      arduino.write(cmd.encode('ascii'))
      ArduinoHelp()
    else:  
      arduino.write(cmd.encode('ascii'))
      resp = arduino.readline().decode('ascii').strip()
      if resp == "Ready":
        resp = arduino.readline().decode('ascii').strip()
      if not resp:
        resp = "Timed-out"
      if resp == '?':
        resp = resp + cmd + ' - bad command'
      if resp == '%':
        resp =  resp + cmd + " - can't execute"
  except:
    resp = "No Arduino"
    arduinoPresent = False

  line += ' = ' + resp
  if ArdMon == True:
    lstArduino.remove(lstArduino.items[0])
    lstArduino.insert(0, line)
  else:
    lstArduino0.remove(lstArduino0.items[0])
    lstArduino0.insert(0, line)
  
  if len(lstArduino.items) > 600:
    for i in range(600, len(lstArduino.items)):
      lstArduino.remove(lstArduino.items[i])
  if len(lstArduino0.items) > 600:
    for i in range(600, len(lstArduino0.items)):
      lstArduino0.remove(lstArduino0.items[i])

  return resp
#--------------------------------------------------------------------------------------------------
def ArduinoHelp():
  i = 0
  resp = ""
  while resp != "END" or i > 100:
    resp = arduino.readline().decode('ascii').strip()
    lstArduino.insert(i, resp)
    i += 1
  return
#--------------------------------------------------------------------------------------------------  
def BtnArduino():
  if tbxSend.value != "":
    Comms(tbxSend.value, "", True)
  return
#--------------------------------------------------------------------------------------------------
def WinDashboard():
  global win
  win = 0
  app.show()
  HideWins()
  return
#--------------------------------------------------------------------------------------------------
def WinSuperUser():
  global win
  win = 1
  winSU.show()
  HideWins()
  return
#--------------------------------------------------------------------------------------------------
def DashboardQuit():
  global Win, arduino
  arduino.close()
  win = 2
  app.destroy()
  quit
  return
#--------------------------------------------------------------------------------------------------
def HideWins():
  if win != 0: app.hide()
  if win != 1: winSU.hide()
  return
#--------------------------------------------------------------------------------------------------
def LongTxt():
  long = longitude
  if long < 0:
    long = -long
  long += 1/7200

  txt = str(int(long)) + "\u00B0"
  txt += ('0' + str(int(long * 60) % 60))[-2:] + "'"
  txt += ('0' + str(int(long * 3600) % 60))[-2:] + '" '
  if longitude >= 0:
    txt += 'E'
  else:
    txt += 'W'
  return txt
#--------------------------------------------------------------------------------------------------
def LatTxt():
  lat = latitude
  if lat < 0:
    lat = -lat
  lat += 1/7200

  txt = str(int(lat)) + "\u00B0"
  txt += ('0' + str(int(lat * 60) % 60))[-2:] + "'"
  txt += ('0' + str(int(lat * 3600) % 60))[-2:] + '" '
  if latitude >= 0:
    txt += 'N'
  else:
    txt += 'S'
  return txt
#--------------------------------------------------------------------------------------------------
def RefreshSkyCam():
  Picture(app, image="skycam800.png", grid=[2, 1, 2, 24])
  return
#--------------------------------------------------------------------------------------------------
def Timer():
  global sun, win

  if Comms('#', '0', False) == '1':
    print("Handbox is connected")
#    if not arduinoPresent:
#      return

  if win == 0:  # Dashboard Win is active
    seeStarConnected = GetSeeStarIpAddr(SeeStarIPaddress, SeeStarIPaddress) == SeeStarIPaddress

    localNow = datetime.now()
    utcNow = datetime.now(timezone.utc)
    now = Time(utcNow)
    txtDate.clear()
    txtDate.append(localNow.strftime("%Y %b %d"))    
    txtTime.clear()
    txtTime.append(localNow.strftime("%Hh %Mm %Ss"))
    txtUTC.clear()
    txtUTC.append(utcNow.strftime("%Hh %Mm %Ss"))

    sidTime = Time.now().sidereal_time('apparent', longitude).hour
    sidHour = int(sidTime)
    sidMin = int((sidTime - sidHour) * 60)
    sidSec = int((sidTime * 3600) % 60)
    sidText = ('0' + str(sidHour))[-2:] + "h " + ('0' + str(sidMin))[-2:] + "m " + ('0' + str(sidSec))[-2:] + 's'
    txtSidTime.clear()
    txtSidTime.append(sidText)

    sunAltAzFrame =AltAz(obstime=now, location=obsSite)
    sunAltAz = get_sun(now).transform_to(sunAltAzFrame)
    sunAlt = sunAltAz.alt.deg
    txtSunrise.clear()
    txtSunrise.append(sun.get_sunrise_time().strftime("%Hh %Mm %Ss"))
    txtSunset.clear()
    txtSunset.append(sun.get_sunset_time().strftime("%Hh %Mm %Ss"))
    txtDarkness.clear()
    if sunAlt > 0:      txtDarkness.append("Daylight")
    elif sunAlt > -6:   txtDarkness.append("Civil twilight")
    elif sunAlt > -12:  txtDarkness.append("Nautical twilight")
    elif sunAlt > -18:  txtDarkness.append("Astronomical twilight")
    else:               txtDarkness.append("Astronomical")

    site = ephem.Observer()
    site.lat = str(latitude)
    site.lon = str(longitude)
    site.date = utcNow
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
    txtMoonrise.clear()
    txtMoonrise.append(ephem.localtime(moonrise).strftime("%Hh %Mm %Ss"))
    txtMoonset.clear()
    txtMoonset.append(ephem.localtime(moonset).strftime("%Hh %Mm %Ss"))
    txtMoonPhase.clear()
    txtMoonPhase.append(str(int(phase * 10) / 10) + '%')
    txtMoonType.clear()
    txtMoonType.append(moonType)
    txtMoonAge.clear()
    txtMoonAge.append(str(int(age * 10) / 10) + " days")

    txtConditions.clear()
    if Comms('r', '0', False) == '0':
      txtConditions.append("Dry")
    elif not arduinoPresent:
      txtConditions.append("No Arduino")
    else:
      txtConditions.append("Raining")
    txtTemp.clear()
    txtTemp.append(Comms('t', "11.5C", False))
    txtPressure.clear()
    txtPressure.append(Comms('b', "1028mb", False))
    txtHumidity.clear()
    txtHumidity.append(Comms('h', "86%", False))

    txtBattery.clear()
    txtBattery.append(Comms('v', "12.7V (Good)", False))
    txtSolar.clear()
    txtSolar.append(Comms('u', "28.1V", False))
    txtCurrent.clear()
    txtCurrent.append(Comms('a', "3500ma", False))

    txtScopeBatt.clear()
    txtScopePwr.clear()
    if seeStarConnected:
      txtScopeBatt.append("0%")
      txtScopePwr.append("CONNECTED")
    else:
      txtScopeBatt.append("Unknown")
      txtScopePwr.append("DISCONNECTED")
    return
#--------------------------------------------------------------------------------------------------
# Buttons
#--------------------------------------------------------------------------------------------------
def BtnShortPress():
  Comms('>' + servoAction,'>', False)     # Angle to turn servo to press scope's power button
  Comms('<' + servoHome, '<', False)      # Angle for servo's arm home position
  resp = Comms("T4000", 'T')              # Press power button for 4 seconds
  txtScopePwr.append
  txtScopePwr.clear()
  if resp == 'T':
    txtScopePwr.append("Short press...")
  else: 
    txtScopePwr.append(resp)
  return

def BtnLongPress():                            # Force scope to power off
  Comms(">131",'>')
  resp = Comms("T6000", 'T')              # Press power button for 6 seconds
  txtScopePwr.clear()
  if resp == 'T':
    txtScopePwr.append("Long press...")
  else: 
    txtScopePwr.append(resp)
  return

def Login():
  return

def ChangePassword():
  return

def BtnEncOpen():
  txtEnc.clear()
  if Comms('O', 'O') == 'O':
    txtEnc.clear()
    txtEnc.append("OPENING")
  return

def BtnEncClose():
  txtEnc.clear()
  if Comms('C', 'C') == 'C':
    txtEnc.append("CLOSING")
  else:
    txtEnc.append("COMMS ERROR")
  return

def BtnEncStop():
  txtEnc.clear()
  if Comms('X', 'C') == 'X':
    txtEnc.append("STOPPED")
  else:
    txtEnc.append("COMMS ERROR")
  return

def BtnCamHeater():
  return

def SldCamHeaterPower():
  return

def AppSeestarAlp():
  print(pathSeestar)
  subprocess.run([pathSeestar, ""])   # "" = parameters to be parsed by the app.
  return

def AppSkyCamAlp():
  subprocess.run([pathSkyCam])
  return

def AppFileManager():
  subprocess.run([pathFileManager])
  return

def AppStellarium():
  subprocess.run([pathStellarium])
  return

def Dummy():
  return

#==================================================================================================
# Window Initialisation
#--------------------------------------------------------------------------------------------------
colorBg = "midnight blue"
colorHdg = "white"
colorTxt = "yellow"
colorBtn = "cyan"
imgLogo = path+"CMHASD.png"
imgSeestar = path+"Seestar.gif"
imgAllSkyCam = path+"AllSkyCam.gif"
imgStellarium = path+"Stellarium.gif"
imgFileManager = path+"FileManager.gif"

app = App(title="CrayStation - DASHBOARD" , width=1500, height= 900, layout="grid", bg=colorBg)
#app.set_full_screen(True)
menubar = MenuBar(app, 
                  toplevel=["DASHBOARD", "Super User", "Quit"],
                  options=[
                    [["Select Dashboard", WinDashboard],
                     ["Login", Login],
                     ["Change password", ChangePassword],
                     ["Telescope:", Dummy],
                     ["       Short Press", BtnShortPress],
                     ["       Long Press", BtnLongPress],
                     ["Enclosure:", Dummy],
                     ["       Open", BtnEncOpen],
                     ["       Close", BtnEncClose],
                     ["       Stop", BtnEncStop],
                     ["Sky Cam Heater:", Dummy],
                     ["       Turn On/Off",BtnCamHeater],
                     ["       Set Heater Power",Login],
                     ["Open Application:", Dummy],
                     ["       Seestar Alp", AppSeestarAlp],
                     ["       All Sky Cam", AppSkyCamAlp],
                     ["       Stellarium", AppStellarium],
                     ["       File Manager", AppFileManager],
                     ["Cameras:", Dummy],
                     ["       View Sky", Dummy],
                     ["       View Enclosure", Dummy]],
                    [["Select", WinSuperUser]],
                    [["Close Dashboard", DashboardQuit]]
                  ])
#picLogo =       Picture(app, image=imgLogo,   grid=[0,0],     align="right")
picLogo =       Picture(app, image=imgLogo,   grid=[0,0])
hdg_App =       Text(app, siteName,           grid=[1,0,3,1], align="left", color=colorHdg, size=20)
#--------------------------------------------------------------------------------------------------
hdgConditions = Text(app, "__STATUS____________________________________________________________", grid=[0,1,4,1], align = "left", color=colorHdg)
hdgLocation =   Text(app, "Location",         grid=[0,2],   align="left", color=colorHdg, size=13)
lblLong =       Text(app, "  Longitude",      grid=[0,3],   align="left", color=colorTxt)
txtLong =       Text(app, LongTxt(),          grid=[1,3],   align="left", color=colorTxt)
lblLat =        Text(app, "  Latitude",       grid=[0,4],   align="left", color=colorTxt)
txtLong =       Text(app, LatTxt(),           grid=[1,4],   align="left", color=colorTxt)
hdgLocalTime =  Text(app, "Local Time",       grid=[0,6],   align="left", color=colorHdg, size=13)
lblDate =       Text(app, "  Date",           grid=[0,7],   align="left", color=colorTxt)
txtDate =       Text(app, "",                 grid=[1,7],   align="left", color=colorTxt)
lblTime =       Text(app, "  Time",           grid=[0,8],   align="left", color=colorTxt)
txtTime =       Text(app, "",                 grid=[1,8],   align="left", color=colorTxt)
lblUTC =        Text(app, "  UTC",            grid=[0,9],   align="left", color=colorTxt)
txtUTC =        Text(app, "",                 grid=[1,9],   align="left", color=colorTxt)
lblSidTime =    Text(app, "  Sidereal Time ", grid=[0,10],  align="left", color=colorTxt)
txtSidTime =    Text(app, "",                 grid=[1,10],  align="left", color=colorTxt)
hdgConditions = Text(app, "Weather",          grid=[0,12],  align="left", color=colorHdg, size=13)
lblConditions = Text(app, "  Conditions",     grid=[0,13],  align="left", color=colorTxt) # Get from Nano
txtConditions = Text(app, "Dry",              grid=[1,13],  align="left", color=colorTxt)
lblTemp =       Text(app, "  Temperature",    grid=[0,14],  align="left", color=colorTxt) # Get from Nano
txtTemp =       Text(app, "20.5C",            grid=[1,14],  align="left", color=colorTxt)
lblPressure =   Text(app, "  Air pressure",   grid=[0,15],  align="left", color=colorTxt) # Get from Nano
txtPressure =   Text(app, "1032mb",           grid=[1,15],  align="left", color=colorTxt)
lblHumidity =   Text(app, "  Humidity",       grid=[0,16],  align="left", color=colorTxt) # Get from Nano
txtHumidity =   Text(app, "86%",              grid=[1,16],  align="left", color=colorTxt)

hdgSun =        Text(app, "Sun",              grid=[2,2],   align="left", color=colorHdg, size=13)
lblSunrise =    Text(app, "  Next rise",      grid=[2,3],   align="left", color=colorTxt)
txtSunrise =    Text(app, "",                 grid=[3,3],   align="left", color=colorTxt)
lblSunset =     Text(app, "  Next set",       grid=[2,4],   align="left", color=colorTxt)
txtSunset =     Text(app, "",                 grid=[3,4],   align="left", color=colorTxt)
lblDarkness =   Text(app, "  Darkness",       grid=[2,5],   align="left", color=colorTxt)
txtDarkness =   Text(app, "",                 grid=[3,5],   align="left", color=colorTxt)
hdgMoon =       Text(app, "Moon",             grid=[2,6],   align="left", color=colorHdg, size=13)
lblMoonrise =   Text(app, "  Next rise",      grid=[2,7],   align="left", color=colorTxt)
txtMoonrise =   Text(app, "",                 grid=[3,7],   align="left", color=colorTxt)
lblMoonset =    Text(app, "  Next set",       grid=[2,8],   align="left", color=colorTxt)
txtMoonset =    Text(app, "",                 grid=[3,8],   align="left", color=colorTxt)
lblMoonPhase =  Text(app, "  Phase",          grid=[2,9],   align="left", color=colorTxt)
txtMoonPhase =  Text(app, "",                 grid=[3,9],   align="left", color=colorTxt)
lblMoonType =   Text(app, "  Type",           grid=[2,10],  align="left", color=colorTxt)
txtMoonType =   Text(app, "",                 grid=[3,10],  align="left", color=colorTxt)
lblMoonAge =    Text(app, "  Age",            grid=[2,11],  align="left", color=colorTxt)
txtMoonAge =    Text(app, "",                 grid=[3,11],  align="left", color=colorTxt)
hdgPower =      Text(app, "Power",            grid=[2,12],  align="left", color=colorHdg)
lblBattery =    Text(app, "  Main battery",   grid=[2,13],  align="left", color=colorTxt)
txtBattery =    Text(app, "12.7V (Good)",     grid=[3,13],  align="left", color=colorTxt)
lblSolar =      Text(app, "  Solar panel",    grid=[2,14],  align="left", color=colorTxt)
txtSolar =      Text(app, "28V",              grid=[3,14],  align="left", color=colorTxt)
lblCurrent =    Text(app, "  Current",        grid=[2,15],  align="left", color=colorTxt)
txtCurrent =    Text(app, "3.5 Amps",         grid=[3,15],  align="left", color=colorTxt)
lblScopeBatt =  Text(app, "  Telescope battery",grid=[2,16],align="left", color=colorTxt)
txtScopeBatt =  Text(app, "100%",             grid=[3,16],  align="left", color=colorTxt)
#--------------------------------------------------------------------------------------------------
hdgObsControls =Text(app, "__OBSERVATORY CONTROLS___________________________________________", grid=[0,18,4,1], align="left", color=colorHdg, size = 13)
lblVersion =    Text(app, version,                  grid=[0,25,3,1], align="left", color=colorHdg, size = 8)
picSkyCam =     Picture(app, image="skycam800.png", grid=[4,1,2,24])

tbxUser =       TitleBox(app, text="USER",                    grid=[0,19], layout="auto")
tbxUser.text_color = colorHdg
btnLogin =      PushButton(tbxUser, text="    LOGIN     ",    command=Login)
btnChgPwd =     PushButton(tbxUser, text="CHANGE\nPASSWORD",  command=ChangePassword)
btnLogin.text_color = colorBtn
btnChgPwd.text_color = colorBtn
tbxUser.align = "top"

tbxHeater =     TitleBox(app, text="SKY CAM HEATER",          grid=[1,19], layout="auto")
tbxHeater.text_color = colorHdg
btnHeater =     PushButton(tbxHeater, text="TURN ON",         command=BtnCamHeater)
gapHeater =     Text(tbxHeater, "HEATER POWER",               color=colorHdg)
sldHeater =     Slider(tbxHeater,                             command=SldCamHeaterPower)
sldHeater.text_color = colorTxt
btnHeater.text_color = colorBtn
tbxHeater.align = "top"

tbxScope =      TitleBox(app, text="TELESCOPE",               grid=[2,19])
tbxScope.text_color = colorHdg
btnShortPress = PushButton(tbxScope, text="SHORT PRESS",      command=BtnShortPress)
btnLongPress =  PushButton(tbxScope, text="LONG PRESS",       command=BtnLongPress)
txtScopePwr =   Text(tbxScope, "",                            align="left", color=colorTxt)
btnShortPress.text_color = colorBtn
btnLongPress.text_color = colorBtn
tbxScope.align = "top"

tbxEnc =        TitleBox(app, text="ENCLOSURE",               grid=[3,19], layout="grid")
tbxEnc.text_color = colorHdg
btnOpen =       PushButton(tbxEnc, text=" OPEN ",             grid=[0,0], command=BtnEncOpen)
btnClose =      PushButton(tbxEnc, text="CLOSE",              grid=[1,0], command=BtnEncClose)
btnStop =       PushButton(tbxEnc, text=" STOP ",             grid=[2,0], command=BtnEncStop)
txtEnc =        Text(tbxEnc, "CLOSED",                        grid=[0,1,1,3], align="left", color=colorTxt)
btnOpen.text_color = colorBtn
btnClose.text_color = colorBtn
btnStop.text_color = colorBtn
tbxEnc.align = "top"
#--------------------------------------------------------------------------------------------------
hdgApps =       Text(app, "__APPLICATIONS______________________________________________________", grid=[0,20,4,1], align="left", color=colorHdg, size = 13)
btnSeestarAlp = PushButton(app, image=imgSeestar,             grid=[0,21], command=AppSeestarAlp)
btnSkyCam =     PushButton(app, image=imgAllSkyCam,           grid=[1,21], command=AppSkyCamAlp)
btnStellarium = PushButton(app, image=imgStellarium,          grid=[3,21], command=AppStellarium, align="left")
btnFiling =     PushButton(app, image=imgFileManager,         grid=[2,21], command=AppFileManager, align="left")
txtSeestarAlp = Text(app, "Seestar Alp",                      grid=[0,22], color=colorTxt)
txtSkyCam =     Text(app, "All Sky Cam",                      grid=[1,22], color=colorTxt)
txtStellarium = Text(app, "Stellarium",                       grid=[3,22], color=colorTxt, align="left")
txtFileManager =Text(app, "File Manager",                     grid=[2,22], align="left", color=colorTxt)
#--------------------------------------------------------------------------------------------------
# Super User Window
winSU = Window(app, title="CrayStation Dashboard - SUPER USER", width=1500, height= 900, layout="grid", bg=colorBg)
winSU.hide()
menubar = MenuBar(winSU, 
                  toplevel=["Condtions", "SUPER USER", "Quit"],
                  options=[
                    [["Select Dashboard", WinDashboard],
                     ["Login", Login],
                     ["Change password", ChangePassword],
                     ["Telescope:", Dummy],
                     ["       Short Press", BtnShortPress],
                     ["       Long Press", BtnLongPress],
                     ["Enclosure:", Dummy],
                     ["       Open", BtnEncOpen],
                     ["       Close", BtnEncClose],
                     ["       Stop", BtnEncStop],
                     ["Sky Cam Heater:", Dummy],
                     ["       Turn On/Off",BtnCamHeater],
                     ["       Set Heater Power",Login],
                     ["Open Application:", Dummy],
                     ["       Seestar Alp", AppSeestarAlp],
                     ["       All Sky Cam", AppSkyCamAlp],
                     ["       Stellarium", AppStellarium],
                     ["       File Manager", AppFileManager],
                     ["Cameras:", Dummy],
                     ["       View Sky", Dummy],
                     ["       View Enclosure", Dummy]],
                    [["Select", WinSuperUser]],
                    [["Close Dashboard", DashboardQuit]]
                  ])
#picLogo =       Picture(winSU, image=imgLogo, grid=[2,0], align="right")
#lblHeading =    Text(winSU, text=siteName, grid=[3,0], size=20, align="left", color=colorHdg)
txtGap =        Text(winSU, " ", grid=[0,0])
lstArduino =    ListBox(winSU, grid=[2,1], width=400, height=600, scrollbar=True)
lstArduino.text_size=8
lstArduino.text_color=colorTxt
lstArduino.font = "Courier New"
tbxSend =       TextBox(winSU, grid=[2,2], width=30)
tbxSend.text_color=colorTxt
btnSend =       PushButton(winSU, command=BtnArduino, text="Send", grid=[3,2])
btnSend.text_color=colorBtn
lstArduino0 =   ListBox(winSU, grid=[4,1], width=400, height=600, scrollbar=True)
lstArduino0.text_size=8
lstArduino0.text_color=colorTxt

#winSU.set_full_screen(True)
#--------------------------------------------------------------------------------------------------
Timer()
tmrTimer = app.repeat(1000, Timer)
app.when_closed = DashboardQuit
app.display()
quit
#==================================================================================================
