#! /usr/bin/env python
#
# Ventilation Control
# Author: Salena Dreadon
# Copyright (c) 2015 
PROG_NAME = "Smart Home Control"
VERSION = "0.11"
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# libraries/ sources acknowledged
# Adafruit_DHT (humidity sensor) from 
# W1ThermSensor (DS18B20 temperature sensor) https://github.com/timofurrer/ds18b20
# ouimeaux (Wemo switch controller) https://github.com/iancmcc/ouimeaux/
# TFT LCD from

# TO DO:
# error handling and recovery
# gv->parameters/classes
# write data to google docs
# add wireless sensors using ESP8266

import Adafruit_DHT as dht
import Adafruit_Nokia_LCD as LCD
import Adafruit_GPIO.SPI as SPI
import Adafruit_GPIO.GPIO as GPIO
import RPi.GPIO as gpio

from w1thermsensor import W1ThermSensor
import ouimeaux
from ouimeaux.environment import Environment
from time import sleep
from time import strftime
from datetime import *
import atexit

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

#gpio = GPIO.RPiGPIOAdapter (mode = rpi_gpio.BOARD) 
DELAY = 60 #seconds delay in main loop

#TRoomID = '0008006b3137'
TRoofID = '0008006b2e67' 
TRoomID = '03150503c1ff'
RoofFanID = 'RoofFan01'  #Name of fan set up in Wemo App
HS_Pin =  26  #BCM -> Pin 37  
LED_Pin = 15  #BCM -> Pin 10  
PWM_Pin = 18  #BCM -> Pin 12  
# Raspberry Pi hardware SPI config:
DC = 23   #BCM -> Pin 16
RST = 24  #BCM -> Pin 18
SPI_PORT = 0
SPI_DEVICE = 0

def writeLogFile():
    f = open (d.strftime("/home/pi/salena/data/log%Y%B.csv"), 'a')
    # st = d.strftime("%d/%m/%Y %I:%M:%S %p") #date time AM/PM
    st = d.strftime("%d/%m/%Y %H:%M:%S,")      #date time 24 Hour
    st += "{0:0.1f},{1:0.1f},".format(trf, 0)   #roof temp, humidity
    st += "{0:0.1f},{1:0.1f},".format(trm, hrm) #room temp, humidity
    st += "True," if f else "False,"
    st += "{0:0.1f},,,".format(trm2)
    st += "\n"
    f.write (st) 
    f.close () 

def isRoomTInRange():
    return (not isRoomCold ()) and (not isRoomHot ())

def isRoomHot ():
    return trm > 22

def isRoomCold ():
    return trm < 20

def isRoomHInRange():
    return (not isRoomHumidityLow ()) and (not isRoomHumidityHigh ())

def isRoomHumidityHigh () :
    return hrm > 75

def isRoomHumidityLow ():
    return hrm < 30

def readDSTemperature (TS):
    try :
        temp = TS.get_temperature()
        if temp in [-127,85]: #both 85C and -127C have been seen as errors
            temp = -999       
    except:                   #error reading temp sensor 
        temp = -999
    finally:
        return temp

def ControlFan(fan, fs):
    print fan,  
    if fs :
        print "on"
        fan.on ()
    else :
        print  "off"
        fan.off ()


def calculateFanSetting ():
    if isRoomHot():
        print "Room Temperature is Hot"
        return (trf + 1) < trm
    elif isRoomCold():
        print "Room Temperature is Cold"
        return trf > (trm + 1) 
    else :
        print "Room Temperature is Comfy"
        return calculateFanSettingByHumidity ()   
    
def calculateFanSettingByHumidity ():
    print ">>Check Humidity"
    if isRoomHumidityHigh ():
        return hrf < hrm
    elif isRoomHumidityLow():
        #print ">>Room Humidity is Low"
        return hrf > hrm  
    else :
        #print "Room Humidity is Comfy"
        return False
    

@atexit.register
def onExit ():
    gpio.cleanup()
    RoofFan01.off ()
    #turn fans off
    print "\n\nExiting Ventilation System"


def checkAlarm ():
    if  (trm > 75 or
        trf > 75 or
        trm2 > 75):
        # high temperature
        print "WARNING HIGH TEMPERATURE"
        soundAlarm1 () 
    elif (trm == -999 or
         trf == -999 or
         trm2 == -999):
        # SENSOR FAULT
        print "WARNING SENSOR FAULT"
        soundAlarm2 () 


def soundAlarm1 ():         # siren sound
    p = gpio.PWM(PWM_Pin,2500)      #2.5kHz
    while True:
        p.start(50)                #%duty cycle
        #print input("Press <return>")
        sleep (1)
        p.ChangeFrequency(2000) 
        sleep (1)
        p.ChangeFrequency(2500) 

def soundAlarm2 ():         # beep
    p = gpio.PWM(PWM_Pin,2600)      #2.6kHz
    while True:
        p.start(50)                #%duty cycle
        #print input("Press <return>")
        sleep (0.2)
        p.stop()
        sleep (2)
           

def LedLight (ls):
    gpio.output (LED_Pin, ls) #LED on/off
    
def LCD_Display ():
    disp.clear()
    disp.display()
    image = Image.new('1', (LCD.LCDWIDTH, LCD.LCDHEIGHT))
    draw = ImageDraw.Draw(image)
    # Draw a white filled box to clear the image.
    draw.rectangle((0,0,LCD.LCDWIDTH,LCD.LCDHEIGHT), outline=255, fill=255)
    L1 = d.strftime("%d %B %Y")      #date time 24 Hour
    L2 = d.strftime("    %I:%M %p")
    L3 = "Roof: {0:0.1f}C  {1:0.1f}%".format(trf, 0)   #roof temp, humidity
    L4 = "Room: {0:0.1f}C  {1:0.1f}%".format(trm, hrm) #room temp, humidity
    L5 = "Fan: ON" if f else "Fan: OFF"
    draw.text((0,0*9), L1, font=font1)
    draw.text((0,1*9), L2, font=font1)
    draw.text((0,2*9), L3, font=font1)
    draw.text((0,3*9), L4, font=font1)    
    draw.text((0,4*9), L5, font=font1)
    # Display image.
    image = image.rotate(180)
    disp.image(image)
    disp.display()

def LCD_Display_Initial (d):
    disp.clear()
    disp.display()
    image = Image.new('1', (LCD.LCDWIDTH, LCD.LCDHEIGHT))
    draw = ImageDraw.Draw(image)
    # Draw a white filled box to clear the image.
    draw.rectangle((0,0,LCD.LCDWIDTH,LCD.LCDHEIGHT), outline=255, fill=255)
    L1 = d.strftime("%d %B %Y")      #date time 24 Hour
    L2 = d.strftime("    %I:%M %p")
    L3 = ""
    L4 = PROG_NAME
    L5 = "Version: "  + VERSION
    draw.text((0,0*9), L1, font=font1)
    draw.text((0,1*9), L2, font=font1)
    draw.text((0,2*9), L3, font=font1)
    draw.text((0,3*9), L4, font=font1)    
    draw.text((0,4*9), L5, font=font1)
    # Display image.
    image = image.rotate(180)
    disp.image(image)
    disp.display()

def LCD_Display_Setup():
    #set up for LCD display
    # Hardware SPI usage:
    disp = LCD.PCD8544(DC, RST, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=4000000))
    # Initialize library.
    disp.begin(contrast=60)
    # Clear display.
    disp.clear()
    disp.display()
    # Create blank image for drawing. # Make sure to create image with mode '1' for 1-bit color.
    image = Image.new('1', (LCD.LCDWIDTH, LCD.LCDHEIGHT))
    # Get drawing object to draw on image.
    draw = ImageDraw.Draw(image)
    font1 = ImageFont.truetype('/home/pi/.fonts/Minecraftia-Regular.ttf', 8)
    #font1 = ImageFont.truetype('/home/pi/.fonts/VCR_OSD_MONO_1.001.ttf', 9)
    #font1 = ImageFont.truetype('/home/pi/.fonts/LCD_Solid.ttf', 10)
    #font1 = ImageFont.truetype('/usr/share/fonts/truetype/freefont/FreeMono.ttf', 10)
    return disp, image, draw, font1

####### START SETUP #######
#print "\n##  Starting Ventilation System ##"
print "## ",PROG_NAME, " ##"
print "##  VERSION:",VERSION, "                  ##"
print strftime("##  %D  %H:%M:%S              ##\n")

print "\nLCD setup"
disp, image, draw, font1 = LCD_Display_Setup()
LCD_Display_Initial (datetime.today())

# setup temperature sensors
# TS = W1ThermSensor(W1ThermSensor.THERM_SENSOR_DS18B20,'0008006b3137')
TRoom = W1ThermSensor(W1ThermSensor.THERM_SENSOR_DS18B20,TRoomID) #DS1820 =old version
TRoof = W1ThermSensor(16,TRoofID)
# check temperature sensors
print "Find Temp Sensors"
for s in W1ThermSensor.get_available_sensors():
    print ">Found:",s.id, s.get_temperature(),"'C"

# setup WeMo switch
print "\nFind Wemo Switches"
wemo = Environment()
print ">start"
wemo.start()
print ">discover"
wemo.discover(5)
print ">Found: ", wemo.list_switches()
print ">get RoofFan"
RoofFan01 = wemo.get_switch(RoofFanID)

# humidty sensor
print "Check Humidity Sensor"
hrm,t = dht.read_retry(dht.DHT22, HS_Pin)
print "Temp = {0:0.1f}'C  Humidity = {1:0.1f}%\n".format(t, hrm)

# setup GPIO for buzzer and LED

#gpio.setmode(gpio.BOARD)
gpio.setup(PWM_Pin,gpio.OUT) #buzzer
gpio.setup(LED_Pin,gpio.OUT) #LED


####### MAIN LOOP #######
print "Start Ventilation Control\n" 
while True:
    d = datetime.today()
    print d.strftime("Time: %H:%M:%S")
                 
    LedLight(True) #LED on
            
    # humidity sensors
    hrf = 60 # dummy value to be replaced by sensor 
    hrm,trm2 = dht.read_retry(dht.DHT22, HS_Pin)
    print "DHT22 Temp = {0:0.1f}'C  Humidity = {1:0.1f}%".format(trm2, hrm)

    # temperature sensors
    trf = readDSTemperature(TRoof)
    trm = readDSTemperature(TRoom)
    print "Roof: {0:0.1f}'C".format(trf), 
    print " Room: {0:0.1f}'C".format(trm)

    print "Room Humidity is", "High" if isRoomHumidityHigh () else ( "Low" if isRoomHumidityLow () else "Comfy")
    #print 

    LedLight(False) #LED off 
                 
    checkAlarm ()
    
    f = calculateFanSetting()
    #print "Set Fan**-> ", f
    ControlFan(RoofFan01, f)
    
    LCD_Display ()
    
    writeLogFile()
    
    print "**wait",DELAY, "seconds**\n"
    sleep(DELAY)



