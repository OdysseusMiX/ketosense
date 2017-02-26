# Ketose detector by Jens Clarholm (jenslabs.com)
# 
# Modified for use with Raspberry Pi 2 by Mike Heinz

""" 
Components Used:
- Raspberry Pi 2
- TGS 822 Gas Sensor
- DHT11 Tempature/Humidity sensor (integrated on PCB)
- I2C 1602 LCD 16x120 Display (4-pin I2C interface)
- MCP3204-C  12-bit Analog-to-Digital Converter
- Pushbuttons (x3)

Code Overview:
  The original project used an Arduino board and analog interfaces. The project has been modified to support a Raspberry Pi 2 board and digital interfaces.
  The analog gas sensor is connected to the RPi2 via the 12-bit ADC. The DHT11 sensor and LCD display are both integrated with custom PCBs to use digital protocols directly.
  As acetone, or similar gas, is detected, the resistence of the gas sensor is reduced. The resistence of the gas sensor is also a function of warmup time allowed, tempature, humidity, and precense of other volatile gases.
  The code turns on the gas sensor heater, detects when the sensor is ready, prompts the user for a measurement, and estimates the concentration of acetone from the resistence measured across the sensor.
"""

# ---------- Setup and Initialization -----------------

# Import Libraries
import mcp3204 as MCP
import smbus
import time
import dht11
import RPi.GPIO as GPIO
import math
#//////////////////

# GPIO Pin Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
# Pushbuttons
resetMaxSwitchPin = 5
resetSensorSwitchPin = 6
toggleModeSwitchPin = 13
GPIO.setup(resetMaxSwitchPin,GPIO.IN)
GPIO.setup(resetSensorSwitchPin,GPIO.IN)
GPIO.setup(toggleModeSwitchPin,GPIO.IN)

# DHT11 data pin
Temp_sensor = 12
DHT = dht11.DHT11(pin = Temp_sensor)

# Gas Sensor via ADC
tgs822 = MCP.MCP3208(1) # Using SPI channel 1 (SPICS1)
#//////////////////


# LCD Display Setup
# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address, if any error, change this address to 0x3f
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1
#//////////////////


# Variables for the program
# Arduino to RPi.GPIO translation variables
LOW = GPIO.LOW
HIGH = GPIO.HIGH

currentMode=1 # 1 = PPM, 2 = mmol/l
GlobalMaxValue=0
GlobalMinValue=0
currentTemperature = 28
currentHumidity = 60

# Initial values for variables
# Starting state for buttons
resetMaxSwitchLastButton = LOW
resetMaxSwitchCurrentButton = LOW
resetSensorSwitchLastButton = LOW
resetSensorSwitchCurrentButton = LOW
toggleModeLastButton = LOW
toggleModeCurrentButton = LOW

# Read gas variables
tempRead1 = 0
tempRead2 = 0
tempRead3 = 0
value1 = 0
value2 = 0
value3 = 0

# General Var
R0 = 4500
lastPPM = 0
#//////////////////



# --------------- LCD Display Methods -----------------
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

# --------------- DHT11 Method Examples -----------------
'''
# Initialise Temp Sensor
DHT = dht11.DHT11(pin = Temp_sensor)

#get DHT11 sensor value
result = DHT.read()
lcd_line1 = "temp:"+str(dht.temperature)+" C"
lcd_line2 = "humid:"+str(dht.humidity)+"%"
'''

# -------------- Missing Built-in functions ----------------------
def delay(ms):
  time.sleep(ms/1000)

def digitalRead(channel):
  return GPIO.input(channel)

# -------------  Ketosense Code ---------------------
def setup():
	# Setup program block

	# Initiate LCD
	lcd_init()

	# Write Welcome message
	printToRow1("Hi I'm Ketosense")
	printToRow2("Time to warm up.")
	delay(2000)

	# Initiate DHT11 temperature and humidity sensor and verify it.
	clearLcd()
	printToRow1("Check DHT sensor: ")
	dht = DHT.read()
	chk = dht.error_code  
	if chk == 0: 
	    printToRow2("Ok. ")
	elif chk == 2: 
	    printToRow2("Checksum error")
	elif chk == 1: 
	    printToRow2("Time out error")
	else:
	    printToRow2("Unknown error")
	delay(2000)


	# Warmup, check that sensor is stabile.
	while (checkIfSensorIsStabile() == False):
		checkIfSensorIsStabile()

	clearLcd()
	printToRow1("Warmup finished")
	delay(1000)
	clearLcd()
	printToRow1("Blow into mouth-")
	printToRow2("piece to start.")

def loop():
	# Loop program block
	global lastPPM
	global currentMode
	global resetMaxSwitchCurrentButton
	global resetMaxSwitchLastButton
	global resetSensorSwitchCurrentButton
	global resetSensorSwitchLastButton
	global toggleModeCurrentButton
	global toggleModeLastButton
	while True:
		# read the analog pin for the gas sensor
#		gas = tgs822.read(0) # Using ADC Ch 0
#		lcd_string("Gas Sensor",LCD_LINE_1)
#		lcd_string("Value:"+str(gas),LCD_LINE_2)
		
		# read three times from gas sensor with 5ms between each reading
  		readsensor()
		# Check if all readings are the same which indictae a stabile behaviour 
		# and if the value is higher or lower update global max and min variables.
		updateNewMaxOrMinWithTempHumidity(tempRead1, tempRead1, tempRead1)

		# print result to display if current value is different to previous value and update the highest value.
		if (acetoneResistanceToPPMf(toResistance(temperatureScaledValue)) != lastPPM):
			lastPPM = acetoneResistanceToPPMf(toResistance(temperatureScaledValue))
			updateScreen()
		

		# read buttons

		# read resetMaxMinSwitch
		resetMaxSwitchCurrentButton = debounce(resetMaxSwitchLastButton, resetMaxSwitchPin)
		if (resetMaxSwitchLastButton == LOW and resetMaxSwitchCurrentButton == HIGH):
			clearLcd()
			printToRow1("Result displayed")
			printToRow2("as mmol/l.")
		resetMaxSwitchLastButton = resetMaxSwitchCurrentButton


		# read resetSensorSwitch
		resetSensorSwitchCurrentButton = debounce(resetSensorSwitchLastButton, resetSensorSwitchPin)
		if (resetSensorSwitchLastButton == LOW and resetSensorSwitchCurrentButton == HIGH):
			while (checkIfSensorIsStabile() == False):
				checkIfSensorIsStabile()
			clearLcd()
			printToRow1("Reset finished")
			delay(1000)
			clearLcd()
			GlobalMaxValue=0
			GlobalMinValue=0
			printToRow1("Blow into mouth-")
			printToRow2("piece to start.") 
		resetSensorSwitchLastButton = resetSensorSwitchCurrentButton


		toggleModeCurrentButton = debounce(toggleModeLastButton, toggleModeSwitchPin)
		if (toggleModeLastButton == LOW and toggleModeCurrentButton == HIGH):
			if (currentMode == 1):
				currentMode=2
				clearLcd()
				printToRow1("Result displayed")
				printToRow2("as mmol/l.")
				delay(1000)
				updateScreen()
			elif (currentMode == 2):
				currentMode=1
				clearLcd()
				printToRow1("Result displayed")
				printToRow2("as PPM.")
				delay(1000)
				updateScreen()
		toggleModeLastButton = toggleModeCurrentButton


#//////////////////////////
#/// Methods start here ///
#//////////////////////////
# Reads sensor 3 times with 5ms delay between reads and store read values in tempRead1, 2 and 3
def ppmToMmol(PPM):
 ppmInmmol = ((PPM / 1000) / 58.08)
 ppmInmmol = ppmInmmol * 1000
 return ppmInmmol

def readsensor():
  global tempRead1
  tempRead1 = tgs822.read(0)
  delay(5)
  tempRead2 = tgs822.read(0)
  delay(5)
  tempRead3 = tgs822.read(0)
  delay(5)


# Update screen with result
def updateScreen():
	clearLcd()
	printToRow1("H:" + str(currentHumidity) + " T:" + str(currentTemperature) + " R:" + str(GlobalMaxValue))
	if (currentMode == 2):
		# Result in mmol/l

		valueNow = ppmToMmol(acetoneResistanceToPPMf(toResistance(temperatureScaledValue)))
		valueMax = ppmToMmol(acetoneResistanceToPPMf(toResistance(GlobalMaxValue)))
		strNow = "{0:0.2f}".format(valueNow)
		strMax = "{0:0.2f}".format(valueMax)
	elif (currentMode == 1):
		# result in PPM
		valueNow = acetoneResistanceToPPMf(toResistance(temperatureScaledValue))
		valueMax = acetoneResistanceToPPMf(toResistance(GlobalMaxValue))
		strNow = str(valueNow)
		strMax = str(valueMax)
	printToRow2(strNow+" Max(" + strMax + ")")
  

# calculate the gas concentration relative to the resistance
def acetoneResistanceToPPMf(resistance):
  tempResistance = resistance
  if (tempResistance > 50000):
  	PPM = 0
  elif (tempResistance < 1):
	PPM = 99999
  else:
	logPPM = (math.log10(tempResistance/R0)*-2.6)+2.7
  	PPM = pow(10, logPPM)

  return int(PPM)



# debounce function
def debounce(last, pin):
  current = digitalRead(pin)
  if (last != current):
    delay(5)
    current = digitalRead(pin)

  return current


# temperature sensor function, values has been hardcoded to humidity = 60 and temperature = 28 to speed up the measuring.
def tempHumidityCompensation(value):
    #chk = DHT.read()
    delay(300)
    # currentHumidity = ((double)DHT11.humidity)
    # Hardcoded after realizing that the temperature and humidity were beahaving stabilly.
    #currentHumidity = 60
    # currentTemperature = ((double)DHT11.temperature)
    #currentTemperature = 28
    # function derrived from regression analysis of the graph in the datasheet
    scalingFactor = (((currentTemperature * -0.02573)+1.898)+((currentHumidity*-0.011)+0.3966))
    # debug
    # clearLcd()
    # printToRow1("Scalefactor:")
    # printFloatToCurrentCursorPossition((float)scalingFactor)
    delay(1000)
    # debugstop*/
    scaledValue = value * scalingFactor
    return int(scaledValue)
    
      
# check if we have new max or min after temperature and humidity scaling has been done. 
def updateNewMaxOrMinWithTempHumidity(value1, value2, value3):
  global GlobalMaxValue
  global GlobalMinValue
  global temperatureScaledValue
  if (value1 == value2 and value1 == value3):
    temperatureScaledValue = tempHumidityCompensation(value1)
    
    if (GlobalMaxValue==0):
      GlobalMaxValue = temperatureScaledValue
    
    if (GlobalMinValue==0):
      GlobalMinValue = temperatureScaledValue
    
    if (temperatureScaledValue < GlobalMinValue):
      GlobalMinValue = temperatureScaledValue
    
    if (temperatureScaledValue > GlobalMaxValue):
      GlobalMaxValue = temperatureScaledValue
    

# check if we have new max or min without temperature and humidity scaling. 
def updateNewMaxOrMin(value1, value2, value3):
  global GlobalMaxValue
  global GlobalMinValue
  global temperatureScaledValue
  if (value1 == value2 and value1 == value3):
    if (GlobalMaxValue==0):
      GlobalMaxValue = value1
    
    if (GlobalMinValue==0):
      GlobalMinValue = value1
    
    if (value1 < GlobalMinValue):
      GlobalMinValue = value1
    
    if (value1 > GlobalMaxValue):
      GlobalMaxValue = value1
    

# Convert the 0-4095 voltage value from gas sensor analog read to a resistance, 1000 is the value of the other resistor in the voltage divide.
def toResistance(reading):
  reading = int(reading)
  vOut = toVoltage(reading)
  if (vOut > 0):
    resistance = ((5/vOut - 1) * 1000)
  else:
    resistance = 100000
  return float(resistance)


# Convert the 0-4095 value from analog read to a voltage.
def toVoltage(reading):
  readingf = float(reading)
  # constant derived from 5/4095 = 0.001220703
  voltageConversionConstant = 0.001220703
  voltageRead = readingf * voltageConversionConstant
  return float(voltageRead)


def printToRow1(string):
  lcd_string(string,LCD_LINE_1)

def printToRow2(string):
  lcd_string(string,LCD_LINE_2)
  
def clearLcd():
  lcd_byte(0x01, LCD_CMD) # Clear screen

def checkIfSensorIsStabile():
  # read samples for 10 seconds
  for i in range(20):
    # read Acetone Sensor
    newCalibrationVal = tgs822.read(0) # Using ADC Ch 0

    # set first sample as baseline
    if (i==0):
      calibrationLow = newCalibrationVal
      calibrationHigh = newCalibrationVal

    # Determine if last sensor reading is higher then previous high
    if (newCalibrationVal > calibrationHigh):
      calibrationHigh = newCalibrationVal
    
    # Determine if sensor reading is lower then previous high
    if (newCalibrationVal < calibrationLow):
      calibrationLow = newCalibrationVal
    
    # Print current max and min to lcd
    clearLcd()
    printToRow1("i:" + str(i))

    printToRow2("Max:" +str(calibrationHigh) + " Min:" + str(calibrationLow))
    delay(1000)

    if ((calibrationHigh - calibrationLow) > 5):
      return False

  #  Check if resistance has not changed more then 5 steps during 10 sec = unstabile sensor
  if ((calibrationHigh - calibrationLow) <= 3 or calibrationHigh <= 135):
    return True 
  else:
    return False



# --------------  Python code to execute if file is used directly ------------------
#  This code only runs when using a command such as '$ sudo python ketosense.py'


if __name__ == '__main__':

  try:
    setup()
    loop()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

# End
GPIO.cleanup()



