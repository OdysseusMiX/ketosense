# Script to test sensor connections and data

""" 
Components Used:
- Raspberry Pi 2
- TGS 822 Gas Sensor
- DHT11 Tempature/Humidity sensor (integrated on PCB)
- I2C 1602 LCD 16x120 Display (4-pin I2C interface)
- MCP3204-C  12-bit Analog-to-Digital Converter
- Pushbuttons (x3)

"""

# ---------- Setup and Initialization -----------------

# Import Libraries
import mcp3204 as MCP
import smbus
import time
import dht11
import RPi.GPIO as GPIO
#//////////////////

# GPIO Pin Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
# Pushbuttons
sw_1 = 5
sw_2 = 6
sw_3 = 13
GPIO.setup(sw_1,GPIO.IN)
GPIO.setup(sw_2,GPIO.IN)
GPIO.setup(sw_3,GPIO.IN)
# DHT11 data pin
Temp_sensor = 12
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
instance = dht11.DHT11(pin = Temp_sensor)

#get DHT11 sensor value
result = instance.read()
lcd_line1 = "temp:"+str(dht.temperature)+" C"
lcd_line2 = "humid:"+str(dht.humidity)+"%"
'''


# --------------- Main Method --------------------
def main():
  # Main program block

  # Initialise devices
  lcd_init()
  instance = dht11.DHT11(pin = Temp_sensor)
  tgs822 = MCP.MCP3208(1) # Using SPI channel 1 (SPICS1)

  while True:
	#get DHT11 sensor value
	dht = instance.read()

	if dht.is_valid():
		lcd_string("temp:"+str(dht.temperature)+" C",LCD_LINE_1)
		lcd_string("humid:"+str(dht.humidity)+"%",LCD_LINE_2)
		time.sleep(1) # 3 second delay
	else:
		err_code = dht.error_code		
		if err_code == 1:
			err_str = "Missing Data"
		elif err_code == 2:
			err_str = "Checksum Err"
		else:
			err_str = "ERR CODE: " + str(err_code)
		
		lcd_string("Temp Sensor Err",LCD_LINE_1)
		lcd_string(err_str,LCD_LINE_2)
		time.sleep(1)
		
			
	# read the analog pin for the gas sensor
	gas = tgs822.read(0) # Using ADC Ch 0
	lcd_string("Gas Sensor OK",LCD_LINE_1)
	lcd_string("Value:"+str(gas),LCD_LINE_2)
	time.sleep(1) # 3 second delay


if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

# End
GPIO.cleanup()


