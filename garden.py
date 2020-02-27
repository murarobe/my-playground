# Import time for delays
import time
# These next are imported for use with MCP3008
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
# Import for GPIO communication
import RPi.GPIO as GPIO

print("Starting script...")
# GPIO number format
print("Mapping relays to GPIO pins...")
relays = [19, 16, 26, 20]

# prSec1 = GPIODevice # Photo Resistive Sensor to monitor sunlight - for future
# thSensor1 = GPIODevice # Temperature/Humidity Sensor - for future

# Warnings are annoying
GPIO.setwarnings(False)

# Declaring that I am referencing GPIO Number
GPIO.setmode(GPIO.BCM)

print("Initializing relays...")
for i in range(len(relays)):
    GPIO.setup(relays[i], GPIO.OUT, initial=1)
    GPIO.output(relays[i], GPIO.HIGH)
    print("Relay at GPIO " + str(relays[i]) + " initialized...")

# Initiating the MCP3008 for analog inputs
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)
mcp = MCP.MCP3008(spi, cs)
print("MCP3008 chip initialized...")
# Grabbing value from channel 0 on the MCP3008
channel0 = AnalogIn(mcp, MCP.P0)
# Grabbing value from channel 1 on the MCP3008
channel1 = AnalogIn(mcp, MCP.P1)
print("Analog input devices declared...")
time.sleep(1)

# Calibrated to each sensor
# Sensor 1, Sensor 2, etc
oldMin = [1.7724879835202563, 1.7724879835202563]
oldMax = [3.200126486610208, 2.9036575875486377]

# Create the range list
oldRange = []
for i in range(len(oldMin)):
    calc = oldMax[i] - oldMin[i]
    oldRange.append(calc)
# print(oldRange)

# Constant - converting everything from 0-100 value
newMin = 0
newMax = 100
newRange = newMax - newMin

print("Begin polling sensors...")
time.sleep(1)
# The meat and potatoes of the program
while True:
    try:
        moistureSensors = [channel0.voltage, channel1.voltage]
        # print("Moisture Sensor 1 Volts: " + str(channel0.voltage))
        # print("Moisture Sensor 2 Volts: " + str(channel1.voltage))
        for i in range(len(moistureSensors)):
            val = 100 - (((moistureSensors[i] -
                         oldMin[i]) * newRange) /
                         oldRange[i]) + newMin
            moistureSensors[i] = abs(val)
            print("Moisture Sensor " + str(i+1) + " "
                  + str(moistureSensors[i]))
            # If Moisture @ i < some number, close relay @ i
            if(moistureSensors[i] < 50):
                print("Relay " + str(i+1) + " at GPIO "
                      + str(relays[i]) + " ON!")
                GPIO.output(relays[i], GPIO.LOW)
                time.sleep(10)
                print("Relay " + str(i+1) + " at GPIO "
                      + str(relays[i]) + " OFF!")
                GPIO.output(relays[i], GPIO.HIGH)
        time.sleep(10)
    except KeyboardInterrupt:
        print("Exiting...")
        quit()

# Get data from sensors every 10 minutes for final deployment
# Get data from sensors every 5 seconds for testing
# If mSensor1 is dry, run pump 5 seconds for testing,
# 20 seconds for deployment