# sunpos.py
import math
import time
import utime
import machine
from machine import Pin, UART
from servo import Servo
from time import sleep
from machine import Pin
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from oled import Write, GFX, SSD1306_I2C
from oled.fonts import ubuntu_mono_15, ubuntu_mono_20
from machine import SoftI2C 
from DS3231 import DS3231


# GPS Code
gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
print(gpsModule)

buff = bytearray(255)

TIMEOUT = False
FIX_STATUS = False

latitude = ""
longitude = ""
satellites = ""
GPStime = ""

def getGPS(gpsModule):
    global FIX_STATUS, TIMEOUT, latitude, longitude, satellites, GPStime
    
    timeout = time.time() + 8 
    while True:
        gpsModule.readline()
        buff = str(gpsModule.readline())
        parts = buff.split(',')
    
        if (parts[0] == "b'$GPGGA" and len(parts) == 15):
            if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                print(buff)
                
                latitude = convertToDegree(parts[2])
                if (parts[3] == 'S'):
                    latitude = str(-float(latitude))
                longitude = convertToDegree(parts[4])
                if (parts[5] == 'W'):
                    longitude = str(-float(longitude))
                satellites = parts[7]
                GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                FIX_STATUS = True
                break
                
        if (time.time() > timeout):
            TIMEOUT = True
            break
        utime.sleep_ms(500)
        
def convertToDegree(RawDegrees):

    RawAsFloat = float(RawDegrees)
    firstdigits = int(RawAsFloat/100) 
    nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
    Converted = float(firstdigits + nexttwodigits/60.0)
    Converted = '{0:.6f}'.format(Converted) 
    return str(Converted)
    
# SERVO Code
s1 = Servo(14)       # Servo pin is connected to GP0
 
def servo_Map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 
def servo_Angle(angle):
    if angle < 0:
        angle = 0
    if angle > 180:
        angle = 180
    s1.goto(round(servo_Map(angle,0,180,0,1024))) # Convert range value to angle value
    


# DS3231 Code
i2c = SoftI2C(scl=Pin(1), sda=Pin(0), freq=400000) 
ds = DS3231(i2c)

# OLED Code
WIDTH  = 128                                         
HEIGHT = 64
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=200000)      
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c) 


while True:
    
    def sunpos(when, location, refraction):
        
     
# Extract the passed data
        year, month, day, hour, minute, second, timezone = when
        latitude, longitude = location
# Math typing shortcuts
        rad, deg = math.radians, math.degrees
        sin, cos, tan = math.sin, math.cos, math.tan
        asin, atan2 = math.asin, math.atan2
# Convert latitude and longitude to radians
        rlat = rad(latitude)
        rlon = rad(longitude)
# Decimal hour of the day at Greenwich
        greenwichtime = hour - timezone + minute / 60 + second / 3600
# Days from J2000, accurate from 1901 to 2099
        daynum = (
            367 * year
            - 7 * (year + (month + 9) // 12) // 4
            + 275 * month // 9
            + day
            - 730531.5
            + greenwichtime / 24
        )
# Mean longitude of the sun
        mean_long = daynum * 0.01720279239 + 4.894967873
# Mean anomaly of the Sun
        mean_anom = daynum * 0.01720197034 + 6.240040768
# Ecliptic longitude of the sun
        eclip_long = (
            mean_long
            + 0.03342305518 * sin(mean_anom)
            + 0.0003490658504 * sin(2 * mean_anom)
        )
# Obliquity of the ecliptic
        obliquity = 0.4090877234 - 0.000000006981317008 * daynum
# Right ascension of the sun
        rasc = atan2(cos(obliquity) * sin(eclip_long), cos(eclip_long))
# Declination of the sun
        decl = asin(sin(obliquity) * sin(eclip_long))
# Local sidereal time
        sidereal = 4.894961213 + 6.300388099 * daynum + rlon
# Hour angle of the sun
        hour_ang = sidereal - rasc
# Local elevation of the sun
        elevation = asin(sin(decl) * sin(rlat) + cos(decl) * cos(rlat) * cos(hour_ang))
# Local azimuth of the sun
        azimuth = atan2(
            -cos(decl) * cos(rlat) * sin(hour_ang),
            sin(decl) - sin(rlat) * sin(elevation),
        )
# Convert azimuth and elevation to degrees
        azimuth = into_range(deg(azimuth), 0, 360)
        elevation = into_range(deg(elevation), -180, 180)
# Refraction correction (optional)
        if refraction:
            targ = rad((elevation + (10.3 / (elevation + 5.11))))
            elevation += (1.02 / tan(targ)) / 60
# Return azimuth and elevation in degrees
        return (round(azimuth, 2), round(elevation, 2))
    def into_range(x, range_min, range_max):
        shiftedx = x - range_min
        delta = range_max - range_min
        return (((shiftedx % delta) + delta) % delta) + range_min
    if __name__ == "__main__":
# Retrieve UTC from DS3231
        ds.get_time()
        clock = ds.get_time()
        clock1 = clock[:6]
        list(clock1)
        gmt = [0]
        clock12 = list(clock1) +list(gmt)
        when = clock12
        getGPS(gpsModule)

    if(FIX_STATUS == True):
        print("Printing GPS data...")
        print(" ")
        print("Latitude: "+latitude)
        print("Longitude: "+longitude)
        print("Satellites: " +satellites)
        print("Time: "+GPStime)
        
        
        FIX_STATUS = False
        
    if(TIMEOUT == True):
        print("No GPS data is found.")
        TIMEOUT = False  
        
        
# Renton WA latitude, longitude
        location = (37.456861, -120.2095)
# Get the Sun's apparent location in the sky
        azimuth, elevation = sunpos(when, location, True)
# Output the results
        
        print("\nUTC: ", when)
        print("Where: ", location)
        print("Azimuth: ", azimuth)
        print("Elevation: ", elevation)
        
        
        
        oled.fill(0)
        write20 = Write(oled, ubuntu_mono_20)
        write20.text("Azim: ", 0, 0)
        write20.text(str(round(azimuth)),80,0)
        write20.text("Elev: ", 0, 20)
        write20.text(str(round(elevation)),80,20)
        # write20.text(str (ghj), 0, 43)
        write20.text(str(clock[3:6]),0,43)
        oled.show()
# Returns Angle to Servo -90 degrees because Servo is just 0-90 degree capable        
        servo_Angle(azimuth - 90)  
        
# Program Delay       
        time.sleep(60)
