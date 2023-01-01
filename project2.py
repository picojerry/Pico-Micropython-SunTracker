# sunpos.py
import math
import time
import utime
import network
import machine

from time import sleep
from machine import Pin
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from oled import Write, GFX, SSD1306_I2C
from oled.fonts import ubuntu_mono_15, ubuntu_mono_20

ssid = 'ockwig'
password = 'Nomerson1234'

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(20)
    print(wlan.ifconfig())
    
try:
    connect()
except KeyboardInterrupt:
    machine.reset()


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


        time.localtime()
        abc = time.localtime()
        abc = abc[:6]
        list(abc)
    
        cde = [-8]
        abc = list(abc) + list(cde)

     

        when = (abc)
        
        
# Renton WA latitude, longitude
        location = (47.4569019, -122.2095766)
# Get the Sun's apparent location in the sky
        azimuth, elevation = sunpos(when, location, True)
# Output the results
        print(time.localtime())
        print("\nWhen: ", when)
        print("Where: ", location)
        print("Azimuth: ", azimuth)
        print("Elevation: ", elevation)
        year, month, day, hour, mins, secs, weekday, yearday = time.localtime()
        ghj = ("{}:{:02d}:{:02d}".format(hour, mins, secs))
        
        oled.fill(0)
        write20 = Write(oled, ubuntu_mono_20)
        write20.text("Azim: ", 0, 0)
        write20.text(str(round(azimuth)),80,0)
        write20.text("Elev: ", 0, 20)
        write20.text(str(round(elevation)),80,20)
        write20.text(str (ghj), 0, 40)

        oled.show()
    
        
        time.sleep(60)
        
        
        
        







                 


    
        
        