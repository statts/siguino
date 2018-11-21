# Siguino

![alt text](https://siguino.com/wp-content/uploads/2017/10/proto5-1024x768.jpg "Siguino final design")

Low power board based on Arduino Pro Mini with in-built Sigfox network support

Siguino Eagle schematics, libraries and Arduino code created by Scott Tattersall

Further project info and manufactured boards for purchase at https://siguino.com

Detailed Getting Started Guide: https://siguino.com/step-by-step-guide/

3rd Party libraries / resources used:

Hardware:
- Many basic components (resistors, capacitors, etc) are from the SparkFun Eagle libraries
- LIS3DH accelerometer from SparkFun: https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library
- Wisol Chip from hellogitty/WiSol (greatech.de) https://github.com/hellogitty/WiSOL

Software:
 (To install, go to url and "download zip", in Arduino IDE click Sketch->Include Library->Add zip and select downloaded zip file)
- LowPower library from RocketScream: https://github.com/rocketscream/Low-Power
- SparkFun LIS3DH library (https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library)
- Arduino Sensors library (https://github.com/adafruit/Adafruit_Sensor)
- Arduino Bosch BME280 library (https://github.com/adafruit/Adafruit_BME280_Library)
- AltSoftSerial (https://github.com/PaulStoffregen/AltSoftSerial)

Enclosure:
- CAD (STL) files for sample enclosure that can be 3d printed as needed
![alt text](https://github.com/statts/siguino/raw/master/enclosure/image1.png "Siguino enclosure side view")
![alt text](https://github.com/statts/siguino/raw/master/enclosure/image2.png "Siguino enclosure inside")


# Sample projects
Along with the sigfox_sensors project that sends all sensor information here is another simple example project:

- Weather Station (arduino/weather_station)
Follow the same above software section to install the approriate libraries first, then download the weather station code.
The code can be flashed onto the Siguino via a standard FTDI breakout board (3.3v version only!), e.g. 
https://www.sparkfun.com/products/9873

The code: A simple weather station that sends avg temperature, light level, humidity and atmospheric pressure. If using Wia to process the data (see above getting started link), these can be easily graphed in Wia using a Wia Widget:
https://developers.wia.io/docs/add-a-widget

More information on integrating with Wia:
https://siguino.com/getting-started-with-siguino-and-wia-io/


Example:

![alt text](https://siguino.com/wp-content/uploads/2018/08/temperature_chart.png "Sample Temperature Chart (Wia)")

# Eagle libraries
In eagle, set the directories for libraries to add the libary files included here if not already included, e.g.
$EAGLEDIR/lbr:{PROJECT-DIR}/Libraries/SparkFun-Eagle-Libraries-master:{PROJECT-DIR}/Libraries/WiSOL-master/Eagle_Library

Plus use the New Library -> execute script to add the components in script form from the scripts dir

# Hardware Revision History:

Rev 0.1: First version, trimmed board

Rev 0.2:
1) Add Reset Pin out as well as dtr for bootloading
2) Change C8 10uf format to standard 0603
3) Update routing to account for unsaved work
4) Re-write/tidy arduino sketch
5) Add de-coupling caps 10uf, 100nf to Accel chip VDD
6) Connect INT1 pin on Accel to D3

Rev 0.3:
1) Add Battery Holder
2) Add Reverse polarity Protection
3) Reposition Antenna

Rev 0.4:
1) Replace Temperature chip
2) Re-route temperature chip
3) Add registration hole for battery holder
4) Minor re-routing of vias under chips

Rev 0.5 / 1.0:
1) Added Magnetometer
2) Replaced temp chip with Temp & Pressure (Bosch BME 280)
3) Re-routing for manufacturing

Rev 1.1:
1) Improved Schematic (with thanks to Marnix Janse)
2) Improved Antenna trace (with thanks to Marnix Janse)

# Licence & Contact

Siguino Hardware and Software licenced under GPLv3, any questions or issues please contact scott@dock.ie
