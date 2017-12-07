# Siguino

![alt text](https://siguino.com/wp-content/uploads/2017/10/proto5-1024x768.jpg "Siguino final design")

Low power board based on Arduino Pro Mini with in-built Sigfox network support

Eagle schematics, libraries and Arduino code created by Scott Tattersall

Further project info at https://siguino.com

3rd Party libraries / resources used:

Hardware:
- Many basic components (resistors, capacitors, etc) are from the SparkFun Eagle libraries
- LIS3DH accelerometer from SparkFun: https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library
- Wisol Chip from hellogitty/WiSol (greatech.de) https://github.com/hellogitty/WiSOL

Software:
- LowPower library from RocketScream: https://github.com/rocketscream/Low-Power

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
4) Change routing to remove vias from under LIS3DH chip
5) Email CircuitSpecialists about Heat gun powering down
6) Find surface mount photo resistor
7) Investigate mag switch multi-trips
8) Re-write/tidy arduino sketch
9) Add de-coupling caps 10uf, 100nf to Accel chip VDD
10) Connect INT1 pin on Accel to D3

Rev 0.3:
1) Add Battery Holder
2) Add Reverse polarity Protection
3) Reposition Antenna

Rev 0.4:
1) Replace Temperature chip
2) Re-route temperature chip
3) Add registration hole for battery holder
4) Minor re-routing of vias under chips

# Licence & Contact

Siguino Hardware and Software licenced under GPLv3, any questions or issues please contact scott@dock.ie
