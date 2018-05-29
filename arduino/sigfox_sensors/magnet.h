#ifndef MAGNET_H_INCLUDED
#define MAGNET_H_INCLUDED

//Magnetometer Registers
#define LIS3MDL_ADDRESS 0x1E
#define LIS3MDL_WHO_AM_I 0x0F // should return 0x3D
#define LIS3MDL_CTRL_REG1 0x20
#define LIS3MDL_CTRL_REG2 0x21
#define LIS3MDL_CTRL_REG3 0x22
#define LIS3MDL_CTRL_REG4 0x23
#define LIS3MDL_CTRL_REG5 0x24
#define LIS3MDL_STATUS_REG 0x27
#define LIS3MDL_OUT_X_L 0x28 
#define LIS3MDL_OUT_X_H 0x29
#define LIS3MDL_OUT_Y_L 0x2A
#define LIS3MDL_OUT_Y_H 0x2B
#define LIS3MDL_OUT_Z_L 0x2C
#define LIS3MDL_OUT_Z_H 0x2D
#define LIS3MDL_TEMP_OUT_L 0x2E
#define LIS3MDL_TEMP_OUT_H 0x2F 
#define LIS3MDL_INT_CFG 0x30

#define LIS3MDL_4Gauss  0  // 0.15 mG per LSB
#define LIS3MDL_8Gauss  1  // 0.30 mG per LSB
#define LIS3MDL_12Gauss 2  // 0.60 mG per LSB
#define LIS3MDL_16Gauss 3  // 1.20 mG per LSB

namespace Magnetometer {

  
  int readRegister(int deviceAddress, byte address){
    int result;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission(0);
    Wire.requestFrom(deviceAddress, 1); // read a byte
    result = Wire.read();
    return result;   
  }

  bool checkMagnetometer(){
  
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readRegister(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I); // Read WHO_AM_I register for LIS3MDL
    if (d!=0x3D){
      Serial.println(F("Error reading from magnetometer"));
      return false;
    }
    return true;
  }
  
}


#endif // MAGNET_H_INCLUDED
