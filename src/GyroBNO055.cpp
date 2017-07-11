/**
 * Implmentation of GyroBNO055 class to get direction from gyro.
 */

#include "GyroBNO055.h"
#include "Timer.h"

#include <iostream>
#include <fstream>

using namespace avc;
using namespace std;

namespace {
  // Default configuration file to use for calibration data
  const char* DEFAULT_CONFIG = "/etc/bno055.cal";

  // Address of register to read to get chip address
  const uint8_t chipIdAddr = 0x0;

  // ID byte expected to be returned by board
  const uint8_t chipIdByte = 0xa0;

  // Register to set the mode of operation of the board in
  const uint8_t operationModeAddr = 0x3d;

  // Address to write to when triggering configuration change
  const uint8_t sysTriggerAddr = 0x3f;

  // Address of power control register
  const uint8_t powerModeAddr = 0x3e;

  // Set to normal power mode
  const uint8_t powerModeNormal = 0x0;

  // Address of page ID register (not sure what that means)
  const uint8_t pageIdAddr = 0x7;

  // Configuration mode
  const uint8_t configMode = 0x00;

  // 9 DOF mode plus absolute angles
  const uint8_t ndofMode = 0x0C;

  // The byte containing the sensor's temperatur
  const uint8_t temperatureAddr = 0x34;

  // The byte containing calibration status information
  const uint8_t calibrationStatusAddr = 0x35;

  // The 22 bytes containing calibration data
  const uint8_t calibrationDataAddr = 0x55;

  // The 6 bytes containing the euler values (pitch, roll and heading) 
  const uint8_t eulerAddr = 0x1a;

  // The 2 heading registers (LSB, MSB)
  const uint8_t headingAddr = 0x1a;

  // Acceleration (gravity + motion) data registers
  const uint8_t accelAddr = 0x08;
  const uint8_t accelXAddr = 0x08;
  const uint8_t accelYAddr = 0x0a;
  const uint8_t accelZAddr = 0x0c;

  // Linear acceleration data registers
  const uint8_t linearAccelAddr = 0x28;
  const uint8_t linearAccelXAddr = 0x28;
  const uint8_t linearAccelYAddr = 0x2a;
  const uint8_t linearAccelZAddr = 0x2c;

  // Gravity data registers
  const uint8_t gravityAddr = 0x2e;
  const uint8_t gravityXAddr = 0x2e;
  const uint8_t gravityYAddr = 0x30;
  const uint8_t gravityZAddr = 0x32;

  bool setMode(BlackLib::BlackI2C& i2c, uint8_t mode) {
    if (!i2c.writeByte(operationModeAddr, mode)) {
      return false;
    }
    Timer::sleepNanos(30000000);
    return true;
  }
}


GyroBNO055::GyroBNO055(BlackLib::i2cName i2cDev, int i2cAddr) :
  i2cGyro(i2cDev, i2cAddr)
{
}

GyroBNO055::~GyroBNO055() {
  if (i2cGyro.isOpen()) {
    i2cGyro.close();
  }
}

bool GyroBNO055::reset() {
  if (i2cGyro.isOpen()) {
    i2cGyro.close();
  }

  i2cGyro.open(BlackLib::ReadWrite);
  if (i2cGyro.fail()) {
    cerr << "Failed to open Gyro I2C device\n";
    return false;
  }

  uint8_t idByte = i2cGyro.readByte(chipIdAddr);
  if (idByte != chipIdByte) {
    cerr << "Is BNO055 chip connected? ID byte returned was: 0x" <<
      std::hex << (idByte & 0xff) << " (expected 0x"
	 << (chipIdByte & 0xff) << ")\n" << std::dec;
    i2cGyro.close();
    return false;
  }

  // Reset to configuration mode (in case it was not in this mode)
  if (!setMode(i2cGyro, configMode)) {
    cerr << "Failed to reset BNO055 to configuration mode\n";
    i2cGyro.close();
    return false;
  }

  // Reset (which may take a bit)
  i2cGyro.writeByte(sysTriggerAddr, 0x20);

  for (int i = 0; i < 10; i++) {
    Timer::sleepNanos(100000000);
    idByte = i2cGyro.readByte(chipIdAddr);
    /*
    cerr << "Chip ID reported: 0x" << std::hex
	 << (idByte & 0xff) << " (looking for: 0x"
	 << (chipIdByte & 0xff) << ")\n" << std::dec;
    */

    if (idByte == chipIdByte) {
      //cerr << "Chip ID OK after reset\n";
      break;
    }
  }

  if (idByte != chipIdByte) {
    cerr << "Failed to reset BNO055 board\n";
    i2cGyro.close();
    return false;
  }

  // Give 50 more milliseconds for chip to settle
  Timer::sleepNanos(50000000);

  // Set normal power mode
  i2cGyro.writeByte(powerModeAddr, powerModeNormal);
  Timer::sleepNanos(10000000);

  // Configure to use Adafruit added onboard crystal oscillator (0x80) or
  // internal oscillator (0x00).
  i2cGyro.writeByte(pageIdAddr, 0);
  i2cGyro.writeByte(sysTriggerAddr, 0x80);
  Timer::sleepNanos(10000000);

  if (!setMode(i2cGyro, ndofMode)) {
    cerr << "Failed to reset BNO055 to NDOF mode\n";
    i2cGyro.close();
    return false;
  }
  Timer::sleepNanos(20000000);

  return true;
}

bool GyroBNO055::getCalibrationStatus(int& sys, int& gyro, int& accel, int& mag) {
  uint8_t rawBytes[1];
  bool success = readBytes(calibrationStatusAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    uint8_t flags = rawBytes[0];
    sys = (flags >> 6) & 0x3;
    gyro = (flags >> 4) & 0x3;
    accel = (flags >> 2) & 0x3;
    mag = flags & 0x3;
  }

  return success;
}

bool GyroBNO055::setCalibrationData(uint8_t* store) {
  bool success = false;
  if (setMode(i2cGyro, configMode)) {

    success =
      writeBytes(calibrationDataAddr, store, CALIBRATION_BUFFER_SIZE)
      && setMode(i2cGyro, ndofMode);
  }
  return success;
}

bool GyroBNO055::getCalibrationData(uint8_t* store) {
  bool success = false;
  if (setMode(i2cGyro, configMode)) {
    success =
      readBytes(calibrationDataAddr, store, CALIBRATION_BUFFER_SIZE)
      && setMode(i2cGyro, ndofMode);
  }
  return success;
}

bool GyroBNO055::getTemperature(int& tempC) {
  uint8_t rawBytes[1];
  bool success = readBytes(temperatureAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    tempC = (char) rawBytes[0];
  }

  return success;
}

bool GyroBNO055::getHeading(float& angDeg) {
  uint8_t rawBytes[2];
  bool success = readBytes(eulerAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    angDeg = getDegrees(&rawBytes[0]);
  }

  return success;
}

bool GyroBNO055::getEuler(float& headDeg, float& rollDeg, float& pitchDeg) {
  uint8_t rawBytes[6];
  bool success = readBytes(eulerAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    headDeg = getDegrees(&rawBytes[0]);
    rollDeg = getDegrees(&rawBytes[2]);
    pitchDeg = getDegrees(&rawBytes[4]);
  }

  return success;
}

bool GyroBNO055::getAccel(float& ax, float& ay, float& az) {
  uint8_t rawBytes[6];
  bool success = readBytes(accelAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    ax = getAccel(&rawBytes[0]);
    ay = getAccel(&rawBytes[2]);
    az = getAccel(&rawBytes[4]);
  }

  return success;
}

bool GyroBNO055::getGravity(float& ax, float& ay, float& az) {
  uint8_t rawBytes[6];
  bool success = readBytes(gravityAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    ax = getAccel(&rawBytes[0]);
    ay = getAccel(&rawBytes[2]);
    az = getAccel(&rawBytes[4]);
  }

  return success;
}

bool GyroBNO055::getLinearAccel(float& ax, float& ay, float& az) {
  uint8_t rawBytes[6];
  bool success = readBytes(linearAccelAddr, rawBytes, sizeof(rawBytes));

  if (success) {
    ax = getAccel(&rawBytes[0]);
    ay = getAccel(&rawBytes[2]);
    az = getAccel(&rawBytes[4]);
  }

  return success;
}

bool GyroBNO055::readBytes(uint8_t addr, uint8_t* memory, int len) {
  if (i2cGyro.isOpen() == false) {
    return false;
  }

  int gotLen = i2cGyro.readBlock(addr, memory, len);
  if (len != gotLen) {
    cerr << "Failed to read in " << len << " bytes from BNO055 (got "
	 << gotLen << ")\n";
    i2cGyro.close();
    return false;
  }
  return true;
}

bool GyroBNO055::writeBytes(uint8_t addr, uint8_t* memory, int len) {
  if (i2cGyro.isOpen() == false) {
    return false;
  }

  if (!i2cGyro.writeBlock(addr, memory, len)) {
    cerr << "Failed to write " << len << " bytes to BNO055\n";
    i2cGyro.close();
    return false;
  }
  return true;
}

bool GyroBNO055::readCalibrationData(const char* config) {
  const char* fileName = (config != 0) ? config : DEFAULT_CONFIG;
  bool results = false;
  char buf[CALIBRATION_BUFFER_SIZE];
  int bufLen = sizeof(buf);

  ifstream file(fileName);
  if (file.read(buf, bufLen)) {
    results = setCalibrationData((uint8_t*) buf);
  } else {
    cerr << "Failed to read " << bufLen << " bytes from calibration file "
	 << fileName << "\n";
  }
  file.close();
  return results;
}

bool GyroBNO055::writeCalibrationData(const char* config) {
  const char* fileName = (config != 0) ? config : DEFAULT_CONFIG;
  char buf[CALIBRATION_BUFFER_SIZE];
  int bufLen = sizeof(buf);
  bool results = getCalibrationData((uint8_t*) buf);

  if (results) {
    ofstream file(fileName, ofstream::out | ofstream::binary);
    results = file.write(buf, bufLen);
    file.close();
    if (!results) {
      cerr << "Failed to write " << bufLen << " bytes of calibration data to "
	 << fileName << "\n";
    }
  }
  return results;
}
