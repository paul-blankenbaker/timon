/**
 * GyroBNO055 definition.
 */
#ifndef __avc_GyroBNO055_h
#define __avc_GyroBNO055_h

#include <BlackI2C.h>

namespace avc {

  /**
   * The GyroBNO055 class is used to configure and read information
   * from the Adafruit BNO055 absolute 9DOF sensor
   * (https://www.adafruit.com/product/2472).
   *
   * <p>This sensor provides directions about which way you are facing
   * and is one of the easiest and most stable gyros I have used.</p>
   * 
   * <p>This C++ implementation uses the BlackLib I2C classes for
   * communications and a lot of information from Adafruit's sampe
   * Arduino code at https://github.com/adafruit/Adafruit_BNO055. For
   * full details on what the sensor is capable of, refer to the Bosh
   * datasheet
   * (http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf).</p>
   *
   * <p>Example usage:</p>
   *
   * <pre><code>
   * GyroBNO055 gyro;
   * 
   * gyro.reset();
   *
   * float ang;
   * if (gyro.getHeading(ang)) {
   *   cout << "Heading: " << ang << "\n";
   * } else {
   *   cout << "Gyro not responding\n";
   * }
   * </code></pre>
   */
  class GyroBNO055 {

  public:
    // Default I2C address of the sensor
    static const int PRIMARY_I2C_ADDR = 0x28;
    // Alternate address for the sensor if ADR line is tied to 3V
    static const int SECONDARY_I2C_ADDR = 0x29;

    // How many bytes are required to store the sensor's calibration data
    static const int CALIBRATION_BUFFER_SIZE = 22;
    
    /**
     * Construct a new GyroBNO055 instance.
     *
     * @param i2cDev The I2C device to communicate with. From what I
     * can tell, is that it should typically be BlackLib::I2C_1
     * (corresponding to /dev/i2c-1 on Debian). This is the default
     * value if omitted.
     *
     * @param i2cAddr The I2C address of the sensor. This is typically
     * PRIMARY_I2C_ADDR (0x28) unless you have tied the ADR line high,
     * then it is SECONDARY_I2C_ADDR (0x29). This parameter is
     * optional and defaults to PRIMARY_I2C_ADDR if omitted.
     */
    GyroBNO055(BlackLib::i2cName i2cDev = BlackLib::I2C_1,
	       int i2cAddr = PRIMARY_I2C_ADDR);

    /**
     * Destructor won't change the sensor, just cleans up any internal
     * info.
     */
    ~GyroBNO055();

    /**
     * This method initializes the sensor to NDOF (nine degrees of
     * freedom mode).
     *
     * <p>This method should be called once at the start of your
     * program. This method takes a long time (think at least 20
     * milliseconds) to complete. The gyro should be stationary during
     * this time.</p>
     *
     * <p>DO NOT USE for relative turns. Instead, read the current
     * angle and try to rotate to the new angle.</p>
     *
     * @return true If the gyro was successfully reset, false if there
     * was a problem (like gyro not found on the I2C bus).
     */
    bool reset();

    /**
     * Get the calibration status from the sensor.
     *
     * @param sys Where to store the system calibration value (0 bad to 3 best).
     * @param gyro Where to store the system calibration value (0 bad to 3 best).
     * @param accel Where to store the system calibration value (0 bad to 3 best).
     * @param mag Where to store the system calibration value (0 bad to 3 best).
     *
     * @return true If value returned, false if error getting value.
     */
    bool getCalibrationStatus(int& sys, int& gyro, int& accel, int& mag);

    /**
     * Retrieves the calibration data from the sensor (you can save
     * this and then use the set method to restore it).
     *
     * @param store Buffer that is CALBRATION_DATA_BUFFER_SIZE bytes in size.
     * @return true If we were able to retrieve the data, false if not.
     */
    bool getCalibrationData(uint8_t* store);

    /**
     * Loads previously saved calibration data back into the sensor.
     *
     * @param store Buffer that contains CALBRATION_DATA_BUFFER_SIZE
     * bytes of calibration data (from prior getCalibrationData()
     * call).
     *
     * @return true If we were able to apply the data, false if not.
     */
    bool setCalibrationData(uint8_t* store);

    /**
     * Reads calibration data in from configuration file and then
     * applies the calibration data via {@link #setCalibrationData}.
     *
     * @param config Full path to configuration file to load (omit or
     * pass 0 to default to "/etc/bno055.cal").
     *
     * @return true If able to read data from file and apply it to sensor.
     */
    bool readCalibrationData(const char* config = 0);

    /**
     * Gets calibration information from sensor then saves it to a
     * file for later use.
     *
     * @param config Full path to configuration file to write to (omit or
     * pass 0 to default to "/etc/bno055.cal").
     *
     * @return true If able to get information from sensor and store in file.
     */
    bool writeCalibrationData(const char* config = 0);

    /**
     * Get the sensor temperature in degrees Celsius.
     *
     * @param tempC Where to store the temperature read from the sensor.
     *
     * @return true If value returned, false if error getting value.
     */
    bool getTemperature(int& tempC);

    /**
     * Get the current heading of the gyro (which way you are facing).
     *
     * @param angDeg Where to store the results if information is available.
     * Value set will be in the range of [0, 360).
     *
     * @return true If value returned, false if error getting value.
     */
    bool getHeading(float& angDeg);

    /**
     * Get the current heading, roll and pitch in a single operation.
     *
     * @param headDeg Where to store the heading if information is available.
     * Value set will be in the range of [0, 360).
     * @param headDeg Where to store the roll if information is available.
     * Value set will be in the range of [-90, 90).
     * @param headDeg Where to store the pitch if information is available.
     * Value set will be in the range of [-180, 180).
     *
     * @return true If values returned, false if error getting values.
     */
    bool getEuler(float& headDeg, float& rollDeg, float& pitchDeg);

    /**
     * Get the current acceleration (gravity + linear motion) in a
     * single operation.
     *
     * @param ax Where to store the acceleration in the X axis (meters/sec^2).
     * @param ay Where to store the acceleration in the Y axis (meters/sec^2).
     * @param az Where to store the acceleration in the Z axis (meters/sec^2).
     *
     * @return true If values returned, false if error getting values.
     */
    bool getAccel(float& ax, float& ay, float& az);

    /**
     * Get the current acceleration due to gravity in a single
     * operation.
     *
     * @param ax Where to store the acceleration in the X axis (meters/sec^2).
     * @param ay Where to store the acceleration in the Y axis (meters/sec^2).
     * @param az Where to store the acceleration in the Z axis (meters/sec^2).
     *
     * @return true If values returned, false if error getting values.
     */
    bool getGravity(float& ax, float& ay, float& az);

    /**
     * Get the current linear acceleration (no gravity) in a single operation.
     *
     * @param ax Where to store the acceleration in the X axis (meters/sec^2).
     * @param ay Where to store the acceleration in the Y axis (meters/sec^2).
     * @param az Where to store the acceleration in the Z axis (meters/sec^2).
     *
     * @return true If values returned, false if error getting values.
     */
    bool getLinearAccel(float& ax, float& ay, float& az);

    /**
     * Method that converts two byte value stored in memory to
     * floating point degrees.
     *
     * @param memory Pointer to memory location where bytes are found
     * (must be at least two bytes long).
     *
     * @return BNO055 integer value converted to floating point degrees.
     */
    static float getDegrees(const uint8_t* memory);

    /**
     * Method that converts two byte value stored in memory to
     * floating point acceleration in meters/sec^2.
     *
     * @param memory Pointer to memory location where bytes are found
     * (must be at least two bytes long).
     *
     * @return BNO055 integer value converted to floating point acceleration.
     */
    static float getAccel(const uint8_t* memory);

    /**
     * Dumps debug information about the gyro to the output stream provided.
     */
    std::ostream& dumpInfo(std::ostream& out) const;

  private:
    // Checks to see if read operation is OK
    bool readBytes(uint8_t addr, uint8_t* memory, int len);
    // Checks to see if write operation is OK
    bool writeBytes(uint8_t addr, uint8_t* memory, int len);

    static short getShort(const uint8_t* memory);

    static float convertShort(const uint8_t* memory, float scale);

    BlackLib::BlackI2C i2cGyro;
  };

  //
  // Inline implementation of functions
  //

  inline short GyroBNO055::getShort(const uint8_t* memory) {
    short shortVal = (((memory[1] & 0xff) << 8) + (memory[0] & 0xff));
    return shortVal;
  }

  inline float GyroBNO055::convertShort(const uint8_t* memory, float scale) {
    float floatVal = getShort(memory) / scale;
    return floatVal;
  }

  inline float GyroBNO055::getDegrees(const uint8_t* memory) {
    return convertShort(memory, 16.0f);
  }

  inline float GyroBNO055::getAccel(const uint8_t* memory) {
    return convertShort(memory, 100.0f);
  }

}

#endif
