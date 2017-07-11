/**
 * Simple test of using the GyroBNO055 class. Displays relative angle
 * and lights up LEDs based on rotation from starting point.
 *
 * To compile/run:
 *
 *   make name=gyro && sudo build/gyro
 *
 * Use ^C to terminate.
 */

#include "UserLeds.h"
#include "GyroBNO055.h"
#include "Timer.h"

#include <BlackGPIO.h>

#include <iostream>
#include <iomanip>
#include <cmath>

#include <signal.h>

using namespace avc;
using namespace std;

namespace {
  // Flag will be set when user terminates via ^C or uses kill on process
  bool hasBeenInterrupted = false;

  void interrupted(int sig) {
    hasBeenInterrupted = true;
  }
}


void getArgInt(int& value, const char* op, int& idx, int argc, const char** argv) {
  if (strcmp(argv[idx], op) == 0) {
    idx++;
    if (idx < argc) {
      value = atoi(argv[idx]);
    }
  }
}

//
// Main entry point into the code basically waits for user
// to press button on BBB then runs the autonomous code
// 
int main(int argc, const char** argv) {
  signal(SIGINT, interrupted);
  signal(SIGTERM, interrupted);
  UserLeds& leds = UserLeds::getInstance();
  int waitCnt = 0;
  int samplesPerSecond = 10;

  // Get command line options
  bool showTemp = false;
  bool calibrate = false;
  bool showHelp = false;
  bool getGravity = false;
  bool getAccel = false;
  bool getLinearAccel = false;

  for (int i = 0; i < argc; i++) {
    calibrate = calibrate || (strcmp(argv[i], "-c") == 0);
    showTemp = showTemp || (strcmp(argv[i], "-t") == 0);
    showHelp = showHelp || (strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0);
    getGravity = getGravity || (strcmp(argv[i], "-g") == 0);
    getAccel = getAccel || (strcmp(argv[i], "-a") == 0);
    getLinearAccel = getLinearAccel || (strcmp(argv[i], "-l") == 0);

    getArgInt(samplesPerSecond, "-s", i, argc, argv);
  }

  // Default to gravity if not specified on command line
  getGravity = getGravity || (!getAccel && !getLinearAccel);

  if (showHelp) {
    cout << "\nUsage: " << argv[0]
	 << " [-h|--help] [-g|-a|-l] [-s RATE] [-c] [-t]\n"
	 << "\nWhere:\n\n"
	 << "  -h|--help\n"
	 << "    Displays this usage information\n"
	 << "  -g|-a|-l\n"
	 << "    Read gravity vector (-g), acceleration vector (-a)\n"
	 << "    or linear accel vector (-l). Gravity is default\n"
	 << "  -s RATE\n"
	 << "    How many times to read data per second (up to 100).\n"
	 << "  -t\n"
	 << "    Displays temperature before reporting data\n"
	 << "  -c\n"
         << "    Performs calibration on sensor and updates calibration\n"
         << "    file /etc/bno055.cal\n"
	 << "\n";
    return 0;
  }

  // Create and reset the gyro
  GyroBNO055 gyro;
  if (!gyro.reset()) {
    cerr << "Failed to reset Gyro\n";
    return 1;
  }

  if (!calibrate) {
    gyro.readCalibrationData();
  }

  while (calibrate) {
    int sys, gyroc, accel, mag;
    if (gyro.getCalibrationStatus(sys, gyroc, accel, mag)) {
      cout << "Calibration waiting for 3's: sys: " << sys << "  gyro: " << gyroc
	   << "  accel: " << accel << "  mag: " << mag << "\n";
      if ((sys == 3) &&  (gyroc == 3) && (accel == 3) && (mag == 3)) {
	gyro.writeCalibrationData();
	break;
      }
      Timer::sleep(0.25);
    } else {
      cerr << "Failed to get calibration status\n";
      return 1;
    }
  }

  if (showTemp) {
    int tempC;
    if (gyro.getTemperature(tempC)) {
      cout << "Sensor temperature: " << tempC << " Celsius\n";
    } else {
      cerr << "Failed to get sensor temperature\n";
      return 1;
    }
  }

  cout << "Time,Heading,Roll,Pitch,ax,ay,az\n";

  // Give a little settle time
  Timer::sleep(0.5);

  Timer timer;
  int nanoPeriod = 1000000000 / samplesPerSecond;
  float lastTime = 0;

  while (hasBeenInterrupted == false) {
    timer.sleepUntilNextNano(nanoPeriod);
    float heading, roll, pitch;
    //Timer gyroReadTime;
    float ax, ay, az;

    //float timeBeforeRead = timer.secsElapsed();
    float time = timer.secsElapsed();

    bool ok = gyro.getEuler(heading, roll, pitch);
    if (ok) {
      if (getGravity) {
	ok = gyro.getGravity(ax, ay, az);
      } else if (getAccel) {
	ok = gyro.getAccel(ax, ay, az);
      } else {
	ok = gyro.getLinearAccel(ax, ay, az);
      }
    }

    if (ok) {
      //float timeAfterRead = timer.secsElapsed();
      //float time = (timeBeforeRead + timeAfterRead) / 2;

      //gyroReadTime.pause();
      // Convert to signed value
      if (heading > 180.0) {
	heading = heading - 360.0;
      }
      float degOff = abs(heading);
      int ledState = 0xf;

      if (degOff > 30.0) {
	ledState = (heading < 0) ? 0xc : 0x3;
      } else if (degOff > 10.0) {
	ledState = (heading < 0) ? 0x8 : 0x1;
      } else if (degOff > 3.0) {
	ledState = (heading < 0) ? 0x4 : 0x2;
      } else if (degOff > 1) {
	ledState = 0;
      } else if (degOff > 0.25) {
	ledState = 0x6;
      }
      leds.setState(ledState);

      cout
	<< setprecision(6) << fixed
	<< (time - lastTime)
	<< setprecision(4) << fixed
	<< ',' << heading 
	//<< " degrees off (" << (gyroReadTime.secsElapsed() * 1000.0)
	//<< " msecs to read)"
	<< ',' << roll
	<< ',' << pitch
	<< setprecision(2) << fixed
	<< ',' << ax
	<< ',' << ay
	<< ',' << az
	<< "\n";

      lastTime = time;
    } else {
      cerr << "Problem reading gyro data\n";
      break;
    }
  }

  leds.setState(0);

  return 0;
}
