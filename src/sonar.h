/** @file sonar.h
 *  @brief Sonar header
 *  @author Karol Leszczyński
 */ 

/*
This file is part of Obstacle Avoiding Robot.

Obstacle Avoiding Robot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Obstacle Avoiding Robot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Obstacle Avoiding Robot.  If not, see <http://www.gnu.org/licenses/>.
*/

#define REMOTE_CONTROL 1

#include <stdio.h>

#include <hFramework.h>
#include "Lego_Ultrasonic.h"
#include "Lego_Touch.h"

#if REMOTE_CONTROL == 1
#include "hCloudClient.h"
#endif

using namespace hFramework;
using namespace hSensors;

class Sonar {
 public:
  /**
  * @brief Sonar class constructor.
  * @param hMotor object for left motor.
  * @param hMotor object for right motor.
  * @param hMotor object for scanner motor.
  * @param Lego_Ultrasonic object for scanner sensor.
  * @param Lego_Touch object for left touch sensor.
  * @param Lego_Touch object for right touch sensor.
  */
  Sonar(hMotor *mLeft, hMotor *mRight, hMotor *mScanner,
    Lego_Ultrasonic *sUltrasonic, Lego_Touch *sLeft, Lego_Touch *sRight);

  /**
  * @brief Sonar class destructor.
  */
  ~Sonar() {};
  
  /**
  * @brief Calibrates scanner and then starts rest of the robot.
  */
  void start();
#if REMOTE_CONTROL == 1
  /**
  * @brief Interface configuration.
  */
  void configHandler();
  
  /**
  * @brief Key event.
  */
  void onKeyEvent(KeyEventType type, int code);
  
  /**
  * @brief Button event.
  */
  void onButtonEvent(hId id, ButtonEventType type);
#endif

 private:
  const uint16_t calibrationStep = 930;
  const uint16_t motorPower = 600;
  const uint16_t scannerStep = 180;
  const uint8_t criticalDistance = 20;
  const uint8_t safeDistance = 35;
  const uint16_t withdrawTime = 3000;
  const uint16_t collisionWithdrawTime = 3000;
  const uint16_t rotationTime = 1000;

  /**
  * @brief Gets distane to objects. Measurement is performed 5 times
  *        and mean value is calculated.
  * @return Distance to object.
  */
  float getDist();
  
  /**
  * @brief Gets 7 distances to objects around robot.
  * @param Arrat of size 7.
  */
  void getDistances(float *allDistances);
  
  /**
  * @brief Calibrates scanner by setting it in desired zero position.
  */
  void calibrateScanner();

  /**
  * @brief Gets distances to objects around robot
  *        and then moves towards safest direction.
  */
  void scanAndGo();
  static void scanAndGoWrapper(void *parm) {
    (static_cast<Sonar*>(parm))->scanAndGo();
  }

  /**
  * @brief Detects collision by using touch sensor
  *        and then moves robot backwards for defined time.
  */
  void collisionProcedure();
  static void collisionProcedureWrapper(void *parm) {
    (static_cast<Sonar*>(parm))->collisionProcedure();
  }

  hMutex motor_mutex;
  hMotor *motorLeft;
  hMotor *motorRight;
  hMotor *motorScanner;
  Lego_Ultrasonic *sensorUltrasonic;
  Lego_Touch *sensorLeft;
  Lego_Touch *sensorRight;
#if REMOTE_CONTROL == 1
  bool remoteOn = false;
#endif
};