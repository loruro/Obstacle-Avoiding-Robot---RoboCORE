/** @file sonar.cpp
 *  @brief Sonar class
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

#include "sonar.h"

Sonar::Sonar(hMotor *mLeft, hMotor *mRight, hMotor *mScanner,
    Lego_Ultrasonic *sUltrasonic, Lego_Touch *sLeft, Lego_Touch *sRight) {
  motorLeft = mLeft;
  motorRight = mRight;
  motorScanner = mScanner;
  sensorUltrasonic = sUltrasonic;
  sensorLeft = sLeft;
  sensorRight = sRight;
}

void Sonar::start() {
  calibrateScanner();
  sys.taskCreate(&scanAndGoWrapper, this, 2, 1000);
  sys.taskCreate(&collisionProcedureWrapper, this, 2, 1000);
}

float Sonar::getDist() {
  float meanDistance = 0;
  for (uint8_t i = 0; i < 5; i++) {
    meanDistance += sensorUltrasonic->readDist();
    sys.delay(10);
  }
  meanDistance /= 5;
  return meanDistance;
}

void Sonar::getDistances(float *allDistances) {
  int16_t angle = -scannerStep * 3;
  for (uint8_t i = 0; i < 7; i++) {
    motorScanner->rotAbs(angle, 1000, 1);
    allDistances[i] = getDist();
    angle += scannerStep;
  }
}

void Sonar::scanAndGo() {
  while (true) {
#if REMOTE_CONTROL == 1
    if (!remoteOn) {
#endif
      float allDistances[7];
      getDistances(allDistances);

      float distanceForward = min(min(allDistances[2], allDistances[3]),
          allDistances[4]);

      motor_mutex.take();

      if (distanceForward > safeDistance) {
        motorRight->setPower(-motorPower);
        motorLeft->setPower(-motorPower);
      } else {
        float distanceLeft = min(min(allDistances[0], allDistances[1]),
            allDistances[2]);
        float distanceRight = min(min(allDistances[4], allDistances[5]),
            allDistances[6]);

        if (distanceForward > distanceLeft && distanceForward > distanceRight) {
          motorRight->setPower(-motorPower);
          motorLeft->setPower(-motorPower);
        } else if (distanceLeft > distanceRight) {
          motorRight->setPower(-motorPower);
          motorLeft->setPower(motorPower);
          sys.delay(rotationTime);
          motorRight->setPower(-motorPower * 0.5);
          motorLeft->setPower(-motorPower * 0.5);
        } else if (distanceRight > criticalDistance) {
          motorRight->setPower(motorPower);
          motorLeft->setPower(-motorPower);
          sys.delay(rotationTime);
          motorRight->setPower(-motorPower * 0.5);
          motorLeft->setPower(-motorPower * 0.5);
        } else {
          motorRight->setPower(motorPower);
          motorLeft->setPower(motorPower);
          sys.delay(withdrawTime);
          motorRight->setPower(0);
          motorLeft->setPower(0);
        }
      }

      motor_mutex.give();

#if REMOTE_CONTROL == 1
    }
#endif
  }
}

void Sonar::collisionProcedure() {
  while (true) {
#if REMOTE_CONTROL == 1
    if (!remoteOn) {
#endif
      bool pressedLeft = sensorLeft->isPressed();
      bool pressedRight = sensorRight->isPressed();
      if (pressedLeft || pressedRight) {
        motor_mutex.take();

        motorRight->setPower(pressedRight * motorPower);
        motorLeft->setPower(pressedLeft * motorPower);
        sys.delay(collisionWithdrawTime);

        motor_mutex.give();
      }
#if REMOTE_CONTROL == 1
    }
#endif
  }
}

void Sonar::calibrateScanner() {
  motorScanner->setPower(-300);
  while (true) {
    uint8_t distance = sensorUltrasonic->readDist();
    sys.delay(10);
    if (distance < 10) {
      motorScanner->resetEncoderCnt();
      break;
    }
  }
  motorScanner->rotAbs(calibrationStep, 300, 1);
  motorScanner->resetEncoderCnt();

}

#if REMOTE_CONTROL == 1
void Sonar::configHandler() {
  platform.ui.button("Remote").setText("Remote control");
}

void Sonar::onKeyEvent(KeyEventType type, int code) {
  if (remoteOn) {
    if (type == KeyEventType::Pressed) {
      uint16_t currentMotorPower = motorPower / 10;
      if (code == KEY_UP) {
        for (uint8_t i = 0; i < 10; i++) {
          motorRight->setPower(-currentMotorPower);
          motorLeft->setPower(-currentMotorPower);
          currentMotorPower += motorPower / 10;
          sys.delay(10);
        }
      } else if (code == KEY_DOWN) {
        for (uint8_t i = 0; i < 10; i++) {
          motorRight->setPower(currentMotorPower);
          motorLeft->setPower(currentMotorPower);
          currentMotorPower += motorPower / 10;
          sys.delay(10);
        }
      } else if (code == KEY_LEFT) {
        for (uint8_t i = 0; i < 10; i++) {
          motorRight->setPower(-currentMotorPower);
          motorLeft->setPower(currentMotorPower);
          currentMotorPower += motorPower / 10;
          sys.delay(10);
        }
      } else if (code == KEY_RIGHT) {
        for (uint8_t i = 0; i < 10; i++) {
          motorRight->setPower(currentMotorPower);
          motorLeft->setPower(-currentMotorPower);
          currentMotorPower += motorPower / 10;
          sys.delay(10);
        }
      }
    } else if (type == KeyEventType::Released) {
      motorRight->setPower(0);
      motorLeft->setPower(0);
    }
  }
}

void Sonar::onButtonEvent(hId id, ButtonEventType type) {
  if (id == "Remote") {
    if (type == ButtonEventType::Pressed) {
      remoteOn ^= 1;
      if (remoteOn) {
        motorRight->setPower(0);
        motorLeft->setPower(0);
      }
    }
  }
}
#endif