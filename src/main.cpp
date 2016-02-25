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

/** @file main.cpp
 *  @brief Obstacle Avoiding Robot
 *  @author Karol Leszczyński
 */ 

#include <stdio.h>

#include <hFramework.h>
#include "Lego_Ultrasonic.h"
#include "Lego_Touch.h"

#include "sonar.h"

using namespace hFramework;
using namespace hSensors;

Sonar *sonar;

#if REMOTE_CONTROL == 1
#include "hCloudClient.h"

void configHandler() {
  sonar->configHandler();
}

void onKeyEvent(KeyEventType type, int code) {
  sonar->onKeyEvent(type, code);
}

void onButtonEvent(hId id, ButtonEventType type) {
  sonar->onButtonEvent(id, type);
}
#endif

/**
* @brief Main.
*/
void hMain(void) {
  sys.setLogDev(&Serial);

  hMotor *mLeft = &hMot3;
  hMotor *mRight = &hMot4;
  hMotor *mScanner = &hMot1;
  Lego_Ultrasonic sensorUltrasonic(hSens1);
  Lego_Touch sensorLeft(hSens3);
  Lego_Touch sensorRight(hSens2);

  sonar = new Sonar(mLeft, mRight, mScanner, &sensorUltrasonic, &sensorLeft,
      &sensorRight);

#if REMOTE_CONTROL == 1
  platform.begin(&Usb);
  platform.ui.configHandler = configHandler;
  platform.ui.onKeyEvent = onKeyEvent;
  platform.ui.onButtonEvent = onButtonEvent;
#endif

  sonar->start();

  while (true) {
    sys.delay(20);
  }
}