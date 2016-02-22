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