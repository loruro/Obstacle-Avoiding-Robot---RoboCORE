/** @file sonar.h
 *  @brief Sonar header
 *  @author Karol Leszczyński
 */ 
 
#define REMOTE_CONTROL 1

#include <hFramework.h>
#include <stdio.h>
#include "Lego_Ultrasonic.h"
#include "Lego_Touch.h"

#if REMOTE_CONTROL == 1
#include "hCloudClient.h"
#endif

using namespace hFramework;
using namespace hSensors;

class Sonar
{
public:
	Sonar(hMotor *mLeft, hMotor *mRight, hMotor *mScanner,
	Lego_Ultrasonic *sUltrasonic, Lego_Touch *sLeft, Lego_Touch *sRight);
	~Sonar();
	void start();

	#if REMOTE_CONTROL == 1
	void configHandler();
	void onKeyEvent(KeyEventType type, int code);
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

	hMutex motor_mutex;
	hMotor *motorLeft;
	hMotor *motorRight;
	hMotor *motorScanner;
	Lego_Ultrasonic *sensorUltrasonic;
	Lego_Touch *sensorLeft;
	Lego_Touch *sensorRight;

	float getDist();
	void getDistances(float *allDistances);

	void scanAndGo();
	static void scanAndGoWrapper(void *parm){
		(static_cast<Sonar*>(parm))->scanAndGo();
	}

	void collisionProcedure();
	static void collisionProcedureWrapper(void *parm){
		(static_cast<Sonar*>(parm))->collisionProcedure();
	}

	void calibrateScanner();

	#if REMOTE_CONTROL == 1
	bool remoteOn = false;
	#endif
};
