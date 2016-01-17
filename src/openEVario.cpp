/*
 ============================================================================
 Name        : openEVario.cpp
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C++,
 ============================================================================
 */

#include <random>
#include <iostream>
#include <math.h>
#include <sys/time.h>

#include "GliderVarioStatus.h"
#include "GliderVarioTransitionMatrix.h"
#include "RotationMatrix.h"
#include "FastMath.h"

using namespace std;
using namespace openEV;

mt19937 randomGenerator;

FloatType x = 0;

int main(void) {
  double U1,U2,V1,V2,S,polarFactor,X,Y	;
  mt19937::result_type min;
  double range;
  double sqrSum = 0.0,sum = 0.0,absSum =0.0;
  GliderVarioStatus ovStatus;
  GliderVarioTransitionMatrix ovTransition;
int i;

  // Test of fastMath.
  FloatType j;
  struct timeval tv1,tv2,tv3,tv4;

  cout << "Start test sin" << endl;


  gettimeofday(&tv1,NULL);
  for (i=0;i<100000;i++) {
      for (j=0.0;j<=360;j+=1.0){
	  x = sin(j);
      }
  }
  gettimeofday(&tv2,NULL);
  cout << 100000*360 << "End test sin" << endl;
  cout << " sin calls took " << double(tv2.tv_sec-tv1.tv_sec)+double(tv2.tv_usec-tv1.tv_usec)/1000000000 << endl;
  gettimeofday(&tv3,NULL);
  for (i=0;i<100000;i++) {
      for (j=0.0;j<=360;j+=1.0){
	  x = FastMath::fastSin(j);
      }
  }
  gettimeofday(&tv4,NULL);
  cout << "End test fastSin" << endl;
  cout << 100000*360 << " fastSin calls took " << double(tv4.tv_sec-tv3.tv_sec)+double(tv4.tv_usec-tv3.tv_usec)/1000000000 << endl;
  cout << endl;

  // Test of rotation matrix
  RotationMatrix rotMat (30.0f,0.0f,-20.0f);
  Vector3DType worldRot (0.0f,0.0f,-18.0f);
  Vector3DType planeRot;
  rotMat.calcWorldVectorToPlaneVector(worldRot,planeRot);
  cout << "-----------" << endl;
  cout << "Yaw, pitch roll = " << rotMat.getYaw() << ", " << rotMat.getPitch() << ", " << rotMat.getRoll() << endl;
  cout << "-----------" << endl;
  cout << "rotMatWorldToPlane = " << endl;
  cout << rotMat.getMatrixGloToPlane() << endl;
  cout << "-----------" << endl;
  cout << "worldRot = " << endl;
  cout << worldRot << endl;
  cout << "-----------" << endl;
  cout << "planeRot = " << endl;
  cout << planeRot << endl;

  // test the transition matrix
  // Initialize the status vector
  ovStatus.longitude = 55.0f;
  ovStatus.latitude = 10.0f;
  ovStatus.altMSL = 500.0f;
  ovStatus.yawAngle = 30.0f;
  ovStatus.pitchAngle = 0.0f;
  ovStatus.rollAngle = 20.0f;

  // Speeds and directions
  ovStatus.groundSpeedNorth = 15.0f;
  ovStatus.groundSpeedEast = 25.0f;
  ovStatus.trueAirSpeed = 30.0f;
  ovStatus.heading = 50.0f;
  ovStatus.rateOfSink = -1.0f;
  ovStatus.verticalSpeed = -2.0;

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // If the plane is pitched up an acceleration on the X axis would speed the plane upward, not forward.
  ovStatus.accelX = 0.0f;
  ovStatus.accelY = 0.0f;
  ovStatus.accelZ = -9.81/FastMath::fastCos(ovStatus.rollAngle);

  // Turn rates in reference to the body coordinate system
  ovStatus.rollRateX = 0.0f;
  ovStatus.pitchRateY = 0.0f;
  {   // intermediate calculation to estimate the turn radius, and the resulting turn rate.
	  FloatType angularAccel = 9.81 * FastMath::fastSin(ovStatus.rollAngle)/FastMath::fastCos(ovStatus.rollAngle);
	  // circular radius and and acceleration are defined as:
	  // a = v^2 / r, i.e. r = v^2 / a
	  FloatType radius = ovStatus.trueAirSpeed*ovStatus.trueAirSpeed/angularAccel;
	  // the circumfence of the circle is 2PI*r, with the speed the time for a full 360 deg circle is therefore
	  // t = 2PIr/TAS
	  FloatType timeFullCircle = 2*M_PI*radius/ovStatus.trueAirSpeed;
	  RotationMatrix rotMat1 (ovStatus.yawAngle,ovStatus.pitchAngle,ovStatus.rollAngle);

	  ovStatus.yawRateZ = 360/timeFullCircle;

	  Vector3DType worldRot (ovStatus.rollRateX,ovStatus.pitchRateY,ovStatus.yawRateZ);
	  Vector3DType planeRot;

	  rotMat1.calcWorldVectorToPlaneVector(worldRot,planeRot);

	  cout << " -- yaw rate calculation ------------" << endl;
	  cout << " angularAccel      = " << angularAccel << endl;
	  cout << " radius            = " << radius << endl;
	  cout << " timeFullCircle    = " << timeFullCircle << endl;
	  cout << " ovStatus.yawRateZ = " << ovStatus.yawRateZ << endl;
	  cout << " World rotation rate vector = " << endl << worldRot << endl;
	  cout << " Plane rotation rate vector = " << endl << planeRot << endl;

	  ovStatus.rollRateX = planeRot(0);
	  ovStatus.pitchRateY = planeRot(1);
	  ovStatus.yawRateZ   = planeRot(2);
  }

  // Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
  ovStatus.gyroBiasX = 0.0f;
  ovStatus.gyroBiasY = 0.0f;
  ovStatus.gyroBiasZ = 0.0f;
  ovStatus.windSpeedNorth = 0.0f;
  ovStatus.windSpeedEast = 0.0f;
  ovStatus.thermalSpeed = 0.0f;

  cout << "-- ovStatus T = " << endl << ovStatus;

  ovTransition.calcTransitionMatrix(0.1,ovStatus);
  cout << "-- ovTransition = " << endl << ovTransition.getTransitionMatrix() << endl;
  ovStatus.getStatusVector() = ovTransition.getTransitionMatrix() * ovStatus.getStatusVector();
  cout << "-- ovStatus after 0.1 sec = " << endl << ovStatus;
  for (i = 0; i< 20; i++) {
	  ovTransition.calcTransitionMatrix(0.1,ovStatus);
	  ovStatus.getStatusVector() = ovTransition.getTransitionMatrix() * ovStatus.getStatusVector();
	  cout << ovStatus;

  }

	min = randomGenerator.min();
	range = double(randomGenerator.max()-randomGenerator.min());

	cout << "Hello World" << endl; /* prints Hello World */

	cout << " min of randomGenerator = " << min << endl;
	cout << " range of randomGenerator = " << range << endl;

	for (i=0; i<10000000;i++) {
	    do {
		U1 = double((randomGenerator()) - min) / range;
		U2 = double((randomGenerator()) - min) / range;
		V1=2.0 * U1 -1.0;            /* V1=[-1,1] */
		V2=2.0 * U2 - 1.0;           /* V2=[-1,1] */
		S=V1 * V1 + V2 * V2;
	    }  while (S >=1.0);
	    polarFactor = sqrt(-2.0 * log(S) / S);
	    X = polarFactor * V1;
	    Y = polarFactor * V2;
	    sum += X + Y;
	    sqrSum += X*X + Y*Y;
	    absSum += fabs(X) + fabs(Y);

	}


	cout << "sum = " << sum << " Mean of " << i*2 << " samples is " << sum / double(i*2)  << endl;
	cout << "Average deviation of " << i*2 << " samples is " << absSum / double(i*2)  << endl;
	cout << "the variance of " << i*2 << " samples is " << sqrSum / double(i*2) << endl;
	cout << "the standard deviation is " << sqrt (sqrSum / double(i*2)) << endl;
	cout << "the sample variance of " << i*2 << " samples is " << sqrSum / double(i*2-1) << endl;
	cout << "the sample standard deviation is " << sqrt (sqrSum / double(i*2-1)) << endl;

	cout << "Size of status vector = " << static_cast<int>(openEV::GliderVarioStatus::NUM_ROWS)<<endl;

	return 0;
}
