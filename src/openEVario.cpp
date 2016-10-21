/*
 *  openEVario.cpp
 *
 *  start module. Contains the main() function. Startup, initialization, and start of the main loop.
 *
 *  Created on: Dec 08, 2015
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not, write to the Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <GliderVarioMeasurementUpdater.h>
#include <iostream>
#include <random>
#include <math.h>
#include <sys/time.h>

#include "GliderVarioStatus.h"
#include "GliderVarioTransitionMatrix.h"
#include "RotationMatrix.h"
#include "FastMath.h"
#include "GliderVarioMeasurementVector.h"

using namespace std;
using namespace openEV;

mt19937 randomGenerator;

FloatType x = 0;

/**
 * \brief The one and only main() function
 * Startup and initialization. Demonization if required. Entry into the main processing loop.
 * @param argc
 * @param argv
 * @return
 *
 * TODO remove all the test code, and replace it by real application code.
 */
int main (int argc, char *argv[]) {
  double U1,U2,V1,V2,S,polarFactor,X,Y	;
  mt19937::result_type min;
  double x,y, res,fastRes,maxDev = 0.0,maxDevAt = -100.0;
  double range;
  double sqrSum = 0.0,sum = 0.0,absSum =0.0;
  GliderVarioStatus ovStatus1, ovStatus2;
  GliderVarioStatus *ovStatusOld = &ovStatus1;
  GliderVarioStatus *ovStatusNew = &ovStatus2;

  GliderVarioTransitionMatrix ovTransition;
  GliderVarioMeasurementVector ovMeasurement;
  GliderVarioMeasurementVector::MeasureVectorType ovMeasureVector;
  GliderVarioMeasurementUpdater ovMeasureMatrix;
int i;



  // Test of fastMath.
  FloatType j;
  struct timeval tv1,tv2,tv3,tv4;

  cout << "-----------------------" << endl;
  cout << "Test of fastMath." << endl;

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

  cout << "-------------------" << endl
	   << "Test of fast arc tangent " << endl;
  for (x = -1 ; x<=1 ; x+=0.2) {
	  for (y = -1 ; y<=1 ; y+=0.2) {
		  res = atan2(y,x) * FastMath::radToDeg;
		  fastRes = FastMath::fastATan2(y,x);
		  if (fastRes > 180.0) {
			  fastRes = -360 + fastRes;
		  }
		  if (fabs(res-fastRes) > fabs(maxDev) && fabs(res-fastRes) < 359.0) {
			  maxDev = res-fastRes;
		  }
	  }
	  cout << "x = " << x << ", y = " << y << ", tanRes = " << res << ", fastRes = " << fastRes << endl;
  }
  cout << "maxDev = " << maxDev << endl;

  cout << "-------------------" << endl
	   << "Test of fast arc sin " << endl;
  maxDev = 0.0;
  for (x = -100 ; x<=100 ; x++) {
	  res = asin(x/100.0) * FastMath::radToDeg;
	  fastRes = FastMath::fastASin(x/100.0);
	  if (fabs(res-fastRes)>fabs(maxDev)) {
		  maxDev = res-fastRes;
		  maxDevAt = x/100;
	  }
  }
  cout << "maxDev = " << maxDev << " at " << maxDevAt << endl;

  cout << "-------------------" << endl
	   << "Test of fast arc cos " << endl;
  maxDev = 0.0;
  for (x = -100 ; x<=100 ; x++) {
	  res = acos(x/100.0) * FastMath::radToDeg;
	  fastRes = FastMath::fastACos(x/100.0);
	  if (fabs(res-fastRes)>fabs(maxDev)) {
		  maxDev = res-fastRes;
		  maxDevAt = x/100;
	  }
  }
  cout << "maxDev = " << maxDev << " at " << maxDevAt << endl;




  cout << "-----------------------" << endl;
  cout << "Test of rotation matrix" << endl;
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

  cout << endl <<
		  "-----------------------" << endl;
  cout << "Test conversion of 3d vectors into rotation matrixes, and into rotation vectors" << endl;
  cout << "Crucial to convert magnetic vector measurements into rotation vectors for attitude and direction" << endl;

  RotationMatrix rotMag (-5,-70,0); // magnetic field 5 deg West, 70deg inclination (realistic in N-Germany)
  Vector3DType uniVector (1,0,0), magField (0,0,0);
  rotMag.calcPlaneVectorToWorldVector(uniVector,magField);
  cout << "Magnetic field vector at 5Deg W, 70Deg incl. = " << endl << magField << endl;
  cout << "length of magnetic field vector is " << sqrtf(magField(0)*magField(0) + magField(1)*magField(1) + magField(2)*magField(2)) << endl;
  cout << "The rotation matrix of the vector is " << endl << rotMag.getMatrixGloToPlane() << endl;

  RotationMatrix rotPlane (60,0,0);


  cout << endl <<
		  "-----------------------" << endl;
  cout << "test the transition matrix" << endl;



  // Initialize the status vector
  ovStatusOld->longitude = 55.0f;
  ovStatusOld->latitude = 10.0f;
  ovStatusOld->altMSL = 500.0f;
  // ovStatusOld->yawAngle = 30.0f;
  ovStatusOld->pitchAngle = 0.0f;
  ovStatusOld->rollAngle = 20.0f;

  // Speeds and directions
  ovStatusOld->groundSpeedNorth = 15.0f;
  ovStatusOld->groundSpeedEast = 25.0f;
  ovStatusOld->trueAirSpeed = 30.0f;
  ovStatusOld->heading = 30.0f;
  ovStatusOld->rateOfSink = 0.5f;
  ovStatusOld->verticalSpeed = -2.0;

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // If the plane is pitched up an acceleration on the X axis would speed the plane upward, not forward.
  ovStatusOld->accelX = 0.0f;
  ovStatusOld->accelY = 0.0f;
  ovStatusOld->accelZ = -9.81/FastMath::fastCos(ovStatusOld->rollAngle);

  // Turn rates in reference to the body coordinate system
  ovStatusOld->rollRateX = 0.0f;
  ovStatusOld->pitchRateY = 0.0f;
  {   // intermediate calculation to estimate the turn radius, and the resulting turn rate.
	  FloatType angularAccel = 9.81 * FastMath::fastSin(ovStatusOld->rollAngle)/FastMath::fastCos(ovStatusOld->rollAngle);
	  // circular radius and and acceleration are defined as:
	  // a = v^2 / r, i.e. r = v^2 / a
	  FloatType radius = ovStatusOld->trueAirSpeed*ovStatusOld->trueAirSpeed/angularAccel;
	  // the circumfence of the circle is 2PI*r, with the speed the time for a full 360 deg circle is therefore
	  // t = 2PIr/TAS
	  FloatType timeFullCircle = 2*M_PI*radius/ovStatusOld->trueAirSpeed;
	  RotationMatrix rotMat1 (ovStatusOld->heading,ovStatusOld->pitchAngle,ovStatusOld->rollAngle);

	  ovStatusOld->yawRateZ = 360/timeFullCircle;

	  Vector3DType worldRot (ovStatusOld->rollRateX,ovStatusOld->pitchRateY,ovStatusOld->yawRateZ);
	  Vector3DType planeRot;

	  rotMat1.calcWorldVectorToPlaneVector(worldRot,planeRot);

	  cout << " -- yaw rate calculation ------------" << endl;
	  cout << " angularAccel      = " << angularAccel << endl;
	  cout << " radius            = " << radius << endl;
	  cout << " timeFullCircle    = " << timeFullCircle << endl;
	  cout << " ovStatusOld->yawRateZ = " << ovStatusOld->yawRateZ << endl;
	  cout << " World rotation rate vector = " << endl << worldRot << endl;
	  cout << " Plane rotation rate vector = " << endl << planeRot << endl;

	  ovStatusOld->rollRateX = planeRot(0);
	  ovStatusOld->pitchRateY = planeRot(1);
	  ovStatusOld->yawRateZ   = planeRot(2);
  }

  // Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
  ovStatusOld->gyroBiasX = 0.0f;
  ovStatusOld->gyroBiasY = 0.0f;
  ovStatusOld->gyroBiasZ = 0.0f;
  ovStatusOld->windSpeedNorth = 0.0f;
  ovStatusOld->windSpeedEast = 0.0f;
  ovStatusOld->thermalSpeed = 0.0f;

  cout << "-- ovStatus T = " << endl << *ovStatusOld;


  // initialize the error covariance and the system noise covariance
  for ( i=0 ; i < GliderVarioStatus::STATUS_NUM_ROWS ; i++) {
	  ovStatus1.getErrorCovariance_P()(i,i) = 1.0f;
	  ovStatus2.getErrorCovariance_P()(i,i) = 1.0f;
	  ovStatus1.getSystemNoiseCovariance_Q()(i,i) = 0.1f;
	  ovStatus2.getSystemNoiseCovariance_Q()(i,i) = 0.1f;
  }

  // Gravity has a bit of variance in the error covariance, but none in the system noise. So it should not increase.
  ovStatus1.getSystemNoiseCovariance_Q()(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.0f;
  ovStatus1.getErrorCovariance_P()(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.01f;
  ovStatus2.getSystemNoiseCovariance_Q()(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.0f;
  ovStatus2.getErrorCovariance_P()(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.01f;

  ovTransition.updateStatus(*ovStatusOld,*ovStatusNew,0.1f);
  cout << "-- ovTransition = " << endl << ovTransition.getTransitionMatrix() << endl;
  cout << "-- ovStatus after 0.1 sec = " << endl << *ovStatusNew << endl;
  cout << "-- Error Covariance = " << endl << ovStatusNew->getErrorCovariance_P() << endl;

  GliderVarioStatus *tmp = ovStatusOld;
  ovStatusOld=ovStatusNew;
  ovStatusNew = tmp;

  for (i = 0; i< 20; i++) {
	  ovTransition.updateStatus(*ovStatusOld,*ovStatusNew,0.1f);
	  cout << *ovStatusNew;

	  tmp = ovStatusOld;
	  ovStatusOld=ovStatusNew;
	  ovStatusNew = tmp;
  }

  cout << "-- Error Covariance after 21 iterations = " << endl << ovStatusNew->getErrorCovariance_P() << endl;

  cout << endl <<
		  "-----------------------" << endl;
  cout << "test normalizing angles" << endl;
  cout << "normalizing yaw" << endl;
  for (i = -720; i <= 720; i+=360) {
	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->rollAngle = -20;
	  ovStatusOld->heading = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->rollAngle = -20;
	  ovStatusOld->heading = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->rollAngle = -20;
	  ovStatusOld->heading = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
  }

  cout << "normalizing roll" << endl;
  for (i = -360-180; i <= 360+180; i+=180) {
	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->heading = 30;
	  ovStatusOld->rollAngle = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->heading = 30;
	  ovStatusOld->rollAngle = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->pitchAngle = 5;
	  ovStatusOld->heading = 30;
	  ovStatusOld->rollAngle = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
  }

  cout << "normalizing Pitch" << endl;
  for (i = -360-180; i <= 360+180; i+=90) {
	  ovStatusOld->rollAngle = 20;
	  ovStatusOld->heading = 30;
	  ovStatusOld->pitchAngle = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->rollAngle = 20;
	  ovStatusOld->heading = 30;
	  ovStatusOld->pitchAngle = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

	  ovStatusOld->rollAngle = 20;
	  ovStatusOld->heading = 30;
	  ovStatusOld->pitchAngle = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
	  ovStatusOld->normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
  }


  cout << endl <<
		  "-----------------------" << endl;
  cout << "test the random generator" << endl;

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

	cout << "Size of status vector = " << static_cast<int>(openEV::GliderVarioStatus::STATUS_NUM_ROWS)<<endl;

	return 0;
}
