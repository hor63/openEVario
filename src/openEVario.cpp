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

#include <iostream>
#include <random>
#include <math.h>
#include <sys/time.h>

#include "GliderVarioStatus.h"
#include "GliderVarioTransitionMatrix.h"
#include "RotationMatrix.h"
#include "FastMath.h"
#include "GliderVarioMeasurementVector.h"
#include "GliderVarioMeasurementMatrix.h"
#include "RotMatrixConversion.h"

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
  double x,y, tanRes,fastTanRes,maxDev = 0.0;
  double range;
  double sqrSum = 0.0,sum = 0.0,absSum =0.0;
  GliderVarioStatus ovStatus;
  GliderVarioTransitionMatrix ovTransition;
  GliderVarioMeasurementVector ovMeasurement;
  GliderVarioMeasurementVector::MeasureVectorType ovMeasureVector;
  GliderVarioMeasurementMatrix ovMeasureMatrix;
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
		  tanRes = atan2(y,x) * FastMath::radToDeg;
		  fastTanRes = FastMath::fastATan2(y,x);
		  if (fastTanRes > 180.0) {
			  fastTanRes = -360 + fastTanRes;
		  }
		  if (fabs(tanRes-fastTanRes) > fabs(maxDev) && fabs(tanRes-fastTanRes) < 359.0) {
			  maxDev = tanRes-fastTanRes;
		  }
	  }
	  cout << "x = " << x << ", y = " << y << ", tanRes = " << tanRes << ", fastTanRes = " << fastTanRes << endl;
	  cout << "maxDev = " << maxDev << endl;

  }


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
  cout << "The rotation matrix of the vector is " << endl << rotMag.getMatrixPlaneToGlo() << endl;

  cout << "calculate the rotation matrix from the vectors" << endl;
  RotationMatrix3DType magRotMatrix;
  magRotMatrix.setZero();
  RotMatrixConversion::vectors2RotMatrix(uniVector,magField,magRotMatrix);
  Vector3DType rotMag1 (0,0,0);
  RotMatrixConversion::rotMatrix2RotVector(magRotMatrix,rotMag1);
  cout << "The reverse calculated rotation matrix is" << endl << magRotMatrix << endl;
  cout << "The calculated Euler angles are " << endl << rotMag1 << endl;

  cout << endl << "transpose the rotation matrix, and calc the Euler angles from that" << endl;
  magRotMatrix.transposeInPlace();
  rotMag1.setZero();
  RotMatrixConversion::rotMatrix2RotVector(magRotMatrix,rotMag1);
  cout << "The transposed rotation matrix is" << endl << magRotMatrix << endl;
  cout << "The calculated Euler angles are " << endl << rotMag1 << endl;

  return (0);

  cout << endl <<
		  "-----------------------" << endl;
  cout << "test the transition matrix" << endl;



  // Initialize the status vector
  ovStatus.longitude = 55.0f;
  ovStatus.latitude = 10.0f;
  ovStatus.altMSL = 500.0f;
  // ovStatus.yawAngle = 30.0f;
  ovStatus.pitchAngle = 0.0f;
  ovStatus.rollAngle = 20.0f;

  // Speeds and directions
  ovStatus.groundSpeedNorth = 15.0f;
  ovStatus.groundSpeedEast = 25.0f;
  ovStatus.trueAirSpeed = 30.0f;
  ovStatus.heading = 30.0f;
  ovStatus.rateOfSink = 0.5f;
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
	  RotationMatrix rotMat1 (ovStatus.heading,ovStatus.pitchAngle,ovStatus.rollAngle);

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

  ovTransition.updateStatus(ovStatus,ovStatus,0.1f);
  cout << "-- ovTransition = " << endl << ovTransition.getTransitionMatrix() << endl;
  cout << "-- ovStatus after 0.1 sec = " << endl << ovStatus;
  for (i = 0; i< 20; i++) {
	  ovTransition.updateStatus(ovStatus,ovStatus,0.1f);
	  cout << ovStatus;

  }

  cout << endl <<
		  "-----------------------" << endl;
  cout << "test normalizing angles" << endl;
  cout << "normalizing yaw" << endl;
  for (i = -720; i <= 720; i+=360) {
	  ovStatus.pitchAngle = 5;
	  ovStatus.rollAngle = -20;
	  ovStatus.heading = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.pitchAngle = 5;
	  ovStatus.rollAngle = -20;
	  ovStatus.heading = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.pitchAngle = 5;
	  ovStatus.rollAngle = -20;
	  ovStatus.heading = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;
  }

  cout << "normalizing roll" << endl;
  for (i = -360-180; i <= 360+180; i+=180) {
	  ovStatus.pitchAngle = 5;
	  ovStatus.heading = 30;
	  ovStatus.rollAngle = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.pitchAngle = 5;
	  ovStatus.heading = 30;
	  ovStatus.rollAngle = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.pitchAngle = 5;
	  ovStatus.heading = 30;
	  ovStatus.rollAngle = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;
  }

  cout << "normalizing Pitch" << endl;
  for (i = -360-180; i <= 360+180; i+=90) {
	  ovStatus.rollAngle = 20;
	  ovStatus.heading = 30;
	  ovStatus.pitchAngle = static_cast<FloatType>(i) - 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.rollAngle = 20;
	  ovStatus.heading = 30;
	  ovStatus.pitchAngle = static_cast<FloatType>(i);
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;

	  ovStatus.rollAngle = 20;
	  ovStatus.heading = 30;
	  ovStatus.pitchAngle = static_cast<FloatType>(i) + 1.0f;
	  cout << "Before: Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl;
	  ovStatus.normalizeAngles();
	  cout << "After:  Yaw,Pitch,Roll = " << ovStatus.heading << ", " << ovStatus.pitchAngle << ", " << ovStatus.rollAngle << endl << endl;
  }

  cout << "calculating measurement vector from status" << endl;
  ovMeasureMatrix.calcMeasurementMatrix(0.1f,ovStatus);
  ovMeasureVector = ovMeasureMatrix.getMeasureMatrix() * ovStatus.getStatusVector();

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
