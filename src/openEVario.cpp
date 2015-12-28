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
  cout << "End test sin" << endl;
  cout << "36000000 sin calls took " << double(tv2.tv_sec-tv1.tv_sec)+double(tv2.tv_usec-tv1.tv_usec)/1000000000 << endl;
  gettimeofday(&tv3,NULL);
  for (i=0;i<100000;i++) {
      for (j=0.0;j<=360;j+=1.0){
	  x = FastMath::fastSin(j);
      }
  }
  gettimeofday(&tv4,NULL);
  cout << "End test fastSin" << endl;
  cout << "36000000 fastSin calls took " << double(tv4.tv_sec-tv3.tv_sec)+double(tv4.tv_usec-tv3.tv_usec)/1000000000 << endl;
  cout << endl;

  // Test of rotation matrix
  RotationMatrix rotMat (20.0f,0.0f,-30.0f);
  Vector3DType worldRot (0.0f,0.0f,-18.0f);
  Vector3DType planeRot;
  rotMat.calcWorldVectorToPlaneVector(worldRot,planeRot);
  cout << "-----------" << endl;
  cout << "rotMatWorldToPlane = " << endl;
  cout << rotMat.getMatrixGloToPlane() << endl;
  cout << "-----------" << endl;
  cout << "worldRot = " << endl;
  cout << worldRot << endl;
  cout << "-----------" << endl;
  cout << "planeRot = " << endl;
  cout << planeRot << endl;

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
