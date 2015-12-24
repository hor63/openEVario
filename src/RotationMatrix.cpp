/**
 * \file RotationMatrix.cpp
 *
 *  Created on: Dec 22, 2015
 *      Author: hor
 */

#include "RotationMatrix.h"
#include "FastMath.h"

namespace openEV
{


RotationMatrix::~RotationMatrix ()
{
  // TODO Auto-generated destructor stub
}

/**
 * Calculates the rotation matrix. The matrix from world coordinates to plane coordinates is calculated only.
 *
 * Again the the angle definitions:
 * - Yaw angle 		\Psi 	= Heading
 * - Pitch angle 	\Theta	= Elevation
 * - Rollwinkel 	\Phi	= Bank angle
 *
 * Implementing the matrix according to the German Wikipedia  \ref https://de.wikipedia.org/wiki/Eulersche_Winkel#Drehfolgen_in_der_Fahrzeugtechnik
 *
 * \begin{align} M_{GNR} & =
 * \begin{pmatrix} 	\cos \Theta \cos \Psi 					& 	\cos \Theta \sin \Psi 					& -\sin \Theta
 * \\ 			\sin \Phi \sin \Theta \cos \Psi - \cos \Phi \sin \Psi 	&	\sin \Phi \sin \Theta \sin \Psi + \cos \Phi \cos \Psi 	& \sin \Phi \cos \Theta
 * \\ 			\cos \Phi \sin \Theta \cos \Psi + \sin \Phi \sin \Psi 	& \cos \Phi \sin \Theta \sin \Psi - \sin \Phi \cos \Psi 	& \cos \Phi \cos \Theta
 * \end{pmatrix} \end{align}
 *
 */
void
RotationMatrix::calculateRotationMatrixGloToPlane ()
{

  if (matrixGloToPlaneIsValid) {
      return;
  }

  // Calculate the sin and cos values from the angles beforehand because they occur multiple times in the matrix.
  FloatType sinPsi	= FastMath::fastSin(yaw);
  FloatType cosPsi	= FastMath::fastCos(yaw);
  FloatType sinTheta	= FastMath::fastSin(pitch);
  FloatType cosTheta	= FastMath::fastCos(pitch);
  FloatType sinPhi	= FastMath::fastSin(roll);
  FloatType cosPhi	= FastMath::fastCos(roll);


  matrixGloToPlane(0,0) = cosTheta * cosPsi;
  matrixGloToPlane(0,1) = cosTheta * sinPsi;
  matrixGloToPlane(0,2) = -sinTheta;

  matrixGloToPlane(1,0) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
  matrixGloToPlane(1,1) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
  matrixGloToPlane(1,2) = sinPhi * cosTheta;

  matrixGloToPlane(2,0) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
  matrixGloToPlane(2,1) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
  matrixGloToPlane(2,2) = cosPhi * cosTheta;

  matrixGloToPlaneIsValid = true;
  matrixPlaneToGloIsValid = false;

}

} // namespace openEV
