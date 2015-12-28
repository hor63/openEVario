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
  FloatType sinYaw	= FastMath::fastSin(yaw);
  FloatType cosYaw	= FastMath::fastCos(yaw);
  FloatType sinPitch	= FastMath::fastSin(pitch);
  FloatType cosPitch	= FastMath::fastCos(pitch);
  FloatType sinRoll	= FastMath::fastSin(roll);
  FloatType cosRoll	= FastMath::fastCos(roll);


  matrixGloToPlane(0,0) = cosPitch * cosYaw;
  matrixGloToPlane(0,1) = cosPitch * sinYaw;
  matrixGloToPlane(0,2) = -sinPitch;

  matrixGloToPlane(1,0) = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
  matrixGloToPlane(1,1) = sinRoll * sinPitch * sinYaw + cosRoll * cosYaw;
  matrixGloToPlane(1,2) = sinRoll * cosPitch;

  matrixGloToPlane(2,0) = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;
  matrixGloToPlane(2,1) = cosRoll * sinPitch * sinYaw - sinRoll * cosYaw;
  matrixGloToPlane(2,2) = cosRoll * cosPitch;

  matrixGloToPlaneIsValid = true;
  matrixPlaneToGloIsValid = false;

}

} // namespace openEV
