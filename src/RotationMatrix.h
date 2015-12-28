/** \file RotationMatrix.h
 *
 *  Created on: Dec 22, 2015
 *      Author: hor
 */

#ifndef ROTATIONMATRIX_H_
#define ROTATIONMATRIX_H_

#include "GliderVarioStatus.h"


namespace openEV
{
/***
 * \class RotationMatrix
 * \brief Rotation matrix to transform the global coordinate system into the plane's coordinate system.
 * Rotation matrix to transform the global coordinate system into the plane's coordinate system
 * References are \ref https://en.wikipedia.org/wiki/Euler_angles#Vehicles_and_moving_frames. The matrix used is Tait-Bryan angles from
 * \ref https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix.
 * A IMHO more concise description is in German \ref https://de.wikipedia.org/wiki/Eulersche_Winkel#Drehfolgen_in_der_Fahrzeugtechnik
 * I am using the Y-Y-Z order as defined in DIN 9300 for aircraft rotational order.
 * I am also using the coordinate system definition according to DIN 9300:
 * - x = horizontal North.
 * - y = horizontal East
 * - z = perpendicular *down*; moving up decreases z, but on the other hand earth gravity is +9.81m/s^2 in z direction.
 * I am using the common names of the angles. In the description you find the greek designations from DIN 9300 (see the German Wikipedia)
 */
class RotationMatrix
{
public:
  typedef Eigen::Matrix<FloatType, 3, 3> RotationMatrixType;

  /// Default constructor. Initialized all angles to 0. The rotation matrix is the Identity matrix.
  inline
  RotationMatrix () :
      yaw 	{ 0 },
      roll	{ 0 },
      pitch	{ 0 },
      // lazy calculations
      matrixGloToPlaneIsValid { false },
      matrixPlaneToGloIsValid { false }
  {
    // this is so trivial that it virtually costs nothing,
    // compared to calculating cos and sin (0) later.
    matrixGloToPlane.setIdentity ();
  }

  /// Constructor with initial angles. The matrix is not yet defined.
  inline
  RotationMatrix (FloatType yaw,
		  FloatType pitch,
		  FloatType roll) :
      yaw 	{ yaw },
      roll	{ roll },
      pitch	{ pitch },
      // lazy calculations
      matrixGloToPlaneIsValid { false },
      matrixPlaneToGloIsValid { false }

  {
  }

  virtual
  ~RotationMatrix ();

  /// set yaw angle \Psi. Invalidates the matrix.
  void
  setYaw (FloatType yaw)
  {
    this->yaw = yaw;
    matrixGloToPlaneIsValid = false;
    matrixPlaneToGloIsValid = false;
  }

  /// set pitch angle \Theta. Invalidates the matrix.
  void
  setPitch (FloatType pitch)
  {
    this->pitch = pitch;
    matrixGloToPlaneIsValid = false;
    matrixPlaneToGloIsValid = false;
  }

  /// set roll angle \Phi. Invalidates the matrix.
  void
  setRoll (FloatType roll)
  {
    this->roll = roll;
    matrixGloToPlaneIsValid = false;
    matrixPlaneToGloIsValid = false;
  }

  /**
   * Convert the plane vector into the world vector
   *
   * @param planeVector[in] The vector in plane coordinates
   * @param worldVector[out] The vector in world coordinates
   */
  void calcPlaneVectorToWorldVector (const Vector3DType& planeVector,Vector3DType& worldVector){
    calculateRotationMatrixPlaneToGlo ();
    worldVector = matrixPlaneToGloIsValid * planeVector;
  }

  /**
   * Convert the world vector into the plane vector
   *
   * @param worldVector[in] The vector in world coordinates
   * @param planeVector[out] The vector in plane coordinates
   */
  void calcWorldVectorToPlaneVector (const Vector3DType& worldVector,Vector3DType& planeVector){
    calculateRotationMatrixGloToPlane ();
    planeVector = matrixGloToPlane * worldVector;
  }

  /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
  inline RotationMatrixType& getMatrixGloToPlane() {
    calculateRotationMatrixGloToPlane();
    return matrixGloToPlane;
  }
  /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
  inline RotationMatrixType& getMatrixPlaneToGlo() {
    calculateRotationMatrixPlaneToGlo();
    return matrixPlaneToGlo;
  }


protected:
  /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
  RotationMatrixType matrixGloToPlane;
  /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
  RotationMatrixType matrixPlaneToGlo;

  bool matrixGloToPlaneIsValid;
  bool matrixPlaneToGloIsValid;

  // The angles.
  /// Yaw angle in deg. \Psi in the norm DIN 9300. Also called \b Heading. Turning right hand around the z axis, i.e. in navigation direction
  FloatType yaw;
  /// Pitch angle in deg. \Theta in the norm DIN 9300. Also called \b Elevation. Turning nose up around the y axis is positive.
  FloatType pitch;
  /// Roll angle in deg. \Phi in the norm DIN 9300. Also called <b>Bank angle</b>.
  FloatType roll;


  /**
   * Calculates the rotation matrix global to plane is calculated.
   */
  void
  calculateRotationMatrixGloToPlane ();

  /**
   * Calculate the rotation matrix plane to global. Do this by transposing the global to plane matrix.
   */
  inline void
  calculateRotationMatrixPlaneToGlo () {
    if (matrixPlaneToGloIsValid) {
	return;
    }

    calculateRotationMatrixGloToPlane();

    matrixPlaneToGlo = matrixGloToPlane.transpose();

  }

};

} // namespace openEV

#endif /* ROTATIONMATRIX_H_ */
