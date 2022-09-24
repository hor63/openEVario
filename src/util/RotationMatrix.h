/* RotationMatrix.h
 *
 *  Created on: Dec 22, 2015
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

#ifndef ROTATIONMATRIX_H_
#define ROTATIONMATRIX_H_

#include <Eigen/Dense>
#include "CommonDefs.h"


namespace openEV
{

/// \brief This vector type is used for all 3-dimensional representations of values in Cartesian coordinates
typedef Eigen::Matrix<FloatType, 3, 1> Vector3DType;
/// \brief The 3D rotation matrix to transfrom 3D vectors from one reference system to another.
///
/// Typical use is transform 3D values from word to plane coordinate systems and vice versa.
typedef Eigen::Matrix<FloatType, 3, 3> RotationMatrix3DType;

/**
 * \brief Rotation matrix to transform the global coordinate system into the plane's coordinate system.
 *
 * Rotation matrix to transform the global coordinate system into the plane's coordinate system
 * References are [Wikipedia: Vehicles and moving frames](https://en.wikipedia.org/wiki/Euler_angles#Vehicles_and_moving_frames).
 *
 * The matrix used is Tait-Bryan angles from
 * [Wikipedia: Euler angles - Rotation_matrix](https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix).
 * A IMHO more concise description is in German:
 * [Wikipedia: Eulersche Winkel - Roll-, Nick- und Gierwinkel: z-y-x-Konvention](https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y%E2%80%B2-x%E2%80%B3-Konvention).
 *
 * I am using the z-y'-x'' order as defined in DIN 9300 for aircraft rotational order to transfer world
 * coordinates into vehicle coordinates.
 * I am also using the world coordinate system definition according to DIN 9300:
 * - x = horizontal North.
 * - y = horizontal East
 * - z = perpendicular *down*; moving up decreases z, but on the other hand earth gravity is +9.81m/s&sup2; in z direction.
 *
 * I am using the common names of the angles. In the description you find the greek designations from DIN 9300.
 * \sa [Wikipedia: Eulersche Winkel - Roll-, Nick- und Gierwinkel](https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y%E2%80%B2-x%E2%80%B3-Konvention)
 */
class OEV_UTILS_PUBLIC RotationMatrix
{
public:

    /// Default constructor. Initialized all angles to 0. The rotation matrix is the Identity matrix.
    inline
    RotationMatrix () :
    yaw 	{ 0 },
    pitch	{ 0 },
    roll	{ 0 },
    // lazy calculations
    matrixGloToPlaneIsValid { true },
    matrixPlaneToGloIsValid { true }
    {
    	// Initially plane and word coordinates are identical.
        // this is so trivial that it virtually costs nothing,
        // compared to calculating cos and sin (0) later.
        matrixGloToPlane.setIdentity ();
        matrixPlaneToGlo.setIdentity ();
    }

    /// Constructor with initial angles. The matrix is not yet defined.
    inline
    RotationMatrix (FloatType yaw,
            FloatType pitch,
            FloatType roll) :
            yaw 	{ yaw },
            pitch	{ pitch },
            roll	{ roll },
            // lazy calculations
            matrixGloToPlaneIsValid { false },
            matrixPlaneToGloIsValid { false }

            {
            }

            virtual
            ~RotationMatrix ();

            /// set yaw angle &Psi;. Invalidates the matrix.
            inline void
            setYaw (FloatType yaw)
            {
                this->yaw = yaw;
                matrixGloToPlaneIsValid = false;
                matrixPlaneToGloIsValid = false;
            }

            inline FloatType getYaw() {return yaw;}

            /// set pitch angle &Theta;. Invalidates the matrix.
            inline void
            setPitch (FloatType pitch)
            {
                this->pitch = pitch;
                matrixGloToPlaneIsValid = false;
                matrixPlaneToGloIsValid = false;
            }

            inline FloatType getPitch() {return pitch;}

            /// set roll angle &Phi;. Invalidates the matrix.
            inline void
            setRoll (FloatType roll)
            {
                this->roll = roll;
                matrixGloToPlaneIsValid = false;
                matrixPlaneToGloIsValid = false;
            }

            inline FloatType getRoll() {return roll;}

            /**
             * Convert the plane vector into the world vector
             *
             * @param[in] planeVector The vector in plane coordinates
             * @param[out] worldVector The vector in world coordinates
             */
            void calcPlaneVectorToWorldVector (const Vector3DType& planeVector,Vector3DType& worldVector){
                calculateRotationMatrixPlaneToGlo ();
                worldVector = matrixPlaneToGlo * planeVector;
            }

            /**
             * Convert the world vector into the plane vector
             *
             * @param[in] worldVector The vector in world coordinates
             * @param[out] planeVector The vector in plane coordinates
             */
            void calcWorldVectorToPlaneVector (const Vector3DType& worldVector,Vector3DType& planeVector){
                calculateRotationMatrixGloToPlane ();
                planeVector = matrixGloToPlane * worldVector;
            }

            /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
            inline RotationMatrix3DType& getMatrixGloToPlane() {
                calculateRotationMatrixGloToPlane();
                return matrixGloToPlane;
            }
            /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
            inline RotationMatrix3DType& getMatrixPlaneToGlo() {
                calculateRotationMatrixPlaneToGlo();
                return matrixPlaneToGlo;
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
            /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
            RotationMatrix3DType matrixGloToPlane;
            /// The rotation matrix from the global(world) coordinate system to the plane coordinate system.
            RotationMatrix3DType matrixPlaneToGlo;

            // The angles.
            /// Yaw angle in deg. &Psi; in the norm DIN 9300. Also called \b Heading. Turning right hand around the z axis, i.e. in navigation direction
            FloatType yaw;
            /// Pitch angle in deg. &Theta; in the norm DIN 9300. Also called \b Elevation. Turning nose up around the y axis is positive.
            FloatType pitch;
            /// Roll angle in deg. &Phi; in the norm DIN 9300. Also called <b>Bank angle</b>.
            FloatType roll;

            bool matrixGloToPlaneIsValid;
            bool matrixPlaneToGloIsValid;

            /**
             * Calculates the rotation matrix. The matrix from world coordinates to plane coordinates is calculated only.
             *
             * Again the the angle definitions:
             * - Yaw angle 		&Psi; 	= Heading
             * - Pitch angle 	&Theta;	= Elevation
             * - Rollwinkel 	&Phi;	= Bank angle
             *
             * Implementing the matrix according to German
             * [Wikipedia: Eulersche Winkel - Roll-, Nick- und Gierwinkel: z-y-x-Konvention](https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y%E2%80%B2-x%E2%80%B3-Konvention)
             *
             * \f[
             * M_{GNR} =
             * \begin{pmatrix} 	\cos \Theta \cos \Psi 					& 	\cos \Theta \sin \Psi 					& -\sin \Theta
             * \\ 			\sin \Phi \sin \Theta \cos \Psi - \cos \Phi \sin \Psi 	&	sin \Phi sin \Theta \sin \Psi + \cos \Phi \cos \Psi 	& \sin \Phi \cos \Theta
             * \\ 			\cos \Phi \sin \Theta \cos \Psi + \sin \Phi \sin \Psi 	& \cos \Phi \sin \Theta \sin \Psi - \sin \Phi \cos \Psi 	& \cos \Phi \cos \Theta
             * \end{pmatrix}
             * \f]
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
                matrixPlaneToGloIsValid = true;
            }

};

} // namespace openEV

#endif /* ROTATIONMATRIX_H_ */
