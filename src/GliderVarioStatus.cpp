/* Implementation of the openEVario status class
 *
 * GliderVarioStatus.cpp
 *
 *  Created on: Dec 8, 2015
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

#include "GliderVarioStatus.h"
#include <iomanip>

namespace openEV
{

FloatType MAG_INCLINATION = 67.0f;

GliderVarioStatus::GliderVarioStatus ()
{
  statusVector_x.setZero();
  systemNoiseCovariance_Q.setZero();
  errorCovariance_P.setZero();

  statusVector_x(STATUS_IND_GRAVITY)= GRAVITY;

}

GliderVarioStatus::~GliderVarioStatus ()
{

}

void GliderVarioStatus::normalizeAngles() {

	  if (pitchAngle < -90.0f) {
		  if (pitchAngle < -360.0f) {
			  do {
				  pitchAngle += 360.0f;
			  } while (pitchAngle < -360.0f);
			  // pitch angle can now be anywhere between 0 and -360
			  // Therefore do it again recursively.
			  return normalizeAngles();
		  } // if (pitchAngle < -360.0f)
		  if (pitchAngle <= -180.0f) {
			  // I am at an upward pointing angle
			  pitchAngle += 360.0f;
		  } else { // if (pitchAngle <= -180.0f)
			  /* I am between -90 and -180 deg, i.e. positive looping beyond perpendicular
			   * i.e. I am flipping direction and roll, and bring pitch back into the 1st quadrant
			   */
			  pitchAngle = -180.0f - pitchAngle;
			  rollAngle += 180.0f;
			  heading += 180.0f;
		  } // if (pitchAngle <= -180.0f)

	  } // if (pitchAngle < -90.0f)

	  if (pitchAngle > 90.0f) {
		  if (pitchAngle >= 360.0f) {
			  do {
				  pitchAngle -= 360.0f;
			  } while (pitchAngle >= 360.0f);
			  // pitch angle can now be anywhere between 0 and +360
			  // Therefore do it again recursively.
			  return normalizeAngles();
		  } // if (pitchAngle > 360.0f)
		  if (pitchAngle > 180.0f) {
			  pitchAngle -= 360.0f;
			  /* Pitch angle is negative now, and between -0 and -180. I deal with that above. Ugh.
			   * So I call myself recursively
			   */
			  return normalizeAngles();
		  } else { // if (pitchAngle > 180.0f)
			  /* I am between 90 and 180 deg, i.e. looping beyond perpendicular
			   * i.e. I am flipping direction and roll, and bring pitch back into the 1st quadrant
			   */
			  pitchAngle = 180.0f - pitchAngle;
			  rollAngle += 180.0f;
			  heading += 180.0f;
		  } // if (pitchAngle > 180.0f)
	  } // if (pitchAngle > 90.0f)

	  // roll and yaw are easier: They are independent from the others :)
	  if (rollAngle < -180.0f) {

		  if (rollAngle >= -360.0f) {
			  // This is the normal case
			  rollAngle += 360.0f;
		  } else { // if (rollAngle >= -360.0f)
			  // This should actually happen very rarely if at all...
			  do {
			  rollAngle += 360.0f;
			  } while (rollAngle < -360.0f);

			  if (rollAngle < -180.0f) {
				  rollAngle += 360.0f;
			  }
		  } // if (rollAngle >= -360.0f)
	  } // if (rollAngle < -180.0f)
	  if (rollAngle >= 180.0f) {

		  if (rollAngle <= 360.0f) {
			  // This is the normal case
			  rollAngle -= 360.0f;
		  } else { // if (rollAngle >= -360.0f)
			  // This should actually happen very rarely if at all...
			  do {
			  rollAngle -= 360.0f;
			  } while (rollAngle > 360.0f);

			  if (rollAngle >= 180.0f) {
				  rollAngle -= 360.0f;
			  }
		  } // if (rollAngle >= -360.0f)
	  } // if (rollAngle < -180.0f)

	  while (heading < 0.0f) {
		  heading += 360.0f;
	  }
	  while (heading >= 360.0f) {
		  heading -= 360.0f;
	  }

}


} // namespace openEV

std::ostream& operator <<(std::ostream &o, openEV::GliderVarioStatus &s) {

	o <<
			" gravity     "
			" longitude   "
			" latitude    "
			" altMSL      "
			" heading     "
			" pitchAngle  "
			" rollAngle   "
			" groundSpeedN"
			" groundSpeedE"
			" trueAirSpeed"
			" rateOfSink  "
			" verticalSpd "
			" thermalSpeed"
			" accelHeading"
			" accelCross  "
			" accelVertica"
			" rollRateX   "
			" pitchRateY  "
			" yawRateZ    "
			" yawRateGloZ "
			" gyroBiasX   "
			" gyroBiasY   "
			" gyroBiasZ   "
			" magDeclinati"
			" magInclinati"
			" compDeviatiX"
			" compDeviatiY"
			" compDeviatiZ"
			" windSpeedNor"
			" windSpeedEas"
			" QFF         "
			" lastPressure"
		<< std::endl;

	o << std::fixed;
	o.precision(7);
	o.fill('_');

	o.precision(7); o.width(13); o << s.gravity     ;
	o.precision(7); o.width(13); o << s.longitude   ;
	o.precision(7); o.width(13); o << s.latitude    ;
	o.precision(7); o.width(13); o << s.altMSL      ;
	o.precision(7); o.width(13); o << s.heading     ;
	o.precision(7); o.width(13); o << s.pitchAngle  ;
	o.precision(7); o.width(13); o << s.rollAngle   ;
	o.precision(7); o.width(13); o << s.groundSpeedNorth;
	o.precision(7); o.width(13); o << s.groundSpeedEast;
	o.precision(7); o.width(13); o << s.trueAirSpeed;
	o.precision(7); o.width(13); o << s.rateOfSink  ;
	o.precision(7); o.width(13); o << s.verticalSpeed;
	o.precision(7); o.width(13); o << s.thermalSpeed;
	o.precision(7); o.width(13); o << s.accelHeading      ;
	o.precision(7); o.width(13); o << s.accelCross      ;
	o.precision(7); o.width(13); o << s.accelVertical      ;
	o.precision(7); o.width(13); o << s.rollRateX   ;
	o.precision(7); o.width(13); o << s.pitchRateY  ;
	o.precision(7); o.width(13); o << s.yawRateZ    ;
	o.precision(7); o.width(13); o << s.yawRateGloZ ;
	o.precision(7); o.width(13); o << s.gyroBiasX   ;
	o.precision(7); o.width(13); o << s.gyroBiasY   ;
	o.precision(7); o.width(13); o << s.gyroBiasZ   ;
	o.precision(7); o.width(13); o << s.magneticDeclination;
	o.precision(7); o.width(13); o << s.magneticInclination;
	o.precision(7); o.width(13); o << s.compassDeviationX;
	o.precision(7); o.width(13); o << s.compassDeviationY;
	o.precision(7); o.width(13); o << s.compassDeviationZ;
	o.precision(7); o.width(13); o << s.windSpeedNorth;
	o.precision(7); o.width(13); o << s.windSpeedEast;
	o.precision(7); o.width(13); o << s.qff;
	o.precision(7); o.width(13); o << s.lastPressure;

	o << std::endl;

	return o;
}
