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

GliderVarioStatus::GliderVarioStatus ()
{
  statusVector.setZero();

}

GliderVarioStatus::~GliderVarioStatus ()
{
  // TODO Auto-generated destructor stub
}

} // namespace openEV

std::ostream& operator <<(std::ostream &o, openEV::GliderVarioStatus &s) {

	o <<    " longitude   "
			" latitude    "
			" altMSL      "
			" yawAngle    "
			" pitchAngle  "
			" rollAngle   "
			" groundSpeedN"
			" groundSpeedE"
			" trueAirSpeed"
			" heading     "
			" rateOfSink  "
			" verticalSpee"
			" accelX      "
			" accelY      "
			" accelZ      "
			" rollRateX   "
			" pitchRateY  "
			" yawRateZ    "
			" gyroBiasX   "
			" gyroBiasY   "
			" gyroBiasZ   "
			" windSpeedNor"
			" windSpeedEas"
			" thermalSpeed" << std::endl;

	o << std::fixed;
	o.precision(7);
	o.fill('_');

	o.precision(7);
	o.width(13);
	o << s.longitude;
	o.precision(7);
	o.width(13);
	o << s.latitude;
	o.precision(7);
	o.width(13);
	o << s.altMSL;
	o.precision(7);
	o.width(13);
	o << s.yawAngle;
	o.precision(7);
	o.width(13);
	o << s.pitchAngle;
	o.precision(7);
	o.width(13);
	o << s.rollAngle;
	o.precision(7);
	o.width(13);
	o << s.groundSpeedNorth;
	o.precision(7);
	o.width(13);
	o << s.groundSpeedEast;
	o.precision(7);
	o.width(13);
	o << s.trueAirSpeed;
	o.precision(7);
	o.width(13);
	o << s.heading;
	o.precision(7);
	o.width(13);
	o << s.rateOfSink;
	o.precision(7);
	o.width(13);
	o << s.verticalSpeed;
	o.precision(7);
	o.width(13);
	o << s.accelX;
	o.precision(7);
	o.width(13);
	o << s.accelY;
	o.precision(7);
	o.width(13);
	o << s.accelZ;
	o.precision(7);
	o.width(13);
	o << s.rollRateX;
	o.precision(7);
	o.width(13);
	o << s.pitchRateY;
	o.precision(7);
	o.width(13);
	o << s.yawRateZ;
	o.precision(7);
	o.width(13);
	o << s.gyroBiasX;
	o.precision(7);
	o.width(13);
	o << s.gyroBiasY;
	o.precision(7);
	o.width(13);
	o << s.gyroBiasZ;
	o.precision(7);
	o.width(13);
	o << s.windSpeedNorth;
	o.precision(7);
	o.width(13);
	o << s.windSpeedEast;
	o.precision(7);
	o.width(13);
	o << s.thermalSpeed;
	o << std::endl;

	return o;
}
