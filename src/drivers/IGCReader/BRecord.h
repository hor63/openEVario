/*
 * BRecord.h
 *
 *  Created on: Oct 3, 2018
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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

#ifndef DRIVERS_IGCREADER_BRECORD_H_
#define DRIVERS_IGCREADER_BRECORD_H_

#include "OEVCommon.h"

namespace openEV {

	typedef struct {
		OEVDuration	timeSinceStart;	///< Duration since the first B-record
		double		latitude; 		///< Latitude in degrees. -9999.0 means the record is undefined
		double 		longitude; 		///< Latitude in degrees. -9999.0 means the record is undefined
		double 		posAccuracy; 	///< Accuracy of the position in m
		double 		altGPS; 		///< GPS altitude in m. -9999.0 means the record is undefined
		double 		altGPSAccuracy; ///< Accuracy of the GPS altitude
		double 		altBaro; 		///< Barometric altitude, calculated in the recorder from a pressure sensor.
		double 		pressure; 		///< Pressure in hPa (mbar) calculated from the \ref altBaro according to the standard atmosphere model
		bool		gpsIsValid; 		///< GPS position and altitude are valid.
	} BRecord;


} // namespace openEV


#endif /* DRIVERS_IGCREADER_BRECORD_H_ */
