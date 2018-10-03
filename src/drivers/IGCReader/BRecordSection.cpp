/*
 * BRecordSection.cpp
 *
 *  Created on: Sep 3, 2018
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <string.h>

#include "BRecordSection.h"

namespace openEV {

BRecordSectionBase::~BRecordSectionBase() {

}

BRecordSectionStd::~BRecordSectionStd() {

}

void BRecordSectionStd::updateFilter (
		GliderVarioMainPriv &varioMain,
		char *const recordString,
		int recordLen
		) {

	double degLat = strToInt(recordString+latDegPos,latDegLen);
	double min = strToInt(recordString+latMinPos,latMinLen) / 1000.0;
	/// Accuracy one sigma in m.
	int accuracy = 2;
	int vertAccuracy = 10;

	if (accuracyLen > 0 && accuracyPos > 0) {
		accuracy = strToInt(recordString + accuracyPos,accuracyLen);
	}

	degLat += min / 60.0;


	{
		GliderVarioMainPriv::LockedCurrentStatus currentStatus(varioMain);
		GliderVarioMeasurementUpdater::GPSLatitudeUpd(degLat,FloatType(accuracy*accuracy), *currentStatus.getMeasurementVector(),*currentStatus.getCurrentStatus());

	}

}

void BRecordSectionStd::processIRecord(char* const recordString,
		int recordLen) {

	/*
	 * The I-Record looks like this:
	 * I	Nu	StEnCod	StEnCod ... CRLF
	 * 0	12	3456789	0123456
	 * 0				1
	 *
	 * with
	 * I	Literal character 'I'
	 * Nu	Number of records
	 * St	Start Byte
	 * En	End byte
	 * Cod	Three-character code of the field, e.g. "FXA"
	 */


	int numRecords = strToInt(recordString + 1,2);

	// termination condition checks that a full field of 7 characters is still available in the string
	for (int i = 0; i < numRecords; i++) {
		int pos = i*7 + 3;


		if (pos + 7 > recordLen) {
			break;
		}

		// get field start and len. When this code is not used, do't worry about the lost time. The I-record occurs only once at the start of the file
		// start byte starts at 1, C/C++ start at 0
		int fieldStart = strToInt(recordString + pos,2) - 1;
		int fieldLen = strToInt(recordString + (pos+2),2) - fieldStart;

		if (fieldStart <= 0 || fieldLen <= 0) {
			continue;
		}

	    // horizontal GPS accuracy. Quasi standard
		if (!strncmp(recordString + (pos+4), "FXA",3)) {
			accuracyPos = fieldStart;
			accuracyLen = fieldLen;
			continue;
		}

		// vertical GPS accuracy. If not present i assume 5 times horizontal accuracy
		if (!strncmp(recordString + (pos+4), "VXA",3)) {
			vertAccuracyPos = fieldStart;
			vertAccuracyLen = fieldLen;
			continue;
		}

		// additional decimal digits for the latitude
		if (!strncmp(recordString + (pos+4), "LAD",3)) {
			latDecimalsPos = fieldStart;
			latDecimalsLen = fieldLen;
			latDecFactor = 1/1000.0; // The base record provides three decimal digits for the latitude minutes
			for (int k=0 ; k < fieldLen; k++) {
				latDecFactor *= 0.1;
			}
			continue;
		}

		// additional decimal digits for the longitude
		if (!strncmp(recordString + (pos+4), "LOD",3)) {
			lonDecimalsPos = fieldStart;
			lonDecimalsLen = fieldLen;
			lonDecFactor = 1/1000.0; // The base record provides three decimal digits for the longitude minutes
			for (int k=0 ; k < fieldLen; k++) {
				lonDecFactor *= 0.1;
			}
			continue;
		}

		// additional decimal digits for the timestamp
		if (!strncmp(recordString + (pos+4), "TDS",3)) {
			timestampDecimalsPos = fieldStart;
			timestampDecimalsLen = fieldLen;
			timeDecFactor = 1.0; // The base record provides whole seconds for the timestamp
			for (int k=0 ; k < fieldLen; k++) {
				lonDecFactor *= 0.1;
			}
			continue;
		}

	}

}

} /* namespace openEV */
