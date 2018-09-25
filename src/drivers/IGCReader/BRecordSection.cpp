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
	int pos = 1;

	// termination condition checks that a full field of 7 characters is still available in the string
	for (int i = 0; i < numRecords; i++) {

		if (i*7 + 10 > recordLen) {
			break;
		}

		if (!strncmp(recordString + pos + 4, "FXA",3)) {

		}

	}

}

} /* namespace openEV */
