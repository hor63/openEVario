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
#include <ratio>

#include "BRecordSection.h"

namespace openEV {

/** \brief Read a signed integer value from a fixed length string
 *
 * Only minimal health checks are performed. The function stops reading when it encounters a non-digit character. Exception is the first character which can be a '-' character.
 *
 * @param str The string which is read as signed integer (does not have to be NULL terminated)
 * @param len Length of valid part the string (Usually the string is not NULL terminated, as the sections of a line in a IGC file are fixed positioned, and not separated.
 * @return Signed integer value
 */
static inline int strToInt (char const* str,int len) {
	int rc = 0;
	int sign = 1;

	if (len <= 0) {
		return rc;
	}

	if (*str == '-') {
		sign = -1;
		str ++;
		len--;
	}

	while (len) {

		if (*str >= '0' && *str <= '9') {
			rc = rc * 10 + (*str - '0');

		} else {
			break;
		}

		len --;
		str ++;
	}

	return sign * rc;

}

/** \brief Read the fractional part of a number.
 *
 * There are B-record extensions for more decimal digits on the coordinates and the time stamp.
 * This method interprets a numeric string as decimal digits right to the decimal.
 * e.g. a string "1234" is interpreted as 0.1234
 *
 * Only minimal health checks are performed. The function stops reading when it encounters a non-digit character.
 *
 * @param str The string which is read as fractional digits (does not have to be NULL terminated)
 * @param len Length of valid part the string (Usually the string is not NULL terminated, as the sections of a line in a IGC file are fixed positioned, and not separated.
 * @return Signed double value
 */
static inline double strToFract (char const* str,int len) {
	double rc = 0;
	double factor = 1.0;

	if (len <= 0) {
		return rc;
	}

	while (len) {

		if (*str >= '0' && *str <= '9') {
			rc = rc * 10.0 + double(*str - '0');
			factor *= 10.0;
		} else {
			break;
		}

		len --;
		str ++;
	}

	return rc / factor;

}

BRecordSectionStd::~BRecordSectionStd() {

}

void BRecordSectionStd::processBRecord (
		char *const recordString,
		int recordLen,
		BRecord &bRecord,
		double &startTimeDay
		) {

	// process the GPS position and alitude only when
	if (recordString[gpsValidPos] == gpsValid) {

		bRecord.altGPS = double(strToInt(recordString + gpsAltPos,gpsAltLen));
		// Minutes is a 5-digit string in the form MMmmm (i.e. MM.mmm).
		// The integer value must be divided by 1000 to return minutes, and divided by 60 to return degrees.
		bRecord.latitude = double(strToInt(recordString + latDegPos,latDegLen)) + double(strToInt(recordString + latMinPos,latMinLen)) / 60000.0;
		if (latDecimalsPos >= 0) {
			// Divide the fractional minutes by 1000 (the standard string already has 3 decimal digits) and then by 60 to return Degrees.
			bRecord.latitude += strToFract(recordString + latDecimalsPos,latDecimalsLen) / 60000.0;
		}
		if (recordString[latNPos] == 'S' || recordString[latNPos] == 's') {
			bRecord.latitude *= -1.0;
		}

		bRecord.longitude = double(strToInt(recordString + lonDegPos,lonDegLen)) + double(strToInt(recordString + lonMinPos,lonMinLen)) / 60000.0;
		if (lonDecimalsPos >= 0) {
			// Divide the fractional minutes by 1000 (the standard string already has 3 decimal digits) and then by 60 to return Degrees.
			bRecord.longitude += strToFract(recordString + lonDecimalsPos,lonDecimalsLen) / 60000.0;
		}
		if (recordString[lonEPos] == 'W' || recordString[latNPos] == 'w') {
			bRecord.longitude *= -1.0;
		}

		if (accuracyPos >= 0) {
			bRecord.posAccuracy = double(strToInt(recordString + accuracyPos,accuracyLen));
		} else {
			// Use a default appropriate for a GPS receiver in a plane in free view to the sky
			bRecord.posAccuracy = 2.0;
		}

		if (vertAccuracyPos >= 0) {
			bRecord.altGPSAccuracy = double(strToInt(recordString + vertAccuracyPos,vertAccuracyLen));
		} else {
			// By default the vertical accuracy is a lot worse than the horizontal accuracy.
			bRecord.altGPSAccuracy = bRecord.posAccuracy * 5.0;
		}


	} else { // if (recordString[gpsValidPos] == gpsValid)

		bRecord.altGPS = valueInvalid;
		bRecord.latitude = valueInvalid;
		bRecord.longitude = valueInvalid;
	} // if (recordString[gpsValidPos] == gpsValid)


	double timestampSecSinceMidnight = double (
			strToInt(recordString + timestampPos,2) * 24*60 +
			strToInt(recordString + (timestampPos + 2),2) * 60 +
			strToInt(recordString + (timestampPos + 4),2)
			);
	if (timestampDecimalsPos >= 0) {
		timestampSecSinceMidnight += strToFract(recordString + timestampDecimalsPos,timestampDecimalsLen);
	}

	if (timestampSecSinceMidnight < startTimeDay) {
		startTimeDay -= 24.0*60.0*60.0;
	}

	bRecord.timeSinceStart =  std::chrono::duration_cast<std::chrono::system_clock::duration> (
			std::chrono::duration<double> (timestampSecSinceMidnight - startTimeDay));

#error Pressure calculation from the barometric altitude is missing

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
			continue;
		}

		// additional decimal digits for the longitude
		if (!strncmp(recordString + (pos+4), "LOD",3)) {
			lonDecimalsPos = fieldStart;
			lonDecimalsLen = fieldLen;
			continue;
		}

		// additional decimal digits for the timestamp
		if (!strncmp(recordString + (pos+4), "TDS",3)) {
			timestampDecimalsPos = fieldStart;
			timestampDecimalsLen = fieldLen;
			continue;
		}

	}

}

} /* namespace openEV */
