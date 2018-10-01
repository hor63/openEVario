/*
 * BRecordSection.h
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

#ifndef DRIVERS_IGCREADER_BRECORDSECTION_H_
#define DRIVERS_IGCREADER_BRECORDSECTION_H_

#include "OEVCommon.h"

#include "drivers/GliderVarioDriverBase.h"
#include "main/GliderVarioMainPriv.h"

namespace openEV {

/** \brief Base class for IGC file B record section processors
 *
 * Subclasses of this class implement reading, and interpreting sections of a B (and occasional K record)
 *
 * B records
 * ---------
 *
 * The standard structure of a B record is as follows:
 *
 *     B time   Latitude Longitude Valid BaroAlt GPSAlt
 *     B HHMMSS DDMMmmmN DDDMMmmmE V     PPPPP   GGGGG CR LF
 *     0 123456 78901234 567890123 4     56789   01234
 *                 1          2                  3
 *
 * Additional sections may follow. These are defined with meaning, position and length in the I record.
 *
 * K records
 * ---------
 *
 * The only fixed field in a K record is the time. Any data meaning is solely defined in the J record:
 *     K time   data CR LF
 *     K HHMMSS xxxxxxxxxx
 *     0 123456 7890...
 */
class BRecordSectionBase {
public:

	/** \brief Constructor
	 *
	 * @param recordPos Start position of the section in one record of the IGC file
	 * @param recordLen Length of the section in one record of the IGC file
	 */
	BRecordSectionBase(
			int recordPos,
			int recordLen)
	:recordPos {recordPos},
	 recordLen {recordLen}
	{ }

	virtual ~BRecordSectionBase();

	/** \brief Read and interprete one B record of the IGC file. Update the Kalman filter with the measurements from the record
	 *
	 * @param varioMain Main object of the filter which is to be updated with the measurements
	 * @param recordString Complete line of the IGC file including the identifying letter (here B), but without the line terminators (CR-LF).
	 * @param recordLen Lengh of the recorf (without the CR-LF terminators)
	 */
	virtual void updateFilter (
			GliderVarioMainPriv &varioMain,
			char *const recordString,
			int recordLen
			) = 0;

	/** \brief Read a signed integer value from a fixed length string
	 *
	 * Only minimal health checks are performed. The function stops reading when it encounters a non-digit character. Exception is the first character which can be a '-' character.
	 *
	 * @param str The string which is read as signed integer (does not have to be NULL terminated)
	 * @param len Length of valid part the string (Usually the string is not NULL terminated, as the sections of a line in a IGC file are fixed positioned, and not separated.
	 * @return Signed interger value
	 */
	static int strToInt (char const* str,int len) {
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

protected:

	int recordPos;
	int recordLen;

}; // class BRecordSectionBase

/// \brief Implements simultaneously the update with latitude and longitude, GPS altitude, and pressure from the B-record fix.
class BRecordSectionStd : public BRecordSectionBase {
public:

	// fixed mandatory positions within a B-record

    static int constexpr latDegPos = 7;
    static int constexpr latDegLen = 2;
    static int constexpr latMinPos = 9;
    static int constexpr latMinLen = 5;
    static int constexpr latNPos   = 14; ///< \brief N or S to indicate North or South of the equator. N becomes positive, S becomes negative latitude

    static int constexpr lonDegPos = 15;
    static int constexpr lonDegLen = 3;
    static int constexpr lonMinPos = 18;
    static int constexpr lonMinLen = 5;
    static int constexpr lonEPos   = 23; ///< \brief E or W to indicate East or West of the Terminator. E becomes positive, W becomes negative longitude.

    static int constexpr gpsValidPos = 24; ///< \brief "A" indicates a valid 3-D fix, "V" indicates an invalid or 2d-only fix.

    static int constexpr baroAltPos = 25;
    static int constexpr baroAltLen = 5;

    static int constexpr gpsAltPos = 30;
    static int constexpr gpsAltLen = 5;


	BRecordSectionStd()
	:BRecordSectionBase {7,28}
	{ }

	virtual ~BRecordSectionStd();

	/// \brief \see BRecordSectionBase::updateFilter
	virtual void updateFilter (
			GliderVarioMainPriv &varioMain,
			char *const recordString,
			int recordLen
			);

	/** \brief Process an I-Record which defines the type and location of optional fields in the following B-Records
	 *
	 * @param recordString The entire I-record string including the leading character I, but exclusive the line terminator CR-LF.
	 * @param recordLen Length of the record. \ref recordString is not necessarily NULL-terminated.
	 */
	void processIRecord (
			char *const recordString,
			int recordLen
			);

protected:

	/// \brief Defined by the FXA code in the I-record
	int accuracyPos = -1;
	/// \brief \see accuracyPos
	int accuracyLen = -1;

	/// \brief Vertical accuracy in meter. Defined in the VXA record in the I-record
	int vertAccuracyPos = -1;
	/// \brief \see vertAccuracyPos
	int vertAccuracyLen = -1;

	/// \brief More decimal places for the latitude. Defined by the LAD code in the I-record
	int latDecimalsPos = -1;
	/// \brief \see latDecimalsPos
	int latDecimalsLen = -1;
	/// \brief Factor to multiply the integer before adding to the base latitude.
	/// \see latDecimalsPos
	double latDecFactor = 0.0;

	/// \brief More decimal places for the longitude. Defined by the LOD code in the I-record
	int lonDecimalsPos = -1;
	/// \brief \see lonDecimalsPos
	int lonDecimalsLen = -1;
	/// \brief Factor to multiply the integer before adding to the base longitude.
	/// \see lonDecimalsPos
	double lonDecFactor = 0.0;

	/// \brief Decimals of the timestamp. Defined by the TDS code in the I-record
	int timestampDecimalsPos = -1;
	/// \brief \see timestampDecimalsPos
	int timestampDecimalsLen = -1;
	/// \brief Factor to multiply the integer before adding to the base timestamp.
	/// \see lonDecimalsPos
	double timeDecFactor = 0.0;


};

} /* namespace openEV */

#endif /* DRIVERS_IGCREADER_BRECORDSECTION_H_ */
