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
 *     B time   Latitude Longitude Valid PressAlt GPSAlt
 *     B HHMMSS DDMMmmmN DDDMMmmmE V     PPPPP    GGGGG CR LF
 *     0 123456 78901234 567890123 4     56789    01234
 *                 1          2                   3
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

	BRecordSectionBase(
			int recordPos,
			int recordLen)
	:recordPos {recordPos},
	 recordLen {recordLen}
	{ }

	virtual ~BRecordSectionBase();

	virtual void updateFilter (
			GliderVarioMainPriv &varioMain,
			char *const recordString,
			int recordLen
			) = 0;

protected:

	int recordPos;
	int recordLen;

}; // class BRecordSectionBase

class BRecordSectionLatitude : public BRecordSectionBase {
public:

	BRecordSectionLatitude()
	:BRecordSectionBase {7,8},
	 accuracyPos {-1},
	 accuracyLen {-1},
	 validityPos {24},
	 validityLen {1}
	{ }

	virtual ~BRecordSectionLatitude();

	virtual void updateFilter (
			GliderVarioMainPriv &varioMain,
			char *const recordString,
			int recordLen
			);

	void set_accuracyLoc (
			int accuracyPos,
			int accuracyLen,
			) {
		this->accuracyPos = accuracyPos;
		this->accuracyLen = accuracyLen;
	}


protected:

	int accuracyPos;
	int accuracyLen;

	int validityPos;
	int validityLen;

};

} /* namespace openEV */

#endif /* DRIVERS_IGCREADER_BRECORDSECTION_H_ */
