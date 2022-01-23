/*
 * NMEA0813.h
 *
 *  Created on: 05.09.2020
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#ifndef DRIVERS_NMEAGPS_NMEA0813_H_
#define DRIVERS_NMEAGPS_NMEA0813_H_

#include <string>

#include "CommonDefs.h"
#include "NMEASentence.h"
#include "NmeaGPSDriver.h"

namespace openEV::drivers::NMEA0813 {


class NMEASet;

/** \brief Implements a reader of NMEA 0813 sentences.
 *
 * Supported is a subset of sentences which are useful for my purposes.
 *
 * \see Primary (free) reference was [GPS Revealed](https://gpsd.gitlab.io/gpsd/NMEA.html) by Eric S. Raymond Version 2.23, Mar 2019
 * \see I use primarily uBlox modules, FLARM also uses them:
 * [NMEA description for uBlox6](https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)
 *
 */
class NMEA0813Protocol {
public:

	NMEA0813Protocol(NMEASet& set);
	virtual ~NMEA0813Protocol();

	void processSensorData (uint8_t const *data,uint32_t dataLen);

private:
	/** \brief The current sentence which is being assembled
	 *
	 */
	NMEASentence currSentence;

	/** \brief true when data collection in \ref currSentence is active,
	 * i.e. I received a start character '$' before, and the current sentence is not discarded.
	 *
	 */
	bool currSentenceActive = false;

	NMEASet& nmeaSet;

	/** \brief Parse one NMEA sentence in \ref currSentence
	 *
	 * Go through currSentence.buf. Replace the ',' separators with '\0' to create partial C strings
	 * for each field.
	 * The start of each field is then stored in currSentence.fields[i].
	 * In addition the first field is separated into currSentence.talkerID, and currSentence.sentenceType.
	 *
	 */
	void parseSentence();

};

} /* namespace openEV */

#endif /* DRIVERS_NMEAGPS_NMEA0813_H_ */
