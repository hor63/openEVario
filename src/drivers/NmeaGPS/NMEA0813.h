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

#include "OEVCommon.h"
#include "NmeaGPSDriver.h"

namespace openEV {

/** \brief Implements a reader of NMEA 0813 sentences.
 *
 * Supported is a subset of sentences which are useful for my purposes.
 *
 * \see Primary (free) reference was [GPS Revealed](https://gpsd.gitlab.io/gpsd/NMEA.html) by Eric S. Raymond Version 2.23, Mar 2019
 * \see I use primarily uBlox modules, FLARM also uses them:
 * [NMEA description for uBlox6](https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)
 *
 */
class NMEA0813 {
public:

	/** \brief Max. length of internal buffers for NMEA sentences.
	 *
	 * According to specification the maximum length of a sentence cannot be more than 82 characters
	 * incl. CR and LF characters. So here I am on the safe side.
	 *
	 */
	static constexpr uint16_t maxLenSentence = 128;

	/** \brief Max. number of fields in a NMEA sentence
	 *
	 */
	static constexpr int maxNumFields   = 20;

	struct NMEASentence {
		/** \brief Complete string of the sentence.
		 *
		 * During processing it is modified: Commas and the '*' are replaced by '\0' as string terminator of the fields
		 */
		uint8_t buf [maxLenSentence];
		/// Number of bytes in \ref buf.
		uint32_t bufLen;
		/// \brief Talker ID from the first field. It is actually copied here.
		uint8_t TalkerID[4];
		/// \brief Points to the character in the first field behind the talker ID designating the sentence type.
		uint8_t *SentenceType;
		/// \brief The data fields of the sentence. The strings themselves lie in \p buf.
		uint8_t * fields [maxNumFields];
		/// Number of defined \ref fields
		uint32_t numFields;
	};

	NMEA0813();
	virtual ~NMEA0813();

	void processSensorData (uint8_t const *data,uint32_t dataLen);

private:
	/** \brief Look-ahead buffer for the next sentence
	 *
	 * When I read a block of data it may contain the remainder of a pending message
	 * and the first part of the next one.
	 * The tail of the pending message will go to NMEASentence::buf together with the first part which resided up to now in \p lAhBuffer.
	 * The head of the next message goes to \p lAhBuffer waiting for the remainder.
	 */
	uint8_t lAhBuffer [maxLenSentence];

	/** \brief Number of bytes in \ref lAhBuffer
	 *
	 */
	uint32_t lAhBufferLen = 0;

	/** \brief The current sentence which is being assembled
	 *
	 */
	NMEASentence currSentence;

	/** \brief true when data collection in \ref currSentence is active,
	 * i.e. I received a start character '$' before, and the current sentence is not discarded.
	 *
	 */
	bool currSentenceActive = true;

	/** \brief Parse and process one NMEA sentence in \ref currSentence
	 *
	 */
	void processSentence();
};

} /* namespace openEV */

#endif /* DRIVERS_NMEAGPS_NMEA0813_H_ */
