/*
 * NMEASentence.h
 *
 *  Created on: 08.10.2020
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

#include "OEVCommon.h"

#ifndef DRIVERS_NMEAGPS_NMEASENTENCE_H_
#define DRIVERS_NMEAGPS_NMEASENTENCE_H_

namespace openEV {

struct NMEASentence {
	/** \brief Max. length of internal buffers for NMEA sentences.
	 *
	 * According to specification the maximum length of a sentence cannot be more than 82 characters
	 * incl. CR and LF characters. So here I am on the safe side.
	 *
	 */
	static constexpr uint16_t maxLenSentence = 256;

	/** \brief Max. number of fields in a NMEA sentence
	 *
	 */
	static constexpr int maxNumFields   = 30;

	/** \brief Complete string of the sentence.
	 *
	 * During processing it is modified: Commas and the '*' are replaced by '\0' as string terminator of the fields
	 */
	uint8_t buf [maxLenSentence];
	/// Number of bytes in \ref buf.
	uint32_t bufLen;
	/// \brief Talker ID from the first field. It is actually copied here.
	uint8_t talkerID[4];
	/// \brief Points to the character in the first field behind the talker ID designating the sentence type.
	uint8_t *sentenceType;
	/// \brief The data fields of the sentence. The strings themselves lie in \p buf.
	uint8_t * (fields [maxNumFields]);
	/// Number of defined \ref fields
	uint32_t numFields;
};

} // namespace openEV


#endif /* DRIVERS_NMEAGPS_NMEASENTENCE_H_ */
