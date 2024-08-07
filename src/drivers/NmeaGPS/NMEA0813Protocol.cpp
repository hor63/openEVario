/*
 * NMEA0813.cpp
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
#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <fstream>

#include "fmt/format.h"

#include "NmeaGPSDriver.h"
#include "NMEA0813Protocol.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS.NMEA0813");
	}
}

#endif


namespace openEV::drivers::NMEA0813 {

NMEA0813Protocol::NMEA0813Protocol(NMEASet& set)
		:nmeaSet{set}
{

	initLogger();

}

NMEA0813Protocol::~NMEA0813Protocol() {
}

void NMEA0813Protocol::processSensorData(const uint8_t *data, uint32_t dataLen) {

	uint32_t i = 0;

	while (i < dataLen) {
		// When I receive a sentence start discard any started sentence.
		if (data[i] == '$') {
			// Mark the start of collecting sentence data into the current message

			if (currSentenceActive) {
				LOG4CXX_WARN(logger,fmt::format(_(
						"{0}: Received unexpected sentence start. Discarding {1} bytes."),
						__PRETTY_FUNCTION__, currSentence.bufLen));
			}

			currSentenceActive = true;
			memset (&currSentence,0,sizeof(currSentence));
		} else { // if (data[i] == '$')
			// When data collection is not active silently discard characters
			if (currSentenceActive) {

				// I am collecting data.

				// Check for end of sentence
				if (data [i] == '\n') {
					// Expect a CR character at the end of the sentence.
					// And the current sentence must at least have two characters before the CR.
					if (currSentence.bufLen > 2 && currSentence.buf[currSentence.bufLen - 1] == '\r') {
						// I got a complete line
						// Throw away the CR.
						currSentence.bufLen --;
						currSentence.buf[currSentence.bufLen] = 0;
						LOG4CXX_DEBUG (logger, "processSensorData: Received one line: \"" << currSentence.buf << "\"");

						parseSentence();

						// Start over.
						currSentenceActive = false;
						i++;
						continue;
					} else {
						LOG4CXX_WARN(logger,fmt::format(_(
								"{0}: Too short sentence received, or CR character is missing. Discarding {1} bytes."),
								__PRETTY_FUNCTION__, currSentence.bufLen));
						currSentenceActive = false;
						i++;
						continue;
					}
				}

				// Now some checks

				// Check for illegal characters
				if ((data[i] < 0x20 || data[i] > 0x7E) && data[i] != '\r') {
					// This is not a printable ASCII character or CR character, and not allowed.
					// Discard the received data and wait for the next sentence start
					LOG4CXX_WARN(logger,fmt::format(_(
							"{0}: Received illegal character {1:#04X}. Discarding current sentence with {2} bytes."),
							__PRETTY_FUNCTION__,static_cast<uint32_t>(data[i]),currSentence.bufLen));
					currSentenceActive = false;
					i++;
					continue;
				}

				// Check for buffer overrun
				if (currSentence.bufLen >= (NMEASentence::maxLenSentence - 1)) {
					// Even with the buffer being a good deal larger than the max. length of sentences (80 char)
					// the buffer overflows.
					// This means I missed somehow the end of the previous sentence.
					// Discard the received data and wait for the next sentence start
					LOG4CXX_WARN(logger,fmt::format(_(
							"{0}: Current sentence buffer overflow (> {2} characters). Discard {1} bytes."),
							__PRETTY_FUNCTION__,currSentence.bufLen,NMEASentence::maxLenSentence));
					currSentenceActive = false;
					i++;
					continue;
				}

				// Add the byte to the buffer and move on
				currSentence.buf[currSentence.bufLen] = data[i];
				currSentence.bufLen ++;

			}

		} // else // if (data[i] == '$')

		i++;
	} // while (i < dataLen)

}

void NMEA0813Protocol::parseSentence() {

	uint8_t checksum = 0;

	for (uint32_t i=0; i<NMEASentence::maxLenSentence && currSentence.buf[i] != 0; i++) {

		// The last first: Checksum check, and the end of the message.
		if (currSentence.buf[i] == '*') {

			// Terminate the previous field
			currSentence.buf[i] = 0;
			if (isxdigit(currSentence.buf[i + 1]) &&
					isxdigit(currSentence.buf[i + 2]) &&
					currSentence.buf[i + 3] == 0) {
				auto msgChkSum = strtoul((char const*)(&currSentence.buf[i + 1]),nullptr,16);
				if (msgChkSum != checksum){
					LOG4CXX_WARN(logger,fmt::format(_(
							"{0}: Checksum in the sentence {1:#04X} is unequal to the calculated checksum {2:#04X}. Discard sentence."),
							__PRETTY_FUNCTION__,msgChkSum,static_cast<uint32_t>(checksum)));
					return;
				} else {
					LOG4CXX_DEBUG(logger, "Checksum \"" << &currSentence.buf[i + 1] << "\" matches.");
				}
			} else {
				LOG4CXX_WARN(logger,fmt::format(_(
						"{0}: Checksum section contains non-numeric characters \"{1}\". Discard sentence."),
						__PRETTY_FUNCTION__,
						reinterpret_cast<char const*>(&currSentence.buf[i])));
				return;
			}
			// End of story in any case.
			break;
		}

		// All characters of the sentence except the '*' at the end is subject to the checksum.
		// The $ character is already stripped away.
		checksum ^= currSentence.buf[i];

		if (currSentence.buf[i] == ',') {
			// Field separator
			// Make it a \0 string terminator
			currSentence.buf[i] = 0;

			if (currSentence.numFields < NMEASentence::maxNumFields) {
			// Store the start of the next field in the sentence
			currSentence.fields[currSentence.numFields] = &currSentence.buf[i+1];
			currSentence.numFields++;
			} else {
				LOG4CXX_WARN(logger,fmt::format(_(
						"{0}: Number of fields in the sentence exceeds the maximum {1} fields. Discard sentence."),
						__PRETTY_FUNCTION__,NMEASentence::maxNumFields));
				return;
			}
		}

	}

	// Process the type of sentence, and talker ID.
	if (currSentence.buf[0] == 'P') {
		// Proprietary message Talker ID is one character.
		currSentence.talkerID[0] = 'P';
		currSentence.talkerID[1] = 0;

		// The next character onwards contains the sentence type.
		// If '\0' follows immediately, so be it.
		// I will throw away proprietary messages anyway at this time of writing.
		currSentence.sentenceTypeString = &currSentence.buf[1];

	} else {
		// A regular NMEA message. Talker ID is expected to be 2 char.
		// But you never can be sure. I will check that too.
		currSentence.talkerID[0] = currSentence.buf[0];
		currSentence.talkerID[1] = currSentence.buf[1];
		currSentence.talkerID[2] = 0;

		// Prevent spill-over of sentence type into the next field in case
		// that the preamble is only one character, and not 'P'.
		// Illegal, but I want to avoid any accidents
		if (currSentence.buf[1] != 0) {
			currSentence.sentenceTypeString = &currSentence.buf[2];
		} else {
			// currSentence.buf[1] is already \0. Therefore the sentence type is also emtpy.
			currSentence.sentenceTypeString = &currSentence.buf[1];
		}
	}

#if defined HAVE_LOG4CXX_H
	if (logger->isDebugEnabled()) {
		std::ostringstream txt;
		txt << "NMEA sentence parsed: \nTalker ID: \"" << currSentence.talkerID
				<< "\", \nsentence type: \"" << currSentence.sentenceTypeString
				<< "\", \nnumber fields: " << currSentence.numFields;
		for (uint32_t i = 0; i < currSentence.numFields; i++) {
			txt << ", fields[" << i << "] = \"" << currSentence.fields[i] << "\"";
		}
		LOG4CXX_TRACE(logger,txt.str());
	}
#endif // #if defined HAVE_LOG4CXX_H

	// Evaluate the sentence type
	if (!strcmp((char const*)(currSentence.sentenceTypeString),"RMC") ) {
		currSentence.sentenceType = NMEASentence::NMEA_RMC;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GGA") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GGA;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GNS") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GNS;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GST") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GST;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GSA") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GSA;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GBS") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GBS;
	} else if (!strcmp((char const*)(currSentence.sentenceTypeString),"GLL") ) {
		currSentence.sentenceType = NMEASentence::NMEA_GLL;
	} else {
		currSentence.sentenceType = NMEASentence::NMEA_DONT_CARE;
	}

	// Throw out any un-supported sentence type.
	if (currSentence.sentenceType != NMEASentence::NMEA_DONT_CARE) {
		// Now process the validated and parsed sentence.
		nmeaSet.processSentence(currSentence);
	}

}

#if !defined DOXYGEN
NMEASentence::NMEASentenceTypeHelperClass NMEASentence::NMEASentenceTypeHelperObj;
#endif

} /* namespace openEV */



std::ostream& operator << (std::ostream &o, openEV::drivers::NMEA0813::NMEASentence::NMEASentenceType t){
	o << openEV::drivers::NMEA0813::NMEASentence::NMEASentenceTypeHelperObj.getString(t);
	return o;
}

#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::NMEA0813::NMEASentence::NMEASentenceType t) {
	std::ostream &o = b;
	return operator << (o,t);
}
#endif

