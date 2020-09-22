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
#  include <config.h>
#endif

#include <fstream>

#include "NMEA0813.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS.NMEA0813");
	}
}

#endif


namespace openEV {

NMEA0813::NMEA0813() {

	initLogger();

}

NMEA0813::~NMEA0813() {
	// TODO Auto-generated destructor stub
}

void NMEA0813::processSensorData(const uint8_t *data, uint32_t dataLen) {

	uint32_t i = 0;

	while (i < dataLen) {
		// When I receive a sentence start discard any started sentence.
		if (data[i] == '$') {
			// Mark the start of collecting sentence data into the current message

			if (currSentenceActive) {
				LOG4CXX_WARN(logger,"processSensorData: Received sentence start. Discarding " << currSentence.bufLen << " bytes.");
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
						LOG4CXX_WARN(logger,"processSensorData: Too short sentence or CR missing. Discarding " << currSentence.bufLen << " bytes.");
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
					LOG4CXX_WARN(logger,"processSensorData: Received illegal character data[" << i << "] = 0X"
							<< std::hex << int(data[i]) << std::dec
							<< ". Discarding " << currSentence.bufLen << " bytes.");
					currSentenceActive = false;
					i++;
					continue;
				}

				// Check for buffer overrun
				if (currSentence.bufLen >= (maxLenSentence - 1)) {
					// Even with the buffer being a good deal larger than the max. length of sentences (80 char)
					// the buffer overflows.
					// This means I missed somehow the end of the previous sentence.
					// Discard the received data and wait for the next sentence start
					LOG4CXX_WARN(logger,"processSensorData: currSentence buffer overflow. Discard " << currSentence.bufLen << " bytes.");
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

void NMEA0813::parseSentence() {

	uint8_t checksum = 0;

	for (uint32_t i=0; i<maxLenSentence && currSentence.buf[i] != 0; i++) {

		// The last first: Checksum check, and the end of the message.
		if (currSentence.buf[i] == '*') {

			// Terminate the previous field
			currSentence.buf[i] = 0;
			if (isxdigit(currSentence.buf[i + 1]) &&
					isxdigit(currSentence.buf[i + 2]) &&
					currSentence.buf[i + 3] == 0) {
				auto msgChkSum = strtoul((char const*)(&currSentence.buf[i + 1]),nullptr,16);
				if (msgChkSum == checksum){
					LOG4CXX_DEBUG(logger, "Checksum \"" << &currSentence.buf[i + 1] << "\" matches.");
					// Now process the validated and parsed sentence.
					processParsedSentence();
				}
			} else {
				LOG4CXX_WARN(logger,"Read invalid checksum section \""
						<< &currSentence.buf[i] << "\". Discard sentence.");
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

			if (currSentence.numFields < maxNumFields) {
			// Store the start of the next field in the sentence
			currSentence.fields[currSentence.numFields] = &currSentence.buf[i+1];
			currSentence.numFields++;
			} else {
				LOG4CXX_WARN(logger,"NMEA: Number fields " << currSentence.numFields << " << exceeds the field pointer array."
						<< " Remainder of message is: \"" << &currSentence.buf[i+1] << "\"");
			}
		}

	}

	// Process the type of sentence, and talker ID.
	if (currSentence.buf[0] == 'P') {
		// Proprietary message Talker ID is one character.
		currSentence.talkerID[0] = 'P';
		currSentence.talkerID[1] = 0;
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
			currSentence.sentenceType = &currSentence.buf[2];
		} else {
			// currSentence.buf[1] is already \0. Therefore the sentence type is also emtpy.
			currSentence.sentenceType = &currSentence.buf[1];
		}
	}

#if defined HAVE_LOG4CXX_H
	if (logger->isDebugEnabled()) {
		std::ostringstream txt;
		txt << "NMEA sentence parsed: \nTalker ID: \"" << currSentence.talkerID
				<< "\", \nsentence type: \"" << currSentence.sentenceType
				<< "\", \nnumber fields: " << currSentence.numFields;
		for (uint32_t i = 0; i < currSentence.numFields; i++) {
			txt << "\n	fields[" << i << "] = \"" << currSentence.fields[i] << "\"";
		}
		LOG4CXX_DEBUG(logger,txt.str());
	}
#endif // #if defined HAVE_LOG4CXX_H

}

void NMEA0813::processParsedSentence() {
}

} /* namespace openEV */
