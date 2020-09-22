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

						processSentence();

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

void NMEA0813::processSentence() {
}

} /* namespace openEV */
