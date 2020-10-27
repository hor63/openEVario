/*
 * NMEASet.cpp
 *
 *  Created on: 24.09.2020
 *      Author: kai_horstmann
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
#include "NMEASet.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS.NMEASet");
	}
}

#endif

namespace openEV {

NMEASet::NMEASet(NmeaGPSDriver& gpsDriver)
		:gpsDriver {gpsDriver},
		teachInRecords{},
		 currTeachInRecord {teachInRecords.end()}
		{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif

}

NMEASet::~NMEASet() {

}

/** \brief My inline and faster isdigit variant
 *
 * @param c character to check
 * @return true if \p c is between '0' and '9' ASCII.
 */
inline bool isDigit (int c) {
	return (c>='0' && c<='9');
}

double NMEASet::strToD (uint8_t const *str) {
	double rc = 0.0;
	double sign = 1.0;
	double factor = 1.0;
	uint8_t const *lStr = str;

	// Eat up leading space
	while (*lStr == ' ') lStr++;

	if (*lStr == '+') {
		// A leading + is allowed, but does not change a thing
		lStr ++;
	} else {
		if (*lStr == '-') sign = -1.0;
	}
	for (; isDigit(*lStr);lStr ++) {
		rc = rc * 10.0 + double(*lStr - '0');
	}

	if (*lStr == '.') {
		// Eat up the decimal separator
		lStr ++;
	} else {
		std::ostringstream s;
		s << "strToD: String \"" << str << "\" is not a valid double value";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// now come the decimal places when there are any.
	for (; isDigit(*lStr);lStr ++) {
		factor /= 10.0;
		rc += double(*lStr - '0') * factor;
		lStr ++;
	}

	// Check if I reached the end of the string. Else there is invalid trailing stuff
	// This is not allowed for this function.
	if (lStr != 0) {
		std::ostringstream s;
		s << "strToD: String \"" << str << "\" is not a valid double value";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	LOG4CXX_DEBUG (logger,"strToD: String \"" << str << "\" converted to " << (rc * sign));
	return rc * sign;
}

double NMEASet::strToD (uint8_t const *str,uint32_t maxLen) {
	uint8_t * strippedStr = new uint8_t[maxLen + 1];
	double rc;

	// Copy the designated part of the string
	memcpy(strippedStr,str,maxLen);
	// Terminate it
	strippedStr[maxLen] = 0;

	try {
		rc = strToD(strippedStr);
	}
	catch (...) {
		delete strippedStr;
		throw;
	}

	delete strippedStr;
	return rc;
}

double NMEASet::nmeaCoordToD(uint8_t const * str) {

	double degrees;
	double minutes;
	uint32_t i;
	uint8_t const *lStr;

	// Look for the decimal.
	for (lStr = str;*lStr != 0; lStr ++) {
		if (*str == '.') break;
	}

	if (*lStr == 0) {
		std::ostringstream s;
		s << "nmeaCoordToD: String \"" << str << "\" does not contain a decimal '.'";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// Where is the dot in the String?
	i = lStr - str;
	if (i < 3) {
		std::ostringstream s;
		s << "nmeaCoordToD: String \"" << str << "\" does not have space for degrees '.'";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// Get the degrees
	degrees = strToD(str,i-2);
	minutes = strToD (str + (i-2));

	LOG4CXX_DEBUG(logger,"nmeaCoordToD: \"" << str << "\" converted to " << degrees << " Degrees, " << minutes << " minutes = "
			<< " = " << (degrees + minutes / 60.0));

	return degrees + minutes / 60.0;
}

uint32_t NMEASet::NMEATimeStampToMS(uint8_t const *timestampStr) {
	uint32_t rc = 0xFFFFFFFFU;
	uint32_t i = 0;
	char const *lStr = (char const *)(timestampStr);

	// First two characters are hours
	if (isDigit(*lStr)) {
		rc = (*lStr - '0') * (1000*3600*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*3600);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}


	// Next two characters are minutes
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*60*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*60);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}

	// Next two characters are seconds
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}

	if (*lStr == '.') {
		lStr ++;
		for (i = 100; i >= 1; i /= 10) {
			if (isDigit(*lStr)) {
				rc += (*lStr - '0') * i;
				lStr ++;
			} else {
				break;
			}
		}
	}

	LOG4CXX_DEBUG(logger,"NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" converted to " << rc);

	return rc;
}

void NMEASet::processSentenceTeachIn(
		const NMEASentence &newSentence) {

	auto currTime = std::chrono::system_clock::now();

	// Some sentences do no carry a GPS timestmap. Therefore by default assume the same cycle.
	auto thisGPSTimeStampMS = currSequenceTimestampMS;
	TeachInRecord locRecord;
	bool useLocRecord = false;

	locRecord.recordType = (char const*)(newSentence.sentenceType);

	try {

		LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Process message type " << newSentence.sentenceType);

		if (! strcmp((char const*)(newSentence.sentenceType),"GGA")) {
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GGA_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GGA_TIME]);
				useLocRecord = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesHDop = true;
				locRecord.definesMSL = true;
				locRecord.definesPosition = true;
			}
		}  else if (! strcmp((char const*)(newSentence.sentenceType),"GBS")) {
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GBS_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GBS_TIME]);
				useLocRecord = true;
				locRecord.definesAbsErr = true;
			}
		} else if (! strcmp((char const*)(newSentence.sentenceType),"GLL")) {
			if (*(newSentence.fields[GLL_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GLL_TIME]);
				useLocRecord = true;
				locRecord.definesPosition = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesHDop = true;
			}
		} else if (! strcmp((char const*)(newSentence.sentenceType),"GNS")) {
			if (*(newSentence.fields[GNS_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GNS_TIME]);
				useLocRecord = true;
				locRecord.definesPosition = true;
				locRecord.definesHDop = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesMSL = true;
			}
		} else if (! strcmp((char const*)(newSentence.sentenceType),"GSA")) {
			// GSA is a bit unique because it does not have an own timestamp field.
			// It kind of swims in the fix update set with other NMEA sentences.
			// For simplicity, and practical experience with GNSS receivers I assume
			// the timestamp is the same as the previous record.
			useLocRecord = true;
			// Because I do not have a timestamp I cannot decide if teach-in started.
			// Teach-in starts when the timestamp changes compared to the previous cycle,
			// or to the initial unplausible timestamp.
			locRecord.definesHDop = true;
			locRecord.definesPDop = true;
			locRecord.definesVDop = true;
			locRecord.definesQualitiyLevel = true;
		}  else if (! strcmp((char const*)(newSentence.sentenceType),"GST")) {
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GST_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GST_TIME]);
				useLocRecord = true;
				locRecord.definesAbsErr = true;
			}
		} else if (! strcmp((char const*)(newSentence.sentenceType),"RMC")) {
			if (*(newSentence.fields[RMC_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[RMC_TIME]);
				useLocRecord = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesPosition = true;
			}
		}

	}
	catch (NMEASetParseException const& e) {
		LOG4CXX_WARN(logger,"processSentenceTeachIn: Sentence " << newSentence.sentenceType << " failed. Reason: " << e.what());
	}

	if (useLocRecord) {

		if (currSequenceTimestampMS != thisGPSTimeStampMS) {
			// If the last timestamp is at the initial value throw away messages until
			// the next timestamp change occurs.
			// Only this way I can be sure to catch the first start of a new cycle properly.
			if (currSequenceTimestampMS != NMEATimeStampUndef) {
				if (teachInRecords.size() == numTeachInCycles) {
					finishTeachIn();

					// Switch to the operational message processor
					// Without that this method would be called forever and be stuck here.
					processSentenceFunction = &processSentenceOperation;

					// Cleanout the collection to free up memory
					teachInRecords.clear();
					currTeachInRecord = teachInRecords.end();

					return;
				}

				// Append an initialized record at the end
				currTeachInRecord = teachInRecords.emplace(teachInRecords.end());

				currTeachInRecord->cycleStart = currTime;
				currTeachInRecord->recordNo = teachInRecords.size() - 1;

				// I am progressing from a previous real timestamp to the next one.
				// This means this is really the start of a new cycle
				teachInStarted = true;
				LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Last timestamp = " << currSequenceTimestampMS
						<< " New timestamp = " << thisGPSTimeStampMS
						<< ". Teach-in in progress. New numTeachInCyclesExecuted = " << teachInRecords.size());
			}
			// this is the start of a new cycle
			currSequenceTimestampMS = thisGPSTimeStampMS;
			// Store the local timestamp as start of the new cycle

		}

		if (teachInStarted) {
		TeachInCollection& currCollection = *currTeachInRecord;

			LOG4CXX_DEBUG(logger, "processSentenceTeachIn: Add message " << newSentence.sentenceType
					<< " to collection #"  << teachInRecords.size());

			locRecord.timeAfterCycleStart = currTime - currCollection.cycleStart;
			currCollection.records.insert(currCollection.records.end(),locRecord);
		}
	}
}

inline void NMEASet::finishTeachIn() {

	CommonCycleList commonCycles;

	CommonRecordItem commonRecord;
	TeachInCollectionIter currTeachInCollectionIter =  teachInRecords.begin();

	// If there are no recoded records the behavior would be undefined.
	if (currTeachInCollectionIter == teachInRecords.end()) {
		return;
	}

	commonRecord.recordNo = 0;
	commonRecord.teachInCollectionPtr = &(*currTeachInCollectionIter);
	// The first record is by definition always the first template.
	commonCycles.emplace_front();
	commonCycles.begin()->commonRecordItems.insert(commonCycles.begin()->commonRecordItems.cend(),commonRecord);


	for (currTeachInCollectionIter++; currTeachInCollectionIter != teachInRecords.end(); currTeachInCollectionIter++) {
		bool foundRecord = false;
		TeachInCollection *newRecord = nullptr;

		// Compare the cycles. Run through the previous collected common sets.
		for (CommonCycleListIter k = commonCycles.begin(); k != commonCycles.end(); k++) {
			newRecord = &(*currTeachInCollectionIter);
			TeachInCollection *refRecord = k->commonRecordItems.begin()->teachInCollectionPtr;
			if (newRecord->records.size() == refRecord->records.size()) {
				TeachInRecordIter newSentenceTypeIter = newRecord->records.begin();
				TeachInRecordIter refSentenceTypeIter = refRecord->records.begin();
				foundRecord = true;
				for (;newSentenceTypeIter != newRecord->records.end() && refSentenceTypeIter != refRecord->records.end();
						newSentenceTypeIter++,refSentenceTypeIter++) {
					if (newSentenceTypeIter->recordType != refSentenceTypeIter->recordType) {
						foundRecord = false;
						break;
					}
				} // for (uint32_t l = 0; l < newRecord.numInCollection; l++)
			} // if (newRecord.numInCollection == refRecord.numInCollection)

			if (foundRecord) {
				LOG4CXX_DEBUG(logger,
						"finishTeachIn: Collection #" << newRecord->recordNo << " joins common record #" << k->recordNo
						<< " as member #" << k->commonRecordItems.size());

				// Store the pointer to the current collection
				CommonRecordItemIter newCommonRecord = k->commonRecordItems.emplace(k->commonRecordItems.cend());
				newCommonRecord->recordNo = k->commonRecordItems.size() -1;
				newCommonRecord->teachInCollectionPtr = newRecord;

				// And finish the search for the current record because I had a hit.
				break;
			}

		} // for (uint32_t k = 0; commonRecords[k].numEqualRecords != 0; k++)

		if (!foundRecord && newRecord != nullptr) {
			// This collection defines a new template.
			CommonCycleListIter newCommonCycle = commonCycles.emplace(commonCycles.cend());
			newCommonCycle->recordNo = commonCycles.size() - 1;
			CommonRecordItemIter newCommonRecordItem = newCommonCycle->commonRecordItems.emplace(newCommonCycle->commonRecordItems.cend());
			newCommonRecordItem->recordNo = 0;
			newCommonRecordItem->teachInCollectionPtr = newRecord;
			LOG4CXX_DEBUG(logger,"finishTeachIn: Add collection #" << newRecord->recordNo
					<< " as new template #" << newCommonCycle->recordNo);
		}
	} // for (uint32_t i = 1; i < numTeachInCycles; i++)

	// Finally determine the dominant set of MNEA sentence types which the GNSS receiver sends per update cycle.
	NMEASet::determineNMEASet(commonCycles);
}


void NMEASet::determineNMEASet(CommonCycleList& commonCycles) {

	CommonCycleListIter usedCommonSequence = commonCycles.begin();
	// First determine the one set with more then 50% of occurrences.
	for (; usedCommonSequence != commonCycles.end();usedCommonSequence++){
		if (usedCommonSequence->commonRecordItems.size() > (numTeachInCycles/2)) {
			// This is the one. There can not be one with more entries
			// Because this one has already more than half.
			LOG4CXX_DEBUG(logger,"determineNMEASet: Found dominant common sequence # " << usedCommonSequence->recordNo
					<< " with " << usedCommonSequence->commonRecordItems.size() << " equal sets.");
			break;
		}
	}

	// If I cannot determine the absolute winner try to determine the relative winner
	if (usedCommonSequence != commonCycles.end()) {
		usedCommonSequence = commonCycles.begin();
		for (CommonCycleListIter i = commonCycles.begin(); i != commonCycles.end();i++){
			if (usedCommonSequence->commonRecordItems.size() < i->commonRecordItems.size()) {
				usedCommonSequence = i;
			}
		LOG4CXX_DEBUG(logger,"determineNMEASet: The major common sequence is #" << usedCommonSequence->recordNo
				<< ". It has " << usedCommonSequence->commonRecordItems.size() << " equal sets.");
		}
	}

	if (usedCommonSequence->commonRecordItems.size() < 3) {
		LOG4CXX_WARN(logger,"determineNMEASet: Could not find a common sequence shared by at least 3 cycles. Go by individual cycle analysis.");
	} else {
		// Use a teach-in record to collect which flags have been set so far by NMEA sentences.
		TeachInRecord collectiveFlags;

		auto usedTeachInCollection = usedCommonSequence->commonRecordItems.begin()->teachInCollectionPtr;

		for (auto i = usedTeachInCollection->records.begin();i != usedTeachInCollection->records.end(); i++) {
			TeachInRecord &currTeachInRecord = *i;
			// Check if the current record brings any new attributes.
			if (
					(currTeachInRecord.definesDifferentialMode && !collectiveFlags.definesDifferentialMode) ||
					(currTeachInRecord.definesMSL && !collectiveFlags.definesMSL) ||
					(currTeachInRecord.definesPosition && !collectiveFlags.definesPosition) ||
					(currTeachInRecord.definesQualitiyLevel && !collectiveFlags.definesQualitiyLevel) ||

					// Here some priorization: When I have HDOP and VDOP or AbsErr I do not care about PDOP any more
					( !collectiveFlags.definesAbsErr &&
						(!collectiveFlags.definesHDop || !collectiveFlags.definesVDop) &&
						currTeachInRecord.definesPDop && !collectiveFlags.definesPDop
					) ||

					// Some either-ors: When I have HDOP and VDOP I do not need AbsErr
					( (!collectiveFlags.definesHDop || !collectiveFlags.definesVDop) &&
						(currTeachInRecord.definesAbsErr && !collectiveFlags.definesAbsErr)
					) ||
					// If I have AbsErr I not not need HDOP and PDOP any more.
					( !collectiveFlags.definesAbsErr &&
						((currTeachInRecord.definesHDop && !collectiveFlags.definesHDop) ||
						 (currTeachInRecord.definesVDop && !collectiveFlags.definesVDop)
						)
					)

				) {

#if defined HAVE_LOG4CXX_H
				if (logger->isDebugEnabled()) {
					std::ostringstream str;
					str << "determineNMEASet: Add " << currTeachInRecord.recordType << ". It defines";

					if (!collectiveFlags.definesAbsErr && currTeachInRecord.definesAbsErr){
						str << " AbsErr,";
					}
					if (!collectiveFlags.definesDifferentialMode && currTeachInRecord.definesDifferentialMode){
						str << " DifferentialMode,";
					}
					if (!collectiveFlags.definesHDop && currTeachInRecord.definesHDop){
						str << " HDop,";
					}
					if (!collectiveFlags.definesVDop && currTeachInRecord.definesVDop){
						str << " VDop,";
					}
					if (!collectiveFlags.definesPDop && currTeachInRecord.definesPDop){
						str << " PDop,";
					}
					if (!collectiveFlags.definesMSL && currTeachInRecord.definesMSL){
						str << " MSL,";
					}
					if (!collectiveFlags.definesPosition && currTeachInRecord.definesPosition){
						str << " Position,";
					}
					if (!collectiveFlags.definesQualitiyLevel && currTeachInRecord.definesQualitiyLevel){
						str << " QualityLevel";
					}
					LOG4CXX_DEBUG(logger,str.str());
				} // if (logger->isDebugEnabled())

#endif // #if defined HAVE_LOG4CXX_H

				// Set any new flags in the collective attribute set
				collectiveFlags.definesAbsErr 			|= currTeachInRecord.definesAbsErr;
				collectiveFlags.definesDifferentialMode	|= currTeachInRecord.definesDifferentialMode;
				collectiveFlags.definesHDop 			|= currTeachInRecord.definesHDop;
				collectiveFlags.definesMSL 				|= currTeachInRecord.definesMSL;
				collectiveFlags.definesPDop 			|= currTeachInRecord.definesPDop;
				collectiveFlags.definesPosition 		|= currTeachInRecord.definesPosition;
				collectiveFlags.definesQualitiyLevel 	|= currTeachInRecord.definesQualitiyLevel;
				collectiveFlags.definesVDop 			|= currTeachInRecord.definesVDop;

				// This record brings a new aspect into my life. Store it.
				usedNMEASentenceTypes.insert(usedNMEASentenceTypes.cend(),currTeachInRecord.recordType);
			} // if ( relevant attributes are defined which were not defined before.
		} // for (uint32_t i = 0;i < usedTeachInCollection.numInCollection; i++)
	} // if (commonRecords[usedCommonSequence].numEqualRecords < 3) {...} else {
}

void NMEASet::processSentenceOperation(
		const NMEASentence &newSentence) {

	LOG4CXX_DEBUG(logger,"processSentenceOperation: Message " << newSentence.talkerID << ' ' << newSentence.sentenceType);

}

} /* namespace openEV */
