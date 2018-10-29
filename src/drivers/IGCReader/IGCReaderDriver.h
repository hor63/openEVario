/*
 * IGCReaderDriver.h
 *
 *  Created on: Aug 15, 2018
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2018  Kai Horstmann
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

#ifndef IGCREADERDRIVER_H_
#define IGCREADERDRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "OEVCommon.h"

#include "drivers/GliderVarioDriverBase.h"
#include "IGCReaderLib.h"
#include "BRecord.h"
#include "BRecordSection.h"

namespace openEV {

/** \brief Simulation/test driver playing back an IGC file
 *
 * This driver reads an IGC file with a flight record, and plays it back in real-time.
 * This means that the Kalman filter updates occur with the exact intervals as the B (GPS fix) records were recorded in the IGC file
 * The IGC file format is defined by the FAI (FÉDÉRATION AÉRONAUTIQUE INTERNATIONALE) INTERNATIONAL GLIDING COMMISSION.
 *
 * Please find the latest version of the specification at <a href="https://www.fai.org/igc-documents" >IGC documents</a>.
 * There look for FLIGHT RECORDERS->"IGC-approved Flight Recorders - Technical Specification".
 * The document version used for this driver is
 * <a href="https://www.fai.org/sites/default/files/documents/igc_fr_spec_with_al4a_2016-4-10.pdf">Second Edition with Amendment 4a</a>
 *
 * The driver reads the I and B records.
 * It updates these measurements in the Kalman filter:
 * - GPS coordinates
 * - GPS altitude
 * - Static Pressure (calculated back from the pressure altitude in the record based on the standard atmosphere)
 *
 * The following measurements are updated when they are provided in the B records
 * - GPS direction (optional)
 * - Speed over ground (optional)
 *
 * Static pressure is calculated back from the pressure altitude in the B records according to the ICAO standard atmosphere.
 * \see <a href="https://de.wikipedia.org/wiki/Barometrische_H%C3%B6henformel#Internationale_H%C3%B6henformel" >Wikipedia: Internationale Höhenformel</a>
 * or <a href="https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations" >Wikipedia: Barometric formula</a>
 */
class IGCReaderDriver  : public GliderVarioDriverBase {
public:

	IGCReaderDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~IGCReaderDriver();

    /**
     * Initialize the driver
     */
    virtual void driverInit() override;


    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMainPriv &varioMain) override;

    virtual void suspend() override;

    virtual void resume() override;

    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

protected:

    /** \brief Length of the line buffer.
     *
     * According to the IGC specification section A2.1 "File Structure" a line should never be longer than
     * 76 char. So this size should be plenty
     */
    static int constexpr lineBufSize = 256;

    /// Latest line of the IGC file read. The buffer is 0-terminated.
    char lineBuffer[lineBufSize] = "";
    /// Number of valid characters in lineBuffer.
    int lineLen = 0;
    int lineNum = 0;

    std::string igcFileName;
    std::ifstream igcFile;

    /// \brief Timestamp of the first B-record in seconds after midnight UTC.
    double startTimeDay = 0.0;

    /// All B-records (GPS fixes and pressure measurements)
    typedef std::map<OEVDuration,BRecord> BRecordsMap;
    typedef BRecordsMap::value_type BRecordsValue;
    typedef BRecordsMap::const_iterator BRecordsCIter;
    typedef BRecordsMap::iterator BRecordsIter;
    BRecordsMap bRecords;

    /// \brief Processes I abd B records of the IGC file
    BRecordSectionProcessor bRecordSectionProcessor;

    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction
     *
     */
    void driverThreadFunction() override;


    /** \brief Opens the IGC file if it has not been opened before
     *
     * If the file is open do nothing.
     */
    void openIGCFile();

    /** \brief Close the IGC file when it was open before.
     *
     * If the file is closed do nothing.
     */
    void closeIGCFile();

    /** \brief Read one line from the IGC file
     *
     * Read a line of \ref igcFile into the line buffer \ref lineBuffer.
     * lineBuffer is 0-terminated, and the number of valid characters excluding the terminating 0
     * stored in \ref lineLen.
     * If
     *
     * @return true: \ref lineBuffer contains a valid new line. False: File end or read error.
     */
    bool readLine();

    /** \brief Read an entire IGC file. Process I and B records, and store the B records in \ref bRecords.
     *
     * Read the file calling \ref readLine, and call
     *
     */
    void readIGCFile ();

};

} /* namespace openEV */
#endif /* IGCREADERDRIVER_H_ */

