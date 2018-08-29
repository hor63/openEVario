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

#include "OEVCommon.h"

// included here because the defines above are used in the include
#include "drivers/GliderVarioDriverBase.h"
#include "IGCReaderLib.h"
namespace openEV {

/** \brief Simulation/test driver playing back an IGC file
 *
 * This driver reads an IGC file with a flight record, and plays it back in real-time.
 * This means that the Kalman filter updates occur with the exact intervals as the B (GPS fix) records were recorded in the IGC file
 * The IGC file format is defined by the FAI (FÉDÉRATION AÉRONAUTIQUE INTERNATIONALE) INTERNATIONAL GLIDING COMMISSION.
 *
 * Please find the latest version of the specification at <a href="https://www.fai.org/igc-documents" >IGC documents</a>.
 * There look for FLIGHT RECORDERS->"IGC-approved Flight Recorders - Technical Specification".
 *
 * The driver reads the I and B and K records.
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
 * See <a href="https://de.wikipedia.org/wiki/Barometrische_H%C3%B6henformel#Internationale_H%C3%B6henformel" >Wikipedia: Internationale Höhenformel</a>
 * or <a href="https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations" >Wikipedia: Barometric formula</a>
 */
class IGCReaderDriver  : public GliderVarioDriverBase {
public:
	IGCReaderDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: GliderVarioDriverBase {driverName,description,instanceName,IGCReaderLib::theOneAndOnly}
	{

	}
	virtual ~IGCReaderDriver();

    /**
     * Initialize the driver
     */
    virtual void driverInit() override;


    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

    virtual void initializeStatus(GliderVarioStatus &varioStatus) override;

    virtual void start(GliderVarioMainPriv *varioMain) override;

    virtual void stop() override;

    virtual void suspend() override;

    virtual void resume() override;

    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

};

} /* namespace OevGLES */
#endif /* IGCREADERDRIVER_H_ */

