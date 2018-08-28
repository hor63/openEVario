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
