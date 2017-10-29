/*
 * sensorDriver.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *  Common definitions for building shared libraries
 *
 *
 *  Definition of class FastMath
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
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

#ifndef SENSORDRIVER_H_
#define SENSORDRIVER_H_

#include <memory>

#include "OEVCommon.h"
#include "drivers/GliderVarioDriverBase.h"

// If used stand-alone, and not within a driver
#if !defined(DRIVER_PUBLIC)
  #define DRIVER_PUBLIC DLL_IMPORT
#endif

#if !defined(DRIVER_LOCAL)
  #define DRIVER_LOCAL  DLL_LOCAL
#endif

namespace openEV {

/// Define a shared pointer to the driver object which keeps a reference count
typedef std::shared_ptr<openEV::GliderVarioDriverBase> GliderVarioDriverBasePtr;

}


extern "C" {
/// Do whatever initialization is required
int DRIVER_PUBLIC driverInit(void);


/// Return a pointer to a driver object. The object must be created with the operator 'new'.
openEV::GliderVarioDriverBasePtr getDriver();


}

#endif /* SENSORDRIVER_H_ */
