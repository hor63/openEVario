/*
 * sensorDriver.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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

#include "OEVCommon.h"
#include "drivers/DriverLibBase.h"


extern "C" {
/** \brief Do whatever initialization is required
 *
 * This function must tolerate to be called multiple times without adverse effects.
 *
 */
void OEV_DRIVER_PUBLIC driverLibInit(void);


/** \brief Return a pointer to a driver library. The object is static in the respective driver library, and must never be deleted by the caller.
 *
 * @return Pointer to the driver library object
 */
openEV::drivers::GliderVarioDriverLibBasePtr OEV_DRIVER_PUBLIC getDriverLib();


}

#endif /* SENSORDRIVER_H_ */
