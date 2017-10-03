/*
 * simulDriver.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *  Internal definitions of the simulation driver
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

#ifndef SIMULDRIVER_H_
#define SIMULDRIVER_H_

#include "OEVCommon.h"


#if defined (BUILDING_SIMUL_DRIVER)
  #define DRIVER_PUBLIC DLL_EXPORT
  #define DRIVER_LOCAL  DLL_LOCAL
#else /* BUILDING_OEV_KALMAN */
  #define DRIVER_PUBLIC DLL_IMPORT
  #define DRIVER_LOCAL  DLL_LOCAL
#endif /* BUILDING_OEV_KALMAN */

// included here because the defined above are used in the include
#include "drivers/sensorDriver.h"

#endif /* SIMULDRIVER_H_ */

