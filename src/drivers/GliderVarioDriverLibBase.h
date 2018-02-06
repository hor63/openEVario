/*
 * GliderVarioDriverLibBase.h
 *
 *  Created on: Feb 6, 2018
 *      Author: hor
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

#ifndef DRIVERS_GLIDERVARIODriverLIBBASE_H_
#define DRIVERS_GLIDERVARIODriverLIBBASE_H_

#include <memory>

#include "OEVCommon.h"

namespace openEV {

/** \brief Driver library base class.
 *
 * This is the base class for the driver library class. Each driver shared library returns an object of this class.
 * Primarily it enumerates the drivers of the library. One shared library may support several drivers.
 * Examples are the MPU-9x50 from TDK, formerly Invensense (MPU-9250, MPU-9150) which can are very similar.
 *
 */
class OEV_UTILS_PUBLIC GliderVarioDriverLibBase {
public:
	GliderVarioDriverLibBase() {
		// TODO Auto-generated constructor stub

	}
	virtual ~GliderVarioDriverLibBase();
};

typedef std::shared_ptr<GliderVarioDriverLibBase> GliderVarioDriverLibBasePtr;

} /* namespace openEV */

#endif /* DRIVERS_GLIDERVARIODriverLIBBASE_H_ */
