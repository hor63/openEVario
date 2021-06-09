/*
 * MPU9150Lib.h
 *
 *  Created on: Jun 07, 2021
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
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

#ifndef DRIVERS_MPU9150_MPU9150LIB_H_
#define DRIVERS_MPU9150_MPU9150LIB_H_

#include "drivers/GliderVarioDriverLibBase.h"

namespace openEV::drivers::TDK_MPU9150 {

class MPU9150Lib : public GliderVarioDriverLibBase {
public:

	virtual void addDrivers(GliderVarioDriverList &gliderVarioDriverList) override;

	static MPU9150Lib theOneAndOnly;


// no one shall create instances except the one and only static instance
private:
	MPU9150Lib();
	virtual ~MPU9150Lib();


};

} /* namespace openEV */

#endif /* DRIVERS_MPU9150_MPU9150LIB_H_ */
