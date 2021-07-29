/*
 * MS4515Lib.h
 *
 *  Created on: Jan 17 2021
 *      Author: hor
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

#ifndef DRIVERS_MS4515_MS4515LIB_H_
#define DRIVERS_MS4515_MS4515LIB_H_

#include "drivers/DriverLibBase.h"

namespace openEV::drivers::MS4515 {

class MS4515Lib : public DriverLibBase {
public:

	virtual void addDrivers(GliderVarioDriverList &gliderVarioDriverList) override;

	static MS4515Lib theOneAndOnly;


// no one shall create instances except the one and only static instance
private:
	MS4515Lib();
	virtual ~MS4515Lib();


};

} /* namespace openEV */

#endif /* DRIVERS_MS4515_MS4515LIB_H_ */
