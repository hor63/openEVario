/*
 * IMUBase.h
 *
 *  Created on: Jul 29, 2021
 *      Author: hor
 */

#ifndef UTIL_DRIVERS_IMUBASE_H_
#define UTIL_DRIVERS_IMUBASE_H_

#include "drivers/DriverBase.h"

namespace openEV {
namespace drivers {

class IMUBase: public DriverBase {
public:
	IMUBase(
			char const *driverName,
			char const *description,
			char const *instanceName,
			DriverLibBase &driverLib
			);
	virtual ~IMUBase();
};

} /* namespace drivers */
} /* namespace openEV */

#endif /* UTIL_DRIVERS_IMUBASE_H_ */
