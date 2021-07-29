/*
 * IMUBase.cpp
 *
 *  Created on: Jul 29, 2021
 *      Author: hor
 */

#include "util/drivers/IMUBase.h"

namespace openEV {
namespace drivers {

IMUBase::IMUBase(
		char const *driverName,
		char const *description,
		char const *instanceName,
		GliderVarioDriverLibBase &driverLib
		)
: GliderVarioDriverBase {driverName,description,instanceName,driverLib}
{
	// TODO Auto-generated constructor stub

}

IMUBase::~IMUBase() {
	// TODO Auto-generated destructor stub
}

} /* namespace drivers */
} /* namespace openEV */
