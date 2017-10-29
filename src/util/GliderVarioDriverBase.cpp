/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: kai_horstmann
 */


#define BUILDING_OEV_DRIVER 1

#include "OEVCommon.h"
#include "drivers/GliderVarioDriverBase.h"

namespace openEV {

GliderVarioDriverBase::SensorCapabilityHelperClass GliderVarioDriverBase::SensorCapabilityHelperObj;

}

std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind) {
	o << openEV::GliderVarioDriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}
