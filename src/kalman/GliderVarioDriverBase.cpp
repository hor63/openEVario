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


#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioDriverBase::_SensorCapability e) {
	std::ostream &o = b;
	return operator << (o,e.e);
}
#endif /* #if defined HAVE_LOG4CXX_H */

