#  Created on: Apr 21 2021
#      Author: hor
#
#    This file is part of openEVario, an electronic variometer for glider planes
#    Copyright (C) 2021  Kai Horstmann
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License along
#    with this program; if not, write to the Free Software Foundation, Inc.,
#    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

SUBDIRS=util kalman main drivers/simul drivers/IGCReader drivers/BoschBMXSensorBoard drivers/NmeaGPS

if BUILD_I2C_PORT_DRIVER
SUBDIRS+=drivers/MPL3115 drivers/MS4515 drivers/AMS5915 drivers/TE-MEAS-AbsPressure drivers/MPU-9150
endif

bin_PROGRAMS=openEVario$(EXEEXT)

openEVario_SOURCES=openEVario.cpp 
 
openEVario_LDADD=main/libopenEVMain.la util/libopenEVUtil.la $(LOG4CXX_LIBS) $(PTHREAD_LIBS)

AM_CXXFLAGS = -I$(top_srcdir)/src -isystem $(top_srcdir)/3rdParty/eigen -I$(top_srcdir)/3rdParty/fmt/include \
	$(LOG4CXX_CXXFLAGS) $(PTHREAD_CFLAGS)

AM_LDFLAGS=$(LOG4CXX_LDFLAGS)

all: $(abs_builddir)/openEVario.properties

$(abs_builddir)/openEVario.properties: $(srcdir)/openEVario.properties
	cp $(srcdir)/openEVario.properties $(abs_builddir)/openEVario.properties

all: $(abs_builddir)/openEVario.logger.properties

$(abs_builddir)/openEVario.logger.properties: $(srcdir)/openEVario.logger.properties
	cp $(srcdir)/openEVario.logger.properties $(abs_builddir)/openEVario.logger.properties

all: $(abs_builddir)/BMX160SensorBoardCalib.properties

$(abs_builddir)/BMX160SensorBoardCalib.properties: $(srcdir)/BMX160SensorBoardCalib.properties
	cp $(srcdir)/BMX160SensorBoardCalib.properties $(abs_builddir)/BMX160SensorBoardCalib.properties

all: $(abs_builddir)/MPU9150Calib.properties

$(abs_builddir)/MPU9150Calib.properties: $(srcdir)/MPU9150Calib.properties
	cp $(srcdir)/MPU9150Calib.properties $(abs_builddir)/MPU9150Calib.properties
	