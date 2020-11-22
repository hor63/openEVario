# openEVario
Electronic variometer application using inertial and pressure sensors (typically I2C on ARM SoC) and GPS input. 

The system is slowly coming together.
 
I designed a [detached IMU sensor board](https://github.com/hor63/hovImuBoard.git) around a Bosch BMX160 IMU with KiCad.   
[The firmware is here](https://github.com/hor63/horOvIp-I2C-Bridge/tree/with-FreeRTOS-UDP).   
The driver is "BMX160SensorBoard", and ready for real-life and integration testing.

The driver for GNSS (GPS) receivers with NMEA0813 protocol is "NmeaGPS". It is currently in development.

Next and finally drivers for absolute and differential pressure sensors will be coming.

Save your bandwidth downloading it unless you are interested in an ongoing work-in-progress :upside_down_face:

    This file is part of openEVario, an electronic variometer for glider planes
    Copyright (C) 2016-2020  Kai Horstmann

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

