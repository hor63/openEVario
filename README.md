# openEVario
Electronic variometer application using inertial and pressure sensors (typically I2C on ARM SoC) and GPS input. 

The system is slowly coming together.
Static tests with all sensors connected are finished. The system works statisfactory, and stabilizes even after shaking and twisting it hard.

There are multiple drivers for sensors available:
- GPS connected by Serial line, USB serial device, or Serial by Bluetooth.
  Protocol is NMEA 0183. The driver uses whatever sentences the device sends, and uses the best fit of sentences.
- IMU: Accelerometer, Gyroscope, and magnetometers:
    - Bosch BMX160. Unfortunately already obsolete.  
      I designed a [detached IMU sensor board](https://github.com/hor63/hovImuBoard.git) around a Bosch BMX160 IMU with KiCad. [The firmware is here](https://github.com/hor63/horOvIp-I2C-Bridge/tree/with-FreeRTOS-UDP).   
    - MPU9150. Also obsolete. This IMU is common in legacy OpenVario sensor boards.
- Barometric sensors
    - MPL3115A2
	 - A whole slew of TE Connectivity MEAS (Measurement Specialties) with 24-bit A/D converter which include:
	     - TE-MEAS-MS5607
	     - TE-MEAS-MS5611 This is the sensor used on OpenVario sensor boards.
	     - TE-MEAS-MS5637
	     - TE-MEAS-MS5803
	     - TE-MEAS-MS5805
	     - TE-MEAS-MS5837
	     - TE-MEAS-MS5839
	     - TE-MEAS-MS5840
- Differential pressure sensors
     - MS4515DO: Here are multiple sub-types with different ranges supported.
     - AMS5915 (un-tested) This is the sensor used on OpenVario sensor boards.

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

