/*
 * crc.h
 *
 *  Created on: May 1, 2020
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#ifndef UTIL_CRC_H_
#define UTIL_CRC_H_

#include "OEVCommon.h"

OEV_UTILS_PUBLIC uint16_t crc16CCIT(uint16_t crc, const void* const block,uint16_t len);

/*
 * Values for FCS calculations.
 */
#define PPP_INITFCS     0xffffU  /* Initial FCS value */
#define PPP_GOODFCS     0xf0b8U  /* Good final FCS value */



#endif /* UTIL_CRC_H_ */
