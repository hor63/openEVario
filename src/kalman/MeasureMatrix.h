/*
 * MeasureMatrix.h
 *
 *  Created on: Dec 24, 2015
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
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

#ifndef MEASUREMATRIX_H_
#define MEASUREMATRIX_H_

#include "OEVCommon.h"

namespace openEV
{

class OEV_PUBLIC MeasureMatrix
{
public:
    MeasureMatrix ();
    virtual
    ~MeasureMatrix ();
};

} /* namespace openEV */

#endif /* MEASUREMATRIX_H_ */
