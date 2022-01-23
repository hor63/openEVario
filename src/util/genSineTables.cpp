/** \brief Generate constant sinus tables for FastMath
 * genSineTables.cpp
 *
 *  Created on: Dec 27, 2015
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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <iostream>
#include <cstdio>
#include <cstring>

#include "util/FastMath.h"

/// Print the sine table into a c++ source file "fileName".
static int printSineTable(const char* fileName){
    FILE* fastMathSineTable = fopen(fileName,"w");
    int i;

    if (!fastMathSineTable) {
        std::cerr << "Cannot open file \"" << fileName << "\" for writing\n" << std::endl;
        return 2;
    }

    // print the header of the file, and the definition of the sine table FastMath::sinusTable
    fprintf(fastMathSineTable,"%s",
            "/*\n\
 *   This file is part of openEVario, an electronic variometer for glider planes\n\
 *   Copyright (C) 2016  Kai Horstmann\n\
 *\n\
 *   This program is free software; you can redistribute it and/or modify\n\
 *   it under the terms of the GNU General Public License as published by\n\
 *   the Free Software Foundation; either version 2 of the License, or\n\
 *   any later version.\n\
 *\n\
 *   This program is distributed in the hope that it will be useful,\n\
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
 *   GNU General Public License for more details.\n\
 *\n\
 *   You should have received a copy of the GNU General Public License along\n\
 *   with this program; if not, write to the Free Software Foundation, Inc.,\n\
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.\n\
 */\n\
\n\
#ifdef HAVE_CONFIG_H\n\
#  include \"config.h\"\n\
#endif\n\
\n\
#include \"FastMath.h\" \n\
\n\
namespace openEV \n\
{\n\
  \n\
  /**\n\
   * Generated by genSineTables.cpp.\n\
   */\n\
const double FastMath::sinusTable [FastMath::sizeSineTable + 1] = {\
\n	0.0"
    );

    // Now print the sine values in 1/8 degrees.
    // always use the 1st quadrant because the approximations are usually more accurate closer to 0
    // do *not* print the 0 value. That is already printed as static text
    // First quadrant: sin as it comes
    for (i = 1 ; i < openEV::FastMath::sizeSineTable/4 ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                sin(static_cast<double>(i) * 360.0 / static_cast<double>(openEV::FastMath::sizeSineTable)*openEV::FastMath::degToRad)
        );
    }
    // Second quadrant: sin (180deg-angle)
    for (i = openEV::FastMath::sizeSineTable/4 ; i < openEV::FastMath::sizeSineTable/2 ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                sin(static_cast<double>(openEV::FastMath::sineSamplesPerDegree*180 - i) / static_cast<double>(openEV::FastMath::sineSamplesPerDegree)*openEV::FastMath::degToRad)	      );
    }
    // Third quadrant: -sin (angle-180)
    for (i = openEV::FastMath::sizeSineTable/2 ; i < 3*openEV::FastMath::sizeSineTable/4 ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                -sin(static_cast<double>(i - openEV::FastMath::sineSamplesPerDegree*180) / static_cast<double>(openEV::FastMath::sineSamplesPerDegree)*openEV::FastMath::degToRad)	      );
    }

    // Fourth quadrant: -sin (360- angle)
    for (i = 3*openEV::FastMath::sizeSineTable/4 ; i <= openEV::FastMath::sizeSineTable ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                -sin(static_cast<double>(openEV::FastMath::sineSamplesPerDegree*360 - i) / static_cast<double>(openEV::FastMath::sineSamplesPerDegree)*openEV::FastMath::degToRad)	      );
    }

    // print the closure of the definition of FastMath::sinusTable
    fprintf(fastMathSineTable,"\n%s",
            "};\n\
\n" );


    // Print the ATAN2 table
    fprintf(fastMathSineTable,"\n%s",
            "const double FastMath::atanTable [FastMath::sizeATanTable + 1] = {\
  \n    0.0"
    );

    // Now print the atan values in 256 steps from 0 - 45 deg.
    // do *not* print the 0 value. That is already printed as static text
    // First quadrant: sin as it comes
    for (i = 1 ; i <= openEV::FastMath::sizeATanTable ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                atan(static_cast<double>(i)/static_cast<double>(openEV::FastMath::sizeATanTable))*openEV::FastMath::radToDeg
        );
    }

    // print the closure of the definition of FastMath::atanTable
    fprintf(fastMathSineTable,"\n%s",
            "};\n\
\n\
" );

    // print the ASIN table
    fprintf(fastMathSineTable,"\n%s",
            "const double FastMath::asinTable [FastMath::sizeASinTable + 1] = {\
  \n    0.0"
    );

    // Now print the asin values in 512 steps from 0 - 90 deg.
    // do *not* print the 0 value. That is already printed as static text
    // First quadrant: sin as it comes
    for (i = 1 ; i <= openEV::FastMath::sizeASinTable ; i++ ) {
        fprintf(fastMathSineTable,",\n    %.20f",
                asin(static_cast<double>(i)/static_cast<double>(openEV::FastMath::sizeASinTable))*openEV::FastMath::radToDeg
        );
    }

    // print the closure of the definition of FastMath::atanTable
    fprintf(fastMathSineTable,"\n%s",
            "};\n\
\n\
}\
" );

    return  0;
}

static void usage () {
    std::cout << "Usage: genSineTables <name of generated C++ source file" << std::endl;
}

int main (int argc, const char** argv) {

    printf ("argc= %d\n",argc);
    if (argc != 2) {
        usage();
        return 1;
    }
    if (!strcmp(argv[1],"?") || !strcmp(argv[1],"-?") || !strcmp(argv[1],"--help") ){
        usage();
        return 0;
    }

    return printSineTable(argv[1]);
}
