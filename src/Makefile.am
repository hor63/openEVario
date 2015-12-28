bin_PROGRAMS=openEVario genSineTables
openEVario_SOURCES=openEVario.cpp GliderVarioStatus.cpp GliderVarioTransitionMatrix.cpp RotationMatrix.cpp FastMath.cpp FastMathSineTable.cpp MeasureMatrix.cpp
genSineTables_SOURCES=genSineTables.cpp

AM_CXXFLAGS="-I$(top_srcdir)/extern/eigen3.2.7/Eigen"

FastMathSineTable.cpp: genSineTables
	./genSineTables FastMathSineTable.cpp
	