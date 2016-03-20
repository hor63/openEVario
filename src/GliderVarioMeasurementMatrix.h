/*
 * GliderVarioMeasurementMatrix.h
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 */

#ifndef GLIDERVARIOMEASUREMENTMATRIX_H_
#define GLIDERVARIOMEASUREMENTMATRIX_H_

#include "GliderVarioStatus.h"
#include "GliderVarioMeasurementVector.h"

namespace openEV {

/***
 * Measurement matrix h of a Kalman filter. Defines the factor to calculate measurement values z from the status vector x. So z = h * x.
 */
class GliderVarioMeasurementMatrix {
public:
	GliderVarioMeasurementMatrix();
	virtual ~GliderVarioMeasurementMatrix();

	/// Multiplication matrix. Dimensions come directly from the status and measurement vector sizes.
	typedef Eigen::Matrix<FloatType,GliderVarioMeasurementVector::MEASURE_NUM_ROWS,GliderVarioStatus::STATUS_NUM_ROWS> MeasureMatrixType;

	/**
	 *
	 * @return reference to the internal matrix for direct matrix manipulation, or arithmetic operations.
	 */
	MeasureMatrixType const &getMeasureMatrix() const {
		return measurementMatrix;
	}

	/***
	 * \todo Calculation of normalized magnetic values from Euler angles of current attitude
	 * Calculates the current calculation matrix from the time difference since the last update, and the latest status (for coordinate conversion).
	 * @param[in] timeDiff
	 * @param[in] lastStatus
	 */
    void
    calcMeasurementMatrix (
	    FloatType timeDiff,
	    GliderVarioStatus const &lastStatus);


protected:
	MeasureMatrixType measurementMatrix;

};

} /* namespace openEV */

#endif /* GLIDERVARIOMEASUREMENTMATRIX_H_ */
