/*
 * MPU-9150Defs.h
 *
 *  Created on: Jun 09,2021
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario,an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License,or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not,write to the Free Software Foundation,Inc.,
 *   51 Franklin Street,Fifth Floor,Boston,MA 02110-1301 USA.
 *
 */

#ifndef DRIVERS_MPU_9150_MPU_9150DEFS_H_
#define DRIVERS_MPU_9150_MPU_9150DEFS_H_

namespace openEV::drivers::TDK_MPU9150 {

	/** \brief I2C address of the MPU-9150.
	 *
	 * This is the default address of the MPU-9150 when the address pin AD0 is pulled to ground.
	 * Else it is 0x69.
	 */
	static constexpr uint8_t MPU_9150_I2CAddr = 0x68;

	/** \brief MPU-9150 register map
	 *
	 * Names and values were taken verbatim from the \n
	 * MPU-9150 Register Map and Descriptions, Revision 4.2
	 */
	enum MPU_9150_REGMAP {
		REG_9150_SELF_TEST_X		= 13,       /**< REG_9150_SELF_TEST_X */
		REG_9150_SELF_TEST_Y		= 14,       /**< REG_9150_SELF_TEST_Y */
		REG_9150_SELF_TEST_Z		= 15,       /**< REG_9150_SELF_TEST_Z */
		REG_9150_SELF_TEST_A		= 16,       /**< REG_9150_SELF_TEST_A */
		REG_9150_SMPLRT_DIV			= 25,       /**< REG_9150_SMPLRT_DIV */
		REG_9150_CONFIG				= 26,          /**< REG_9150_CONFIG */
		REG_9150_GYRO_CONFIG		= 27,       /**< REG_9150_GYRO_CONFIG */
		REG_9150_ACCEL_CONFIG		= 28,      /**< REG_9150_ACCEL_CONFIG */
		REG_9150_FIFO_EN			= 35,          /**< REG_9150_FIFO_EN */
		REG_9150_I2C_MST_CTRL		= 36,      /**< REG_9150_I2C_MST_CTRL */
		REG_9150_I2C_SLV0_ADDR		= 37,     /**< REG_9150_I2C_SLV0_ADDR */
		REG_9150_I2C_SLV0_REG		= 38,      /**< REG_9150_I2C_SLV0_REG */
		REG_9150_I2C_SLV0_CTRL		= 39,     /**< REG_9150_I2C_SLV0_CTRL */
		REG_9150_I2C_SLV1_ADDR		= 40,     /**< REG_9150_I2C_SLV1_ADDR */
		REG_9150_I2C_SLV1_REG		= 41,      /**< REG_9150_I2C_SLV1_REG */
		REG_9150_I2C_SLV1_CTRL		= 42,     /**< REG_9150_I2C_SLV1_CTRL */
		REG_9150_I2C_SLV2_ADDR		= 43,     /**< REG_9150_I2C_SLV2_ADDR */
		REG_9150_I2C_SLV2_REG		= 44,      /**< REG_9150_I2C_SLV2_REG */
		REG_9150_I2C_SLV2_CTRL		= 45,     /**< REG_9150_I2C_SLV2_CTRL */
		REG_9150_I2C_SLV3_ADDR		= 46,     /**< REG_9150_I2C_SLV3_ADDR */
		REG_9150_I2C_SLV3_REG		= 47,      /**< REG_9150_I2C_SLV3_REG */
		REG_9150_I2C_SLV3_CTRL		= 48,     /**< REG_9150_I2C_SLV3_CTRL */
		REG_9150_I2C_SLV4_ADDR		= 49,     /**< REG_9150_I2C_SLV4_ADDR */
		REG_9150_I2C_SLV4_REG		= 50,      /**< REG_9150_I2C_SLV4_REG */
		REG_9150_I2C_SLV4_DO		= 51,       /**< REG_9150_I2C_SLV4_DO */
		REG_9150_I2C_SLV4_CTRL		= 52,     /**< REG_9150_I2C_SLV4_CTRL */
		REG_9150_I2C_SLV4_DI		= 53,       /**< REG_9150_I2C_SLV4_DI */
		REG_9150_I2C_MST_STATUS		= 54,    /**< REG_9150_I2C_MST_STATUS */
		REG_9150_INT_PIN_CFG		= 55,       /**< REG_9150_INT_PIN_CFG */
		REG_9150_INT_ENABLE			= 56,       /**< REG_9150_INT_ENABLE */
		REG_9150_INT_STATUS			= 58,       /**< REG_9150_INT_STATUS */
		REG_9150_ACCEL_XOUT_H		= 59,      /**< REG_9150_ACCEL_XOUT_H */
		REG_9150_ACCEL_XOUT_L		= 60,      /**< REG_9150_ACCEL_XOUT_L */
		REG_9150_ACCEL_YOUT_H		= 61,      /**< REG_9150_ACCEL_YOUT_H */
		REG_9150_ACCEL_YOUT_L		= 62,      /**< REG_9150_ACCEL_YOUT_L */
		REG_9150_ACCEL_ZOUT_H		= 63,      /**< REG_9150_ACCEL_ZOUT_H */
		REG_9150_ACCEL_ZOUT_L		= 64,      /**< REG_9150_ACCEL_ZOUT_L */
		REG_9150_TEMP_OUT_H			= 65,       /**< REG_9150_TEMP_OUT_H */
		REG_9150_TEMP_OUT_L			= 66,       /**< REG_9150_TEMP_OUT_L */
		REG_9150_GYRO_XOUT_H		= 67,       /**< REG_9150_GYRO_XOUT_H */
		REG_9150_GYRO_XOUT_L		= 68,       /**< REG_9150_GYRO_XOUT_L */
		REG_9150_GYRO_YOUT_H		= 69,       /**< REG_9150_GYRO_YOUT_H */
		REG_9150_GYRO_YOUT_L		= 70,       /**< REG_9150_GYRO_YOUT_L */
		REG_9150_GYRO_ZOUT_H		= 71,       /**< REG_9150_GYRO_ZOUT_H */
		REG_9150_GYRO_ZOUT_L		= 72,       /**< REG_9150_GYRO_ZOUT_L */
		REG_9150_EXT_SENS_DATA_00	= 73,   /**< REG_9150_EXT_SENS_DATA_00 */
		REG_9150_EXT_SENS_DATA_01	= 74,   /**< REG_9150_EXT_SENS_DATA_01 */
		REG_9150_EXT_SENS_DATA_02	= 75,   /**< REG_9150_EXT_SENS_DATA_02 */
		REG_9150_EXT_SENS_DATA_03	= 76,   /**< REG_9150_EXT_SENS_DATA_03 */
		REG_9150_EXT_SENS_DATA_04	= 77,   /**< REG_9150_EXT_SENS_DATA_04 */
		REG_9150_EXT_SENS_DATA_05	= 78,   /**< REG_9150_EXT_SENS_DATA_05 */
		REG_9150_EXT_SENS_DATA_06	= 79,   /**< REG_9150_EXT_SENS_DATA_06 */
		REG_9150_EXT_SENS_DATA_07	= 80,   /**< REG_9150_EXT_SENS_DATA_07 */
		REG_9150_EXT_SENS_DATA_08	= 81,   /**< REG_9150_EXT_SENS_DATA_08 */
		REG_9150_EXT_SENS_DATA_09	= 82,   /**< REG_9150_EXT_SENS_DATA_09 */
		REG_9150_EXT_SENS_DATA_10	= 83,   /**< REG_9150_EXT_SENS_DATA_10 */
		REG_9150_EXT_SENS_DATA_11	= 84,   /**< REG_9150_EXT_SENS_DATA_11 */
		REG_9150_EXT_SENS_DATA_12	= 85,   /**< REG_9150_EXT_SENS_DATA_12 */
		REG_9150_EXT_SENS_DATA_13	= 86,   /**< REG_9150_EXT_SENS_DATA_13 */
		REG_9150_EXT_SENS_DATA_14	= 87,   /**< REG_9150_EXT_SENS_DATA_14 */
		REG_9150_EXT_SENS_DATA_15	= 88,   /**< REG_9150_EXT_SENS_DATA_15 */
		REG_9150_EXT_SENS_DATA_16	= 89,   /**< REG_9150_EXT_SENS_DATA_16 */
		REG_9150_EXT_SENS_DATA_17	= 90,   /**< REG_9150_EXT_SENS_DATA_17 */
		REG_9150_EXT_SENS_DATA_18	= 91,   /**< REG_9150_EXT_SENS_DATA_18 */
		REG_9150_EXT_SENS_DATA_19	= 92,   /**< REG_9150_EXT_SENS_DATA_19 */
		REG_9150_EXT_SENS_DATA_20	= 93,   /**< REG_9150_EXT_SENS_DATA_20 */
		REG_9150_EXT_SENS_DATA_21	= 94,   /**< REG_9150_EXT_SENS_DATA_21 */
		REG_9150_EXT_SENS_DATA_22	= 95,   /**< REG_9150_EXT_SENS_DATA_22 */
		REG_9150_EXT_SENS_DATA_23	= 96,   /**< REG_9150_EXT_SENS_DATA_23 */
		REG_9150_I2C_SLV0_DO		= 99,       /**< REG_9150_I2C_SLV0_DO */
		REG_9150_I2C_SLV1_DO		= 100,      /**< REG_9150_I2C_SLV1_DO */
		REG_9150_I2C_SLV2_DO		= 101,      /**< REG_9150_I2C_SLV2_DO */
		REG_9150_I2C_SLV3_DO		= 102,      /**< REG_9150_I2C_SLV3_DO */
		REG_9150_I2C_MST_DELAY_CTRL	= 103,/**< REG_9150_I2C_MST_DELAY_CTRL */
		REG_9150_SIGNAL_PATH_RESET	= 104, /**< REG_9150_SIGNAL_PATH_RESET */
		REG_9150_USER_CTRL			= 106,       /**< REG_9150_USER_CTRL */
		REG_9150_PWR_MGMT_1			= 107,      /**< REG_9150_PWR_MGMT_1 */
		REG_9150_PWR_MGMT_2			= 108,      /**< REG_9150_PWR_MGMT_2 */
		REG_9150_FIFO_COUNTH		= 114,      /**< REG_9150_FIFO_COUNTH */
		REG_9150_FIFO_COUNTL		= 115,      /**< REG_9150_FIFO_COUNTL */
		REG_9150_FIFO_R_W			= 116,        /**< REG_9150_FIFO_R_W */
		REG_9150_WHO_AM_I			= 117         /**< REG_9150_WHO_AM_I */
	};

	struct AK8975_mag_trim_registers {
		uint8_t asa_x;
		uint8_t asa_y;
		uint8_t asa_z;
	};

	/** \brief AK8975 register map
	 *
	 * Register names and addresses of the AK8975 magnetometer which is embedded in the MPU-9150
	 *
	 * Names and values were taken verbatim from the \n
	 * MPU-9150 Register Map and Descriptions, Revision 4.2
	 */

	enum AK8975_REGMAP {
		REG_AK8975_WIA		= 0X00,
		REG_AK8975_INFO		= 0X01,
		REG_AK8975_ST1		= 0X02,
		REG_AK8975_HXL		= 0X03,
		REG_AK8975_HXH		= 0X04,
		REG_AK8975_HYL		= 0X05,
		REG_AK8975_HYH		= 0X06,
		REG_AK8975_HZL		= 0X07,
		REG_AK8975_HZH		= 0X08,
		REG_AK8975_ST2		= 0X09,
		REG_AK8975_CNTL		= 0X0A,
		REG_AK8975_RSV		= 0X0B,
		REG_AK8975_ASTC		= 0X0C,
		REG_AK8975_TS1		= 0X0D,
		REG_AK8975_TS2		= 0X0E,
		REG_AK8975_I2CDIS	= 0X0F,
		REG_AK8975_ASAX		= 0X10,
		REG_AK8975_ASAY		= 0X11,
		REG_AK8975_ASAZ		= 0X12,

	};

} // namespace openEV::drivers::TDK_MPU9150

#endif /* DRIVERS_MPU_9150_MPU_9150DEFS_H_ */
