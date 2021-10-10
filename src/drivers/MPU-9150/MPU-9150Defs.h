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

	// =======================================================================
	// == MPU 9150/9050 IMU section ==========================================
	// =======================================================================

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

	// Constants for SLV4_ADDR

	/// Read or write to the device. High when reading
	static constexpr uint8_t I2C_SLV4_RW = 1 << 7;

	// Constants for SLV4_CTRL
	/// \brief Enable the Slave 4 start the transaction.
	///
	/// Be sure to have filled REG_9150_I2C_SLV4_ADDR, REG_9150_I2C_SLV4_REG,
	/// and REG_9150_I2C_SLV4_DO (if writing) before writing this bit into REG_9150_I2C_SLV4_CTRL
	static constexpr uint8_t I2C_SLV4_EN = 1 << 7;
	/// \brief Enable interrupt generation when the transaction is finished
	static constexpr uint8_t I2C_SLV4_INT_EN = 1 << 6;
	/// \brief Disable sending the content of REG_9150_I2C_SLV4_REG as register address before reading and writing
	static constexpr uint8_t I2C_SLV4_REG_DIS = 1 << 5;

	// Constants for PWR_MGMT_1
	/// \brief Device reset flag
	static constexpr uint8_t PWR_MGMT_DEVICE_RESET = 1 << 7;
	/// \brief Device sleep mode
	static constexpr uint8_t PWR_MGMT_SLEEP = 1 << 6;
	/// \brief Device cycle mode.
	///
	/// Cycles between sleep and active mode automatically.
	static constexpr uint8_t PWR_MGMT_CYCLE = 1 << 5;
	/// \brief Disable temperature measurement
	static constexpr uint8_t PWR_MGMT_DISABLE_TEMP = 1 << 3;
	/// \brief Clock source selection
	enum ClkSel {
		CLKSEL_INTERNAL = 0,
		CLKSEL_PLL_X_GYR = 1,
		CLKSEL_PLL_Y_GYR = 2,
		CLKSEL_PLL_Z_GYR = 3,
		CLKSEL_PLL_EXT_32_768KHZ_REF = 6,
		CLKSEL_PLL_EXT_19_2MHZ_REF = 5,
		// CLKSEL_RESERVED = 6,
		CLKSEL_STOP_CLK = 5,
	};

	// Constants for GYRO_CONFIG
	/// \brief Enable X gyro self test
	static constexpr uint8_t GYRO_X_SELF_TEST = 1 << 7;
	/// \brief Enable Y gyro self test
	static constexpr uint8_t GYRO_Y_SELF_TEST = 1 << 6;
	/// \brief Enable Z gyro self test
	static constexpr uint8_t GYRO_Z_SELF_TEST = 1 << 5;
	/// \brief Gyro full range from -x Deg/sec to +x Deg/sec
	///
	/// The values from the list in the register map are already
	/// shifted to the correct position in the register.
	enum GyroRange {
		GYRO_RANGE_250  = 0,
		GYRO_RANGE_500  = 1 << 3,
		GYRO_RANGE_1000 = 2 << 3,
		GYRO_RANGE_2000 = 3 << 3
	};

	// Constants for ACCEL_CONFIG
	/// \brief Enable X accel self test
	static constexpr uint8_t ACCEL_X_SELF_TEST = 1 << 7;
	/// \brief Enable Y accel self test
	static constexpr uint8_t ACCEL_Y_SELF_TEST = 1 << 6;
	/// \brief Enable Z accel self test
	static constexpr uint8_t ACCEL_Z_SELF_TEST = 1 << 5;
	/// \brief Accel full range from -x g to +x g
	///
	/// The values from the list in the register map are already
	/// shifted to the correct position in the register.
	enum AccelRange {
		ACCEL_RANGE_2G  = 0,
		ACCEL_RANGE_4G  = 1 << 3,
		ACCEL_RANGE_8G  = 2 << 3,
		ACCEL_RANGE_16G = 3 << 3
	};

	// Constants for CONFIG
	// I am skipping constants for the FSYNC pin routing. I am not using it anywhere
	/** \brief Configuration of digital low-pass filter (DLPF).
	 * Values describe the bandwith of the low-pass filter \n
	 * Please note that setting the filter to 256Hz bandwidth the base sample rate is 8kHz,
	 * else it is 1 kHz. The actual sample rate is is determined by the divider value in REG_9150_SMPLRT_DIV.
	 *
	 */
	enum DLPFConfig {
		DLPF_256HZ = 0,/**< DLPF_256HZ */
		DLPF_188HZ = 1,/**< DLPF_188HZ */
		DLPF_98HZ  = 2,/**< DLPF_98HZ */
		DLPF_42HZ  = 3,/**< DLPF_42HZ */
		DLPF_20HZ  = 4,/**< DLPF_20HZ */
		DLPF_10HZ  = 5,/**< DLPF_10HZ */
		DLPF_5HZ   = 6,/**< DLPF_5HZ */
		// DLPF_RESERVED   = 6,
	};

	// SMPRT_DIV is only an 8-bit value defining the sample rate divider.
	// The base sample rate is defined by the DLP Filter configuration (see enum DLPFConfig above)

	// Constants for INT_PIN_CFG
	/** \brief Logic level of the interrupt pin
	 *
	 * 0: Active High.
	 * 1: Active Low.
	 */
	static constexpr uint8_t INT_LEVEL = 1 << 7;
	/** \brief Interrupt pin driver
	 *
	 * 0: Push-Pull.
	 * 1: Open-Drain.
	 */
	static constexpr uint8_t INT_OPEN = 1 << 6;
	/** \brief Duration of signal on the interrupt pin
	 *
	 * 0: 50us pulse.
	 * 1: Active until interrupt flag is cleared.
	 */
	static constexpr uint8_t LATCH_INT_EN = 1 << 5;
	/** \brief Clearing of interrupt status
	 *
	 * 0: Interrupt status is cleared when REG_9150_INT_STATUS is being read.
	 * 1: Interrupt status is cleared upon any read operation.
	 */
	static constexpr uint8_t INT_RD_CLEAR = 1 << 4;
	/** \brief Logic level of the FSYNC pin
	 *
	 * 0: Active High.
	 * 1: Active Low.
	 */
	static constexpr uint8_t FSYNC_INT_LEVEL = 1 << 3;
	/** \brief Enable the FSYNC pin as external interrupt source
	 */
	static constexpr uint8_t FSYNC_INT_EN = 1 << 2;
	/** \brief Enable direct access to the Aux I2C bus via the main I2C bus
	 *
	 * Set this 1, and I2C_MST_EN in register REG_9150_USER_CTRL to 0
	 * to see and operate the embedded AK8975 directly from the main bus
	 */
	static constexpr uint8_t I2C_BYPASS_EN = 1 << 1;

	// Constants for USER_CTRL
	/// \brief Enable the FIFO
	static constexpr uint8_t FIFO_EN = 1 << 6;
	/** \brief Enable the I2C master
	 *
	 * The I2C master is the internal built-in I2C controller which
	 * controls the auxiliary I2C bus.
	 * When you enable this you also need to clear
	 * I2C_BYPASS_EN in REG_9150_INT_PIN_CFG to avoid interference with the main
	 * I2C bus. This way auxiliry I2C bus is autonomous.
	 */
	static constexpr uint8_t I2C_MST_EN = 1 << 5;
	/** \brief Disable the interface
	 *
	 * The register map document says: "Always write this bit as zero." \n
	 * My guess is that is silences the sensor to the outside world until you power-cycle it.
	 * Why this is implemented in the first place is beyond me.
	 */
	static constexpr uint8_t I2C_IF_DIS = 1 << 4;
	/** \brief Reset the FIFO
	 *
	 * FIFO_EN must be cleared for the reset to take effect.
	 */
	static constexpr uint8_t FIFO_RESET = 1 << 2;
	/** \brief Reset the I2C Master
	 *
	 * I2C_MST_EN must be cleared for the reset to take effect.
	 */
	static constexpr uint8_t I2C_MST_RESET = 1 << 1;
	/** \brief Reset accel, gyro, and thermometer.
	 *
	 * Reset the signal paths of accel, and gyro, and thermometer,
	 * as well as the sensor registers
	 */
	static constexpr uint8_t SIG_COND_RESET = 1;

	// constants for I2C_MST_CTRL
	/// \brief Enable multi-master capability for the internal I2C master.
	///
	/// Not required as long you do not share the *auxiliary* I2C bus with another I2C master.
	static constexpr uint8_t MULT_MST_EN = 1 << 7;
	/** \brief Delay Data Ready interrupt until external data are loaded
	 *
	 * Do not raise the Data Ready interrupt DATA_RDY_INT in register REG_9150_INT_STATUS
	 * until the registers in REG_9150_EXT_SENS_DATA_00 ff. have been loaded by the
	 * Slave 0-3 transactions.
	 */
	static constexpr uint8_t WAIT_FOR_ES = 1 << 6;
	/// \brief Load Slave 3 data into the FIFO when it is enabled.
	static constexpr uint8_t SLV_3_FIFO_EN = 1 << 5;
	/** \brief Determine if Stop-Start or re-start is sent between read transactions.
	 *
	 * If cleared a restart is sent between read transactions.
	 * If set a Stop-Start sequence is sent between read transactions.
	 *
	 * If a write follows a read Stop-Start is always being sent.
	 * The documentation is unfortunately silent about a read following a write. Sigh.
	 */
	static constexpr uint8_t I2C_MST_P_NSR = 1 << 4;
	// Bit 0-3 determine the I2C Master clock frequency
	// Just leave them at 0. That is 348 kHz which fits the internal AK8975 just fine.

	// Constants for INT_ENABLE
	/// \brief Enable the FIFO overflow interrupt.
	static constexpr uint8_t FIFO_OFLOW_EN = 1 << 4;
	/// \brief Enable the I2C Master interrupt.
	static constexpr uint8_t I2C_MST_INT_EN = 1 << 3;
	/// \brief Enable the Data Ready interrupt.
	static constexpr uint8_t DATA_RDY_EN = 1;


	// Constants for INT_STATUS
	/// \brief FIFO overflow interrupt
	static constexpr uint8_t FIFO_OFLOW_INT = 1 << 4;
	/** \brief I2C master interrupt.
	 *
	 * Interrupt sources are defined in REG_9150_I2C_MST_STATUS
	 */
	static constexpr uint8_t I2C_MST_INT = 1 << 3;
	/** \brief Data Ready interrupt
	 *
	 * The flag is cleared when this register is being read.
	 */
	static constexpr uint8_t DATA_RDY_INT = 1;

	// Constants for Slave 0-3. 'x' standand for the slave number in the following
	/// \brief When set the slave performs a read transaction
	static constexpr uint8_t I2C_SLVx_RW = 1 << 7;
	/// \brief Enable the slave. Only effective when the length is > 0
	static constexpr uint8_t I2C_SLVx_EN = 1 << 7;
	/// \brief Enable byte swapping
	static constexpr uint8_t I2C_SLVx_BYTE_SW = 1 << 6;
	/** \brief Disable sending the register address at the start of the transaction
	 *
	 * When set the transaction is a pure write or read operation.
	 * Otherwise the register address must be written to the I2C_SLVx_REG register.
	 */
	static constexpr uint8_t I2C_SLVx_REG_DIS = 1 << 5;
	/// Determines if odd or even pairs of bytes are swapped when I2C_SLVx_BYTE_SW is set
	static constexpr uint8_t I2C_SLVx_GRP = 1 << 4;

	// =======================================================================
	// == AK8975 Magnetometer section ========================================
	// =======================================================================
	/** \brief I2C address of the embedded AK8975.
	 *
	 * The address is fixed at 0x0c because the LSB pin is not exposed outside.
	 */
	static constexpr uint8_t AK8975_I2CAddr = 0x0c;

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

	// Constants for CNTL
	/** \brief Set the device power down mode
	 *
	 * Base status where all other status transitions start.
	 */
	static constexpr uint8_t AK8975_PWR_DOWN = 0;
	/** \brief Start a single measurement
	 *
	 * Executes a single conversion, and stores the values in the measurement registers
	 *
	 * After the conversion the fag resets itself and the sensor
	 * automatically returns to the Power Down mode
	 *
	 * After the conversion REG_AK8975_ST1 REG_AK8975_ST2 contain the result statuses.
	 */
	static constexpr uint8_t AK8975_SINGLE_MEAS = 0b0001;
	/** \brief Activate the self test mode.
	 *
	 * Activates the internal magnet field source for self test.
	 *
	 * Status must be reset to Power Down mode before issuing other commands.
	 */
	static constexpr uint8_t AK8975_SELF_TEST = 0b1000;
	/** \brief Activate PROM read mode.
	 *
	 * Allows reading registers REG_AK8975_ASAX, REG_AK8975_ASAY, and REG_AK8975_ASAZ
	 * bearing the factory set sensitivity adjustment values.
	 */
	static constexpr uint8_t AK8975_PROM_READ = 0b1111;

	// Constants for ST1
	/// When set data is ready in the measurement registers
	static constexpr uint8_t AK8975_DRDY = 1;

	// Constants for ST2
	/// Magnetic sensor overflow; measurement is invalid
	static constexpr uint8_t AK8975_HOFL = 1 << 3;
	/// Invalid measurement data due to reading measurement registers
	/// while a conversion cycle is still ongoing.
	static constexpr uint8_t AK8975_DERR = 1 << 3;


} // namespace openEV::drivers::TDK_MPU9150

#endif /* DRIVERS_MPU_9150_MPU_9150DEFS_H_ */
