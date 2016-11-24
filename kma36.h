/**
 * \file kma36.h
 *
 * \brief KMA36 Universal magnetic encoder sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 */

#ifndef KMA36_H_INCLUDED
#define KMA36_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

enum kma36_status {
	kma36_status_ok,
	kma36_status_no_i2c_acknowledge,
	kma36_status_i2c_transfer_error,
	kma36_status_crc_error,
};

enum kma36_i2c_address {
	kma36_i2c_address_GND,
	kma36_i2c_address_DCOILP,
	kma36_i2c_address_DCOILN,
	kma36_i2c_address_DVCC_SE,
	kma36_i2c_address_VCC
};

enum kma36_oversampling {
	kma36_oversampling_2 = 0,
	kma36_oversampling_4,
	kma36_oversampling_8,
	kma36_oversampling_32
};

enum kma36_status kma36_read_data(uint8_t *);

// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the kma36 device.
 */
void kma36_init(void);

/**
 * \brief Check whether KMA36 device is connected
 *
 * \return bool : status of KMA36
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool kma36_is_connected(void);

/**
 * \brief Configures KMA36 I2C address to be used depending on HW configuration
 *        The address depends on connection of one of following pins (GND, DCOILP, DCOILN, DVCC_SE, VCC) with A0 pin
 *
 * \param[in] kma36_i2c_address : KMA36 I2C address
 *
 */
void kma36_set_i2c_address( enum kma36_i2c_address );

/**
 * \brief Request KMA36 to enter sleep mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on 
 */
enum kma36_status kma36_sleep_enter(void);

/**
 * \brief Request KMA36 to exit sleep mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on PROM coefficients
 */
enum kma36_status kma36_sleep_exit(void);

/**
 * \brief Request KMA36 to enable low power mode. In this mode, only 180° measurements are possible
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_enable_low_power_mode(void);

/**
 * \brief Request KMA36 to disable low power mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_low_power_mode(void);

/**
 * \brief Request KMA36 to enable full turn counting
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_enable_counter(void);

/**
 * \brief Request KMA36 to disable full turn counting
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_counter(void);

/**
 * \brief Request KMA36 to enable fast measurement update rate
 *        In fast mode, measurement accuracy is reduced
 *        Update rate = 1 / ( 1.4ms x oversampling x const )
 *		  - const = 1 if fast rate disabled
 *		  - const = 2 if fast rate enabled
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_enable_fast_rate(void);

/**
 * \brief Request KMA36 to disable fast measurement update rate 
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_fast_rate(void);

/**
 * \brief Set KMA36 accuracy - unit is to be discussed
 *        Resolution impacts the measurement update rate.
 *        Update rate = 1 / ( 1.4ms x oversampling x const )
 *		  - const = 1 if fast rate disabled
 *		  - const = 2 if fast rate enabled
 *
 * \param[in] kma36_oversampling : oversampling rate to be used.
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_set_accuracy(enum kma36_oversampling);

/**
 * \brief Set KMA36 resolution
 *
 * \param[in] uint16_t : unit in one-hundredth of a degree
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on PROM coefficients
 */
enum kma36_status kma36_set_resolution(uint16_t);

/**
 * \brief Read KMA36 current angle
 *
 * \param[in] float * : Returns the number of degrees of rotation
                        If counter mode is enabled, the number of degrees might exceed +/- 360°
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on PROM coefficients
 */
enum kma36_status kma36_read_angle(float*);

#endif /* KMA36_H_INCLUDED */