/**
 * \file kma36.c
 *
 * \brief KMA36 Universal magnetic encoder sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to KMA36 datasheet :
 *	http://www.meas-spec.com/product/t_product.aspx?id=8911&terms=kma36
 *
 */

#include "kma36.h"

 /**
  * The header "i2c.h" has to be implemented for your own platform to 
  * conform the following protocol :
  *
  * enum i2c_transfer_direction {
  * 	I2C_TRANSFER_WRITE = 0,
  * 	I2C_TRANSFER_READ  = 1,
  * };
  * 
  * enum status_code {
  * 	STATUS_OK           = 0x00,
  * 	STATUS_ERR_OVERFLOW	= 0x01,
  *		STATUS_ERR_TIMEOUT  = 0x02,
  * };
  * 
  * struct i2c_master_packet {
  * 	// Address to slave device
  * 	uint16_t address;
  * 	// Length of data array
  * 	uint16_t data_length;
  * 	// Data array containing all data to be transferred
  * 	uint8_t *data;
  * };
  * 
  * void i2c_master_init(void);
  * enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);
  */
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// KMA36 device address
#define KMA36_ADDRESS_GND							0x59 //0b1011001
#define KMA36_ADDRESS_DCOILP						0x5A //0b1011010
#define KMA36_ADDRESS_DCOILN						0x5B //0b1011011
#define KMA36_ADDRESS_DVCC_SE						0x5C //0b1011100
#define KMA36_ADDRESS_VCC							0x5D //0b1011101

#define KMA36_CONFIGURATION_DELAY                   30   // ms delay for configuration to take effect

// Index and Size of the byte array for writing data
#define SEND_DATA_KCONF_INDEX						0
#define SEND_DATA_KRESH_INDEX						1
#define SEND_DATA_KRESL_INDEX						2
#define SEND_DATA_CRC_INDEX							3
#define SEND_DATA_SIZE								4

// Index and Size of the byte array for writing data
#define READ_DATA_MA0_INDEX							0
#define READ_DATA_MA1_INDEX							1
#define READ_DATA_ILC0_INDEX						2	
#define READ_DATA_ILC1_INDEX						3	
#define READ_DATA_ILC2_INDEX						4	
#define READ_DATA_ILC3_INDEX						5	
#define READ_DATA_KCONF_INDEX						6
#define READ_DATA_SIZE								7

#define KCONF_OVCS_MASK								0x03
#define KCONF_OVCS_SHIFT							0x00
#define KCONF_SPEED_MODE_ENABLE						0x04
#define KCONF_LOW_POWER_MODE_ENABLE					0x08
#define KCONF_COUNTER_MODE_ENABLE					0x10
#define KCONF_SLEEP_MODE_ENABLE						0x80

// Static functions
static enum kma36_status kma36_write_data(uint8_t *);
//static enum kma36_status kma36_read_data(uint8_t *);
static uint8_t kma36_crc_compute (uint8_t *data);
static enum kma36_status kconf_enable_bits(uint8_t mask);
static enum kma36_status kconf_disable_bits(uint8_t mask);

uint8_t kma36_address = KMA36_ADDRESS_GND;
uint8_t kma36_configuration[SEND_DATA_SIZE] = { 0x02, 0x40, 0x00, 0x7F};
uint16_t kma36_resolution = 1023;
bool    kma36_counter_enabled = false;
enum status_code i2c_status;

/**
 * \brief Configures the SERCOM I2C master to be used with the KMA36 device.
 */
void kma36_init(void)
{
	/* Initialize and enable device with config. */
	i2c_master_init();
}

/**
 * \brief Configures KMA36 I2C address to be used depending on HW configuration
 *
 * \param[in] address : KMA36 I2C address
 *
 */
void kma36_set_i2c_address( enum kma36_i2c_address address)
{
	if(address == kma36_i2c_address_GND)
		kma36_address = KMA36_ADDRESS_GND;
	else if(address == kma36_i2c_address_DCOILP)
		kma36_address = KMA36_ADDRESS_DCOILP;
	else if(address == kma36_i2c_address_DCOILN)
		kma36_address = KMA36_ADDRESS_DCOILN;
	else if(address == kma36_i2c_address_DVCC_SE)
		kma36_address = KMA36_ADDRESS_DVCC_SE;
	else if(address == kma36_i2c_address_VCC)
		kma36_address = KMA36_ADDRESS_VCC;
		
}

/**
 * \brief Check whether kma36 device is connected
 *
 * \return bool : status of KMA36
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool kma36_is_connected(void)
{
	//enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = kma36_address,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}
	
/**
 * \brief Writes the KMA36 data passed
 *
 * \param[in] uint8_t *: Data pointer on data to write. The array passed shall have fixed size of SEND_DATA_SIZE = 4
				This function is computing and storing the CRC at index 3 of the array
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum kma36_status kma36_write_data(uint8_t *data)
{
	//enum status_code i2c_status;

	// Compute CRC
	data[SEND_DATA_CRC_INDEX] = kma36_crc_compute(data);
	
	struct i2c_master_packet transfer = {
		.address     = kma36_address,
		.data_length = SEND_DATA_SIZE,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return kma36_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return kma36_status_i2c_transfer_error;
	
	delay_ms(KMA36_CONFIGURATION_DELAY);
	
	return kma36_status_ok;
}

/**
 * \brief Reads the KMA36 user register.
 *
 * \param[out] uint8_t* : Storage of data being read. The array passed shall have fixed size of READ_DATA_SIZE = 7
 *
 * \return kma36_status : status of kma36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum kma36_status kma36_read_data(uint8_t *data)
{
	//enum status_code i2c_status;

	/* Read data */
	struct i2c_master_packet read_transfer = {
		.address     = kma36_address,
		.data_length = READ_DATA_SIZE,
		.data        = data,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0
	};
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return kma36_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return kma36_status_i2c_transfer_error;

	delay_ms(30);
	
	return kma36_status_ok;
}

/**
 * \brief Compute CRC based on data. Minimum 3 bytes have to be passed in the byte array
 *
 * \param[in] uint8_t* : Pointer on data to compute CRC
 *
 * \return uint8_t : computed CRC
 */
uint8_t kma36_crc_compute (uint8_t *data)
{
	return (0xFF - (data[SEND_DATA_KCONF_INDEX] + data[SEND_DATA_KRESH_INDEX] + data[SEND_DATA_KRESL_INDEX]) + 0x01);
}

/**
 * \brief Request KMA36 to enter sleep mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer 
 */
enum kma36_status kma36_sleep_enter(void)
{
	return kconf_enable_bits(KCONF_SLEEP_MODE_ENABLE);
}

/**
 * \brief Request KMA36 to exit sleep mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_sleep_exit(void)
{
	return kconf_disable_bits(KCONF_SLEEP_MODE_ENABLE);
}

/**
 * \brief Request KMA36 to enable low power mode. In this mode, only 180° measurements are possible
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_enable_low_power_mode(void)
{
	return kconf_enable_bits(KCONF_LOW_POWER_MODE_ENABLE);
}

/**
 * \brief Request KMA36 to disable low power mode
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_low_power_mode(void)
{
	return kconf_disable_bits(KCONF_LOW_POWER_MODE_ENABLE);
}


/**
 * \brief Request KMA36 to enable full turn counting
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_enable_counter(void)
{
	enum kma36_status status;
	
	status =  kconf_enable_bits(KCONF_COUNTER_MODE_ENABLE);
	if( status != kma36_status_ok)
		return status;
	
	kma36_counter_enabled = true;

	return kma36_status_ok;
}

/**
 * \brief Request KMA36 to disable full turn counting
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_counter(void)
{
	enum kma36_status status;
	
	status =  kconf_disable_bits(KCONF_COUNTER_MODE_ENABLE);
	if( status != kma36_status_ok)
		return status;
	
	kma36_counter_enabled = false;

	return kma36_status_ok;
}

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
enum kma36_status kma36_enable_fast_rate(void)
{
	return kconf_enable_bits(KCONF_SPEED_MODE_ENABLE);
}

/**
 * \brief Request KMA36 to disable fast measurement update rate 
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kma36_disable_fast_rate(void)
{
	return kconf_disable_bits(KCONF_SPEED_MODE_ENABLE);
}

/**
 * \brief Set KMA36 accuracy - unit is to be discussed
 *        Accuracy impacts the measurement update rate.
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
enum kma36_status kma36_set_accuracy(enum kma36_oversampling ovs)
{
	enum kma36_status status;
	uint8_t read_data[READ_DATA_SIZE];
	uint8_t save = kma36_configuration[SEND_DATA_KCONF_INDEX];
	
	status = kma36_read_data(read_data);
	if( status != kma36_status_ok)
		return status;

	read_data[READ_DATA_KCONF_INDEX] &=  ~(KCONF_OVCS_MASK << KCONF_OVCS_SHIFT);
	kma36_configuration[SEND_DATA_KCONF_INDEX] = read_data[READ_DATA_KCONF_INDEX] | ((ovs & KCONF_OVCS_MASK) << KCONF_OVCS_SHIFT);

	status = kma36_write_data(kma36_configuration);
	if( status != kma36_status_ok)
	// If Error - Restore previous register value
		kma36_configuration[SEND_DATA_KCONF_INDEX] = save;
	
	return status;
}

/**
 * \brief Set KMA36 resolution
 *
 * \param[in] uint16_t : unit in one-hundredth of a degree
						 Note that maximal resolution is exactly 360/32768 = 0.01099
						 It is not expected also to have a resolution above 360° which would not make much sense.
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on PROM coefficients
 */
enum kma36_status kma36_set_resolution(uint16_t res)
{
	enum kma36_status status;
	uint16_t kres;
	uint8_t save[2];
	
	save[0] = kma36_configuration[SEND_DATA_KRESH_INDEX];
	save[1] = kma36_configuration[SEND_DATA_KRESL_INDEX];
	
	kres = 360 * 100 / res;
	
	if(kres > 32767)
		kres = 32767;
	
	kma36_resolution = kres;
	
	kma36_configuration[SEND_DATA_KRESH_INDEX] = kres>>8;
	kma36_configuration[SEND_DATA_KRESL_INDEX] = kres&0xFF;

	status = kma36_write_data(kma36_configuration);
	if( status != kma36_status_ok) {
		kma36_configuration[SEND_DATA_KRESH_INDEX] = save[0];
		kma36_configuration[SEND_DATA_KRESL_INDEX] = save[1];
	}
	
	return status;
}

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
enum kma36_status kma36_read_angle(float *angle)
{
	enum kma36_status status;
	uint8_t read_data[READ_DATA_SIZE];
	int32_t ma;
	
	status = kma36_read_data(read_data);
	if( status != kma36_status_ok)
		return status;
	
	if( !kma36_counter_enabled ) {
		
		// Compute angle according to current resolution
		ma = (read_data[READ_DATA_MA1_INDEX] <<8) | read_data[READ_DATA_MA0_INDEX];
	
	}else {
			// Negative number
			ma =   (((read_data[READ_DATA_ILC3_INDEX] & 0x7F) <<24) | (read_data[READ_DATA_ILC2_INDEX]<<16)
			     | (read_data[READ_DATA_ILC1_INDEX]<<8) | read_data[READ_DATA_ILC0_INDEX]) ;
	}
				
	// Compute angle according to current resolution
	*angle = (float)ma / ((float)kma36_resolution/360.0f);
			
	return kma36_status_ok;
}

/**
 * \brief Generic function to enable bits in Kconf register based on mask provide as input
 *
 * \param[in] mask : bits to enable in the Kconf register
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kconf_enable_bits(uint8_t mask)
{
	enum kma36_status status;
	uint8_t read_data[READ_DATA_SIZE];
	uint8_t save = kma36_configuration[SEND_DATA_KCONF_INDEX];
	
	status = kma36_read_data(read_data);
	if( status != kma36_status_ok)
		return status;

	kma36_configuration[SEND_DATA_KCONF_INDEX] = read_data[READ_DATA_KCONF_INDEX] | mask;

	status = kma36_write_data(kma36_configuration);
	if( status != kma36_status_ok)
	// If Error - Restore previous register value
		kma36_configuration[SEND_DATA_KCONF_INDEX] = save;
	
	return status;
}

/**
 * \brief Generic function to disable bits in Kconf register based on mask provide as input
 *
 * \param[in] mask : bits to disable in the Kconf register
 *
 * \return kma36_status : status of KMA36
 *       - kma36_status_ok : I2C transfer completed successfully
 *       - kma36_status_i2c_transfer_error : Problem with i2c transfer
 *       - kma36_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - kma36_status_crc_error : CRC error on write transfer
 */
enum kma36_status kconf_disable_bits(uint8_t mask)
{
	enum kma36_status status;
	uint8_t read_data[READ_DATA_SIZE];
	uint8_t save = kma36_configuration[SEND_DATA_KCONF_INDEX];
	
	status = kma36_read_data(read_data);
	if( status != kma36_status_ok)
		return status;
	
	kma36_configuration[SEND_DATA_KCONF_INDEX] = read_data[READ_DATA_KCONF_INDEX] & (~mask);

	status = kma36_write_data(kma36_configuration);
	if( status != kma36_status_ok)
	// If Error - Restore previous register value
		kma36_configuration[SEND_DATA_KCONF_INDEX] = save;
	
	return status;
}

#ifdef __cplusplus
}
#endif