/*
 * Copyright 2020 Li Wei.
 * Author: Li Wei <oldrevATgmail.com>
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "atca_config.h"

#include "atca_hal.h"

#ifdef ATCA_HAL_STM32F0
#include <stm32f0xx_hal.h>
#endif
#ifdef ATCA_HAL_STM32F1
#include <stm32f1xx_hal.h>
#endif
#ifdef ATCA_HAL_STM32F2
#include <stm32f2xx_hal.h>
#endif

#include "cryptoauthlib.h"


#define MAX_I2C_BUSES 2  //ESP32 has 2 I2C bus


typedef struct ATCAI2CMaster_tag
{
	I2C_HandleTypeDef stm32_i2c;
    int ref_ct;
    uint8_t bus_index;
} ATCAI2CMaster;

static ATCAI2CMaster s_i2c_hal_data[MAX_I2C_BUSES];   // map logical, 0-based bus number to index
static int s_i2c_bus_ref_ct = 0;
/** \brief method to change the bus speec of I2C
 * \param[in] iface  interface on which to change bus speed
 * \param[in] speed  baud rate (typically 100000 or 400000)
 */
void hal_i2c_change_baud(ATCAIface iface, uint32_t speed)
{
	/*
    esp_err_t rc;
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int bus = cfg->atcai2c.bus;

    conf.master.clk_speed = speed;
    rc = i2c_param_config(i2c_hal_data[bus]->id, &conf);
    if (rc == ESP_OK)
    {
//        ESP_LOGD(TAG, "Baudrate Changed");
    }
    else
    {
//        ESP_LOGW(TAG, "Baudrate Change Failed");
    }
    */
}

/** \brief
    - this HAL implementation assumes you've included the START Twi libraries in your project, otherwise,
    the HAL layer will not compile because the START TWI drivers are a dependency *
 */

/** \brief hal_i2c_init manages requests to initialize a physical interface.  it manages use counts so when an interface
 * has released the physical layer, it will disable the interface for some other use.
 * You can have multiple ATCAIFace instances using the same bus, and you can have multiple ATCAIFace instances on
 * multiple i2c buses, so hal_i2c_init manages these things and ATCAIFace is abstracted from the physical details.
 */

/** \brief initialize an I2C interface using given config
 * \param[in] hal - opaque ptr to HAL data
 * \param[in] cfg - interface configuration
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
    ATCAI2CMaster* data = &s_i2c_hal_data[cfg->atcai2c.bus];

    if (s_i2c_bus_ref_ct == 0)
    {
        for (int i = 0; i < MAX_I2C_BUSES; i++)
        {
            s_i2c_hal_data[i].ref_ct = 0;
        }
    }


    if (data->ref_ct <= 0)
    {
        // Bus isn't being used, enable it

    	/*
    	 hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

		if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
		{
			return ATCA_COMM_FAIL;
		}
		*/

        // store this for use during the release phase
        data->bus_index = cfg->atcai2c.bus;

        // buses are shared, this is the first instance
        data->ref_ct = 1;
    }
    else
    {
        // Bus is already is use, increment reference counter
        data->ref_ct++;
    }


    ((ATCAHAL_t*)hal)->hal_data = data;
	return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C send
 * \param[in] iface         instance
 * \param[in] word_address  device transaction type
 * \param[in] txdata        pointer to space to bytes to send
 * \param[in] txlength      number of bytes to send
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t word_address, uint8_t *txdata, int txlength)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);

    if (!cfg)
    {
        return ATCA_BAD_PARAM;
    }

    if (0xFF != word_address)
    {
        txdata[0] = word_address;   // insert the Word Address Value, Command token
        txlength++;                 // account for word address value byte.
    }

    I2C_HandleTypeDef* stm32_i2c = &s_i2c_hal_data[cfg->atcai2c.bus].stm32_i2c;
    uint16_t device_address = cfg->atcai2c.slave_address >> 1;

    if(HAL_I2C_Mem_Write(stm32_i2c, device_address, 0, 0, txdata, (uint16_t)txlength, 5000) != HAL_OK)
    {
        return ATCA_COMM_FAIL;
    }

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C receive function
 * \param[in]    iface         Device to interact with.
 * \param[in]    word_address  device transaction type
 * \param[out]   rxdata        Data received will be returned here.
 * \param[in,out] rxlength     As input, the size of the rxdata buffer.
 *                             As output, the number of bytes received.
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t word_address, uint8_t *rxdata, uint16_t *rxlength)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries;
    uint32_t status = !ATCA_SUCCESS;
    uint16_t rxdata_max_size = *rxlength;
    uint8_t min_response_size = 4;
    uint16_t read_length = 2;

    if ((NULL == cfg) || (NULL == rxlength) || (NULL == rxdata))
    {
        return ATCA_TRACE(ATCA_INVALID_POINTER, "NULL pointer encountered");
    }

    uint16_t device_address = cfg->atcai2c.slave_address >> 1;

    *rxlength = 0;
    //Read Length byte i.e. first byte from device
    if (rxdata_max_size < 1)
    {
        return ATCA_SMALL_BUFFER;
    }

    I2C_HandleTypeDef* stm32_i2c = &s_i2c_hal_data[cfg->atcai2c.bus].stm32_i2c;
    do
    {
        /*Send Word address to device...*/
        retries = cfg->rx_retries;
        while (retries-- > 0 && status != ATCA_SUCCESS)
        {
            status = hal_i2c_send(iface, word_address, &word_address, 0);

        }
        if (ATCA_SUCCESS != status)
        {
            ATCA_TRACE(status, "hal_i2c_send - failed");
            break;
        }

#if ATCA_TA_SUPPORT
        /*Set read length.. Check for register reads or 1 byte reads*/
        if ((word_address == ATCA_MAIN_PROCESSOR_RD_CSR) || (word_address == ATCA_FAST_CRYPTO_RD_FSR)
            || (rxdata_max_size == 1))
        {
            read_length = 1;
        }
#endif

        atca_delay_ms(2);
        status = ATCA_COMM_FAIL;
        if(HAL_I2C_Master_Receive(stm32_i2c, device_address, rxdata, read_length, 5000) == HAL_OK)
        {
            status = ATCA_SUCCESS;
        }

        if (ATCA_SUCCESS != status)
        {
            ATCA_TRACE(status, "hal read - failed");
            break;
        }

        if (1 == read_length)
        {
            ATCA_TRACE(status, "1 byte read completed");
            break;
        }

        /*Calculate bytes to read based on device response*/
        if (cfg->devtype == TA100)
        {
            read_length = ((uint16_t)rxdata[0] * 256) + rxdata[1];
            min_response_size += 1;
        }
        else
        {
            read_length =  rxdata[0];
        }

        if (read_length > rxdata_max_size)
        {
            status = ATCA_TRACE(ATCA_SMALL_BUFFER, "rxdata is small buffer");
            break;
        }

        if (read_length < min_response_size)
        {
            status = ATCA_TRACE(ATCA_RX_FAIL, "packet size is invalid");
            break;
        }

        /* Read given length bytes from device */
        atca_delay_ms(2);

        status = ATCA_COMM_FAIL;
        if(HAL_I2C_Master_Receive(stm32_i2c, device_address, &rxdata[2], read_length - 2, 5000) == HAL_OK)
        {
            status = ATCA_SUCCESS;
        }

        if (ATCA_SUCCESS != status)
        {
            ATCA_TRACE(status, "hal read - failed");
            break;
        }

    }
    while (0);

    *rxlength = read_length;
    return status;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_release(void *hal_data)
{
    ATCAI2CMaster *hal = (ATCAI2CMaster*)hal_data;

    s_i2c_bus_ref_ct--;
    // if the use count for this bus has gone to 0 references, disable it.
    // protect against an unbracketed release
    if (hal != NULL && --(hal->ref_ct) <= 0)
    {
    	// 禁用 I2C
    	if(HAL_I2C_DeInit(&hal->stm32_i2c) != HAL_OK) {
    		return ATCA_COMM_FAIL;
    	}
        hal->ref_ct = 0;
    }

    return ATCA_SUCCESS;
}

/** \brief wake up CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to wakeup
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries = cfg->rx_retries;
    uint32_t bdrt = cfg->atcai2c.baud;
    uint8_t data[4] = { 0x00, 0x00, 0x00, 0x00 };

    // if not already at 100kHz, failed
    if (bdrt != 100000)
    {
        return ATCA_COMM_FAIL;
    }

	// Send 0x00 as wake pulse
    I2C_HandleTypeDef* stm32_i2c = &s_i2c_hal_data[cfg->atcai2c.bus].stm32_i2c;
    HAL_I2C_Master_Transmit(stm32_i2c, (uint16_t)0x00, data, 1, 1000);

    // rounded up to the nearest ms
    //atca_delay_ms(((uint32_t)cfg->wake_delay + (1000 - 1)) / 1000);         // wait tWHI + tWLO which is configured based on device type and configuration structure
    atca_delay_ms(10);

    // if necessary, revert baud rate to what came in.
    /*
    if (bdrt != 100000)
    {
        change_i2c_speed(iface, bdrt);
    }
    */

    uint16_t device_address = cfg->atcai2c.slave_address >> 1;
    int status = !HAL_OK;
    while (retries-- > 0 && status != HAL_OK)
    {
    	status = HAL_I2C_Master_Receive(stm32_i2c, device_address, data, 4, 1000);
    }
    if (status != HAL_OK)
    {
        return ATCA_COMM_FAIL;
    }

    return hal_check_wake(data, 4);
}

/** \brief idle CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to idle
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    uint8_t data[4] = {0x02, 0, 0, 0};// idle word address value

    uint16_t device_address = cfg->atcai2c.slave_address >> 1;
    I2C_HandleTypeDef* stm32_i2c = &s_i2c_hal_data[cfg->atcai2c.bus].stm32_i2c;
    if(HAL_I2C_Mem_Write(stm32_i2c, device_address, 0, 0, data, 1, 5000) != HAL_OK)
    {
        return ATCA_COMM_FAIL;
    }

    return ATCA_SUCCESS;
}

/** \brief sleep CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to sleep
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    uint8_t data[4] = {0x01, 0, 0, 0};// sleep word address value

    uint16_t device_address = cfg->atcai2c.slave_address >> 1;
    I2C_HandleTypeDef* stm32_i2c = &s_i2c_hal_data[cfg->atcai2c.bus].stm32_i2c;
    if(HAL_I2C_Mem_Write(stm32_i2c, device_address, 0, 0, data, 1, 5000) != HAL_OK)
    {
        return ATCA_COMM_FAIL;
    }

    return ATCA_SUCCESS;
}

/** \brief discover i2c buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge
 * \param[in] i2c_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
{
//    ESP_LOGI(TAG, "hal_i2c_discover_buses");
    return ATCA_UNIMPLEMENTED;
}

/** \brief discover any CryptoAuth devices on a given logical bus number
 * \param[in]  bus_num  logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg     pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] found   number of devices found on this bus
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_discover_devices(int bus_num, ATCAIfaceCfg *cfg, int *found)
{
//    ESP_LOGI(TAG, "hal_i2c_discover_devices");
    return ATCA_UNIMPLEMENTED;
}

