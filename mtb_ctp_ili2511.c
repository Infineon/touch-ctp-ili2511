/*******************************************************************************
* \file mtb_ctp_ili2511.c
* \version 1.0.0
*
* \brief
* Provides implementation of the ILI2511 CTP driver library.
*
********************************************************************************
* \copyright
* Copyright 2025 Cypress Semiconductor Corporation (an Infineon company)
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "mtb_ctp_ili2511.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_LOW                           (0U)
#define GPIO_HIGH                          (1U)

#define BYTE_SHIFT                         (8U)
#define BIT_MASK                           (0x3FFF)
#define I2C_TIMEOUT_COUNT                  (5U)
#define RETRY_COUNT                        (5U)
#define I2C_DELAY_MS                       (1U)
#define RESET_VAL                          (0U)
#define SET_VAL                            (1U)

#define RST_PIN_LOW_HOLD_TIME_MS           (50U)
#define INITIALIZATION_TIME_MS             (100U)
#define READ_DELAY_MS                      (2U)

/* Combine I2C controller error statuses in single mask  */
#define I2C_CONTROLLER_ERROR_MASK          (CY_SCB_I2C_MASTER_DATA_NAK | \
                                            CY_SCB_I2C_MASTER_ADDR_NAK | \
                                            CY_SCB_I2C_MASTER_ARB_LOST | \
                                            CY_SCB_I2C_MASTER_ABORT_START | \
                                            CY_SCB_I2C_MASTER_BUS_ERR)


/*******************************************************************************
* Global Variables
*******************************************************************************/
static mtb_ctp_ili2511_config_t* ctp_config = NULL;


/*******************************************************************************
* Function name: mtb_ctp_ili2511_i2c_controller_write
********************************************************************************
*
* Performs I2C controller write to the ILI2511 CTP target.
*
* \param write_buffer
* Pointer to the I2C buffer for data to write to the target.
*
* \param buff_size
* Size of the I2C write buffer.
*
* \return cy_en_scb_i2c_status_t
* I2C controller write status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ili2511_i2c_controller_write(uint8_t* write_buffer,
                                                                   uint32_t buff_size)
{
    cy_stc_scb_i2c_master_xfer_config_t transfer_config;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t retry_count               = RETRY_COUNT;
    uint32_t i2c_controller_stat      = RESET_VAL;
    uint32_t timeout_count;

    CY_ASSERT(NULL != write_buffer);

    /* I2C controller transfer configuration */
    transfer_config.slaveAddress      = MTB_CTP_ILI2511_I2C_SLAVE_ADDR;
    transfer_config.buffer            = write_buffer;
    transfer_config.bufferSize        = buff_size;
    transfer_config.xferPending       = false;

    do
    {
        /* Initiate write transaction */
        /* The Start condition is generated to begin this transaction */
        i2c_status = Cy_SCB_I2C_MasterWrite(ctp_config->scb_inst, &transfer_config,
                                            ctp_config->i2c_context);

        if (CY_SCB_I2C_SUCCESS == i2c_status)
        {
            /* Total timeout 5 ms */
            timeout_count = I2C_TIMEOUT_COUNT;
            /* Wait until controller completes write transfer or time out has occurred */
            do
            {
                i2c_controller_stat = Cy_SCB_I2C_MasterGetStatus(ctp_config->scb_inst,
                                                                 ctp_config->i2c_context);
                Cy_SysLib_Delay(I2C_DELAY_MS);
                timeout_count--;
            } while ((CY_SCB_I2C_MASTER_BUSY & i2c_controller_stat) && timeout_count);

            if ((!(I2C_CONTROLLER_ERROR_MASK & i2c_controller_stat)) &&
                (transfer_config.bufferSize ==
                 Cy_SCB_I2C_MasterGetTransferCount(ctp_config->scb_inst, ctp_config->i2c_context)))
            {
                i2c_status  = CY_SCB_I2C_SUCCESS;
                retry_count = RESET_VAL;
            }
            else
            {
                /* Timeout/error recovery */
                Cy_SCB_I2C_Disable(ctp_config->scb_inst, ctp_config->i2c_context);
                Cy_SCB_I2C_Enable(ctp_config->scb_inst);
                retry_count--;
            }
        }
        else
        {
            retry_count--;
        }
    } while (retry_count);

    return i2c_status;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_i2c_controller_read
********************************************************************************
*
* Performs I2C controller read from the ILI2511 CTP target.
*
* \param read_buffer
* Pointer to the I2C buffer for data to be read from the target.
*
* \param buff_size
* Size of the I2C read buffer.
*
* \return cy_en_scb_i2c_status_t
* I2C controller read status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ili2511_i2c_controller_read(uint8_t* read_buffer,
                                                                  uint32_t buff_size)
{
    cy_stc_scb_i2c_master_xfer_config_t transfer_config;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t retry_count               = RETRY_COUNT;
    uint32_t i2c_controller_stat      = RESET_VAL;
    uint32_t timeout_count;

    CY_ASSERT(NULL != read_buffer);

    /* I2C controller transfer configuration */
    transfer_config.slaveAddress      = MTB_CTP_ILI2511_I2C_SLAVE_ADDR;
    transfer_config.buffer            = read_buffer;
    transfer_config.bufferSize        = buff_size;
    transfer_config.xferPending       = false;

    do
    {
        /* Initiate read transaction. */
        i2c_status = Cy_SCB_I2C_MasterRead(ctp_config->scb_inst, &transfer_config,
                                           ctp_config->i2c_context);

        if (CY_SCB_I2C_SUCCESS == i2c_status)
        {
            /* Total timeout 5 ms */
            timeout_count = I2C_TIMEOUT_COUNT;
            /* Wait until controller complete read transfer or time out has occurred */
            do
            {
                i2c_controller_stat = Cy_SCB_I2C_MasterGetStatus(ctp_config->scb_inst,
                                                                 ctp_config->i2c_context);
                Cy_SysLib_Delay(I2C_DELAY_MS);
                timeout_count--;
            } while ((CY_SCB_I2C_MASTER_BUSY & i2c_controller_stat) && timeout_count);

            if (!(I2C_CONTROLLER_ERROR_MASK & i2c_controller_stat))
            {
                i2c_status  = CY_SCB_I2C_SUCCESS;
                retry_count = RESET_VAL;
            }
            else
            {
                /* Timeout/error recovery */
                Cy_SCB_I2C_Disable(ctp_config->scb_inst, ctp_config->i2c_context);
                Cy_SCB_I2C_Enable(ctp_config->scb_inst);
                retry_count--;
            }
        }
        else
        {
            retry_count--;
        }
    } while (retry_count);

    return i2c_status;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_is_alive
********************************************************************************
*
* This function checks whether ILI2511 IC is alive or not by reading the chip
* ID and firmware details of it.
*
* \param void
*
* \return cy_en_scb_i2c_status_t
* ILI2511 alive status in terms of I2C operation result
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ili2511_is_alive(void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    uint8_t cmd = (uint8_t)MTB_CTP_ILI2511_GET_FIRMWARE_VER_CMD;
    uint8_t firmware_version[MTB_CTP_ILI2511_FIRMWARE_NUM_BYTES] = { (uint8_t)RESET_VAL };

    /* Send I2C packet with command to the target */
    status = mtb_ctp_ili2511_i2c_controller_write(&cmd, sizeof(cmd));

    if (CY_SCB_I2C_SUCCESS == status)
    {
        status = mtb_ctp_ili2511_i2c_controller_read(firmware_version,
                                                     MTB_CTP_ILI2511_FIRMWARE_NUM_BYTES);
        if (CY_SCB_I2C_SUCCESS == status)
        {
            /*Check whether the firmware version is valid or not */
            if ((MTB_CTP_ILI2511_CHIP_ID_CODE != firmware_version[MTB_CTP_ILI2511_CHIP_ID_POS])
                || (MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER !=
                    firmware_version[MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER_POS])
                || (MTB_CTP_ILI2511_MINOR_FIRMWARE_VER !=
                    firmware_version[MTB_CTP_ILI2511_MINOR_FIRMWARE_VER_POS])
                || (MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER !=
                    firmware_version[MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER_POS])
                || (MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4 !=
                    firmware_version[MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4_POS])
                || (MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5 !=
                    firmware_version[MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5_POS])
                || (MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6 !=
                    firmware_version[MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6_POS])
                || (MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7 !=
                    firmware_version[MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7_POS]))
            {
                status = CY_SCB_I2C_BAD_PARAM;
            }
        }
    }

    return status;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_read_raw_touch_data
********************************************************************************
*
* This function reads the raw x, y coordinate data from the touch sensor.
*
* \param touch_data
* Pointer to the buffer to store touch data.
*
* \param size
* Size of the buffer to store touch data.
*
* \return cy_en_scb_i2c_status_t
* Read raw touch data operation status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ili2511_read_raw_touch_data(uint8_t* touch_data,
                                                                  uint32_t size)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t get_touch_cmd = (uint8_t)MTB_CTP_ILI2511_TOUCH_INFO_CMD;

    CY_ASSERT(NULL != touch_data);

    i2c_status = mtb_ctp_ili2511_i2c_controller_write(&get_touch_cmd, sizeof(get_touch_cmd));
    if (CY_SCB_I2C_SUCCESS == i2c_status)
    {
        i2c_status = mtb_ctp_ili2511_i2c_controller_read(touch_data, size);
    }

    return i2c_status;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_irq_handler
********************************************************************************
*
* Touch panel GPIO interrupt handler to detect touch events on the panel.
* When a finger touches on the touch panel surface, the IRQ pin will be pulled
* low and we set the touch_event flag true.
*
* \param void
*
* \return void
*
*******************************************************************************/
__WEAK void mtb_ctp_ili2511_irq_handler(void)
{
    if ((NULL != ctp_config) &&
        Cy_GPIO_GetInterruptStatus(ctp_config->irq_port, ctp_config->irq_pin))
    {
        ctp_config->touch_event = true;
        Cy_GPIO_ClearInterrupt(ctp_config->irq_port, ctp_config->irq_pin);
    }
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_init
********************************************************************************
*
* Performs ILI2511 touch controller driver initialization using I2C interface.
*
* \param ctp_ili2511_config
* Pointer to the ILI2511 touch controller configuration structure.
*
* \return cy_rslt_t
* Initialization status. CY_RSLT_SUCCESS on success else CY_RSLT_ILI2511_DEV_ERR,
* I2C error(s).
*
* \funcusage
* \snippet snippet/main.c snippet_mtb_ctp_ili2511_init
*
*******************************************************************************/
cy_rslt_t mtb_ctp_ili2511_init(mtb_ctp_ili2511_config_t* ctp_ili2511_config)
{
    cy_rslt_t status    = CY_RSLT_SUCCESS;
    uint8_t retry_count = RETRY_COUNT;

    CY_ASSERT(NULL != ctp_ili2511_config);

    ctp_config = ctp_ili2511_config;
    ctp_config->touch_event = false;

    cy_stc_sysint_t ctp_irq_cfg =
    {
        .intrSrc       = ctp_config->irq_num,
        .intrPriority  = MTB_CTP_ILI2511_IRQ_PRIORITY
    };

    /* Initialize CTP RESET GPIO pin with initial value as HIGH */
    Cy_GPIO_Pin_FastInit(ctp_config->rst_port, ctp_config->rst_pin,
                         CY_GPIO_DM_STRONG_IN_OFF, GPIO_HIGH, HSIOM_SEL_GPIO);

    /* Perform reset sequence to enable ILI2511 touch controller */
    Cy_GPIO_Write(ctp_config->rst_port, ctp_config->rst_pin, GPIO_LOW);
    Cy_SysLib_Delay(RST_PIN_LOW_HOLD_TIME_MS);
    Cy_GPIO_Write(ctp_config->rst_port, ctp_config->rst_pin, GPIO_HIGH);
    Cy_SysLib_Delay(INITIALIZATION_TIME_MS);

    do
    {
        /* Check whether ILI2511 touch controller is alive or not */
        status = (cy_rslt_t)mtb_ctp_ili2511_is_alive();
        if (CY_RSLT_SUCCESS != status)
        {
            retry_count--;
        }
        else
        {
            retry_count = RESET_VAL;
        }
    } while (retry_count);

    if (CY_RSLT_SUCCESS == status)
    {
        /* Initialize CTP IRQ pin and config touch interrupt */
        Cy_GPIO_Pin_FastInit(ctp_config->irq_port, ctp_config->irq_pin, CY_GPIO_DM_PULLUP,
                             GPIO_HIGH, HSIOM_SEL_GPIO);
        Cy_GPIO_SetInterruptEdge(ctp_config->irq_port, ctp_config->irq_pin, CY_GPIO_INTR_FALLING);
        Cy_GPIO_SetInterruptMask(ctp_config->irq_port, ctp_config->irq_pin, SET_VAL);
        Cy_SysInt_Init(&ctp_irq_cfg, &mtb_ctp_ili2511_irq_handler);
        NVIC_EnableIRQ(ctp_irq_cfg.intrSrc);
    }
    else
    {
        status = CY_RSLT_ILI2511_DEV_ERR;
    }

    return status;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_get_single_touch
********************************************************************************
*
* This function fetches raw touch coordinates using
* "mtb_ctp_ili2511_read_raw_touch_data" function, applies calibration and
* returns actual x, y touch coordinates.
*
* \param touch_x
* Pointer to the X coordinate of touched location.
*
* \param touch_y
* Pointer to the Y coordinate of touched location.
*
* \return cy_rslt_t
* Result of reading single touch data operation, CY_RSLT_SUCCESS on success else
* CY_RSLT_ILI2511_NO_TOUCH, I2C error(s).
*
*******************************************************************************/
cy_rslt_t mtb_ctp_ili2511_get_single_touch(int* touch_x, int* touch_y)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    static uint16_t last_touch_x = RESET_VAL;
    static uint16_t last_touch_y = RESET_VAL;
    uint16_t current_touch_x = RESET_VAL;
    uint16_t current_touch_y = RESET_VAL;
    uint8_t touch_data[MTB_CTP_ILI2511_TOUCH_NUM_BYTES] = { (uint8_t)RESET_VAL };

    CY_ASSERT(NULL != touch_x);
    CY_ASSERT(NULL != touch_y);

    result = (cy_rslt_t)mtb_ctp_ili2511_read_raw_touch_data(touch_data,
                                                            MTB_CTP_ILI2511_TOUCH_NUM_BYTES);

    if (CY_RSLT_SUCCESS == result)
    {
        /* Consolidate x and y touch data in 32-bit format */
        current_touch_x = (touch_data[MTB_CTP_ILI2511_TOUCH_X_MSB_POS] << (BYTE_SHIFT)) |
                          touch_data[MTB_CTP_ILI2511_TOUCH_X_LSB_POS];
        current_touch_x = current_touch_x & (BIT_MASK);
        current_touch_y = (touch_data[MTB_CTP_ILI2511_TOUCH_Y_MSB_POS] << (BYTE_SHIFT)) |
                          touch_data[MTB_CTP_ILI2511_TOUCH_Y_LSB_POS];
        current_touch_y = current_touch_y & (BIT_MASK);

        /* To check the last touch position is equal to the current touch position */
        if ((last_touch_x != current_touch_x) || (last_touch_y != current_touch_y))
        {
            last_touch_x = current_touch_x;
            last_touch_y = current_touch_y;

            /* Calibrate x and y coordinates to map with the LCD display */
            *touch_x = (int)(((current_touch_x * (MTB_CTP_TOUCH_RESOLUTION_X)) /
                              MTB_CTP_ILI2511_CALIBRATION_VALUE) *
                             MTB_CTP_ILI2511_CALIBRATION_OFFSET);
            *touch_y = (int)(((current_touch_y * (MTB_CTP_TOUCH_RESOLUTION_Y)) /
                              MTB_CTP_ILI2511_CALIBRATION_VALUE) *
                             MTB_CTP_ILI2511_CALIBRATION_OFFSET);
        }
        else
        {
            result = (cy_rslt_t)CY_RSLT_ILI2511_NO_TOUCH;
        }
    }

    return result;
}


/*******************************************************************************
* Function name: mtb_ctp_ili2511_deinit
********************************************************************************
*
* De-initialize the CTP driver.
*
* \param void
*
* \return void
*
*******************************************************************************/
void mtb_ctp_ili2511_deinit(void)
{
    CY_ASSERT(NULL != ctp_config);

    /* Disable CTP IRQ*/
    NVIC_DisableIRQ(ctp_config->irq_num);

    /* Drive CTP RESET pin to LOW */
    Cy_GPIO_Write(ctp_config->rst_port, ctp_config->rst_pin, GPIO_LOW);

    /* Drive CTP IRQ pin to LOW */
    Cy_GPIO_Write(ctp_config->irq_port, ctp_config->irq_pin, GPIO_LOW);
}


/* [] END OF FILE */
