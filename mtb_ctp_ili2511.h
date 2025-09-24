/*******************************************************************************
* \file mtb_ctp_ili2511.h
*
* \brief
* Provides constants, parameter values, and API prototypes for the ILI2511
* touch driver library.
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

#ifndef MTB_CTP_ILI2511_H
#define MTB_CTP_ILI2511_H


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cy_result.h"
#include <stdbool.h>


/*******************************************************************************
* Macros
*******************************************************************************/
/* Error code definition */
#define CY_RSLT_ILI2511_ERR_BASE                          (0x0253)

/* No touch detected by the driver */
#define CY_RSLT_ILI2511_NO_TOUCH                           \
                CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_ILI2511_ERR_BASE, 1)

/* ILI2511 device error */
#define CY_RSLT_ILI2511_DEV_ERR                            \
               CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_ILI2511_ERR_BASE, 2)

#define MTB_CTP_TOUCH_RESOLUTION_X                        (1024U)
#define MTB_CTP_TOUCH_RESOLUTION_Y                        (600U)

/* CTP I2C slave address */
#define MTB_CTP_ILI2511_I2C_SLAVE_ADDR                    (0x41)

/* CTP IRQ priority */
#define MTB_CTP_ILI2511_IRQ_PRIORITY                      (3U)

/* ILI2511 command codes */
#define MTB_CTP_ILI2511_TOUCH_INFO_CMD                    (0x10)
#define MTB_CTP_ILI2511_GET_FIRMWARE_VER_CMD              (0x40)

/* ILI511 firmware data byte positions */
#define MTB_CTP_ILI2511_CHIP_ID_POS                       (0U)
#define MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER_POS            (1U)
#define MTB_CTP_ILI2511_MINOR_FIRMWARE_VER_POS            (2U)
#define MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER_POS          (3U)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4_POS  (4U)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5_POS  (5U)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6_POS  (6U)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7_POS  (7U)
#define MTB_CTP_ILI2511_FIRMWARE_NUM_BYTES                (8U)

/* ILI2511 data values */
#define MTB_CTP_ILI2511_CHIP_ID_CODE                      (0x06)
#define MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER                (0x00)
#define MTB_CTP_ILI2511_MINOR_FIRMWARE_VER                (0x00)
#define MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER              (0x00)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4      (0x62)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5      (0x90)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6      (0x01)
#define MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7      (0x02)

/* ILI2511 touch panel coordinate data byte positions */
#define MTB_CTP_ILI2511_TOUCH_X_MSB_POS                   (1U)
#define MTB_CTP_ILI2511_TOUCH_X_LSB_POS                   (2U)
#define MTB_CTP_ILI2511_TOUCH_Y_MSB_POS                   (3U)
#define MTB_CTP_ILI2511_TOUCH_Y_LSB_POS                   (4U)
#define MTB_CTP_ILI2511_TOUCH_NUM_BYTES                   (31U)

/* MTB_CTP_ILI2511_CALIBRATION_OFFSET used to align with the actual coordinate */
#define MTB_CTP_ILI2511_CALIBRATION_OFFSET                (4U)

#define MTB_CTP_ILI2511_CALIBRATION_VALUE                 (65535U)


/*******************************************************************************
* Data Structures
*******************************************************************************/
/* ILI2511 touch controller configuration structure */
typedef struct
{
    CySCB_Type* scb_inst;
    cy_stc_scb_i2c_context_t* i2c_context;
    GPIO_PRT_Type* rst_port;
    uint32_t rst_pin;
    GPIO_PRT_Type* irq_port;
    uint32_t irq_pin;
    IRQn_Type irq_num;
    volatile bool touch_event;
} mtb_ctp_ili2511_config_t;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void mtb_ctp_ili2511_irq_handler(void);
cy_rslt_t mtb_ctp_ili2511_init(mtb_ctp_ili2511_config_t* ctp_ili2511_config);
cy_rslt_t mtb_ctp_ili2511_get_single_touch(int* touch_x, int* touch_y);
void mtb_ctp_ili2511_deinit(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */


#endif /* MTB_CTP_ILI2511_H */


/* [] END OF FILE */
