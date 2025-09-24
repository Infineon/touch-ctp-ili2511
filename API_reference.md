# ILI2511 capacitive touch panel driver library API reference guide

## General description

This library provides a basic set of functions to support Capacitive Touch Panel (CTP) of 10.1 inch TFT LCD [WF101JTYAHMNB0](https://www.winstar.com.tw/products/tft-lcd/ips-tft/ips-touch.html) driven by [ILI2511](https://www.crystalfontz.com/controllers/datasheet-viewer.php?id=487) controller. 

For more information about the TFT display, see [WF101JTYAHMNB0](https://www.winstar.com.tw/products/tft-lcd/ips-tft/ips-touch.html) display.


## Code snippets

### Snippet 1: Touch driver initialization and reading touch data

The following snippet initializes the touch panel driver and reads touch data from it.

```
/* ILI2511 touch controller configuration. */
mtb_ctp_ili2511_config_t ctp_ili2511_cfg =
{
    .scb_inst            = CYBSP_I2C_CONTROLLER_HW,
    .i2c_context         = NULL,
    .rst_port            = GPIO_PRT17,
    .rst_pin             = 3U,
    .irq_port            = GPIO_PRT17,
    .irq_pin             = 2U,
    .irq_num             = ioss_interrupts_gpio_17_IRQn,
    .touch_event         = false,
};

/* Performs graphics subsystem and I2C initialization prior to touch driver initialization. */

/* Adds the valid I2C context in the touch controller configuration after I2C init. */
ctp_ili2511_cfg.i2c_context = &i2c_cntxt;

/* Initializes ILI2511 touch driver. */
result = mtb_ctp_ili2511_init(&ctp_ili2511_cfg);
if (CY_RSLT_SUCCESS != result)
{
    /* Handles possible errors. */
    CY_ASSERT(0);
}

/* Reads the touch coordinates when a touch event is detected.
 * Invokes this function within the input device read callback registered to
 * the graphics framework or periodically/callback to the touch interrupt in 
 * a bare metal application.
 */
if (ctp_ili2511_cfg.touch_event)
{
    /* Resets the touch detection event once set in the touch IRQ handler. */
    ctp_ili2511_cfg.touch_event = false;

    /* Reads the touch coordinates and takes action based on the UI event. */
    result = mtb_ctp_ili2511_get_single_touch(&touch_x, &touch_y);
    if (CY_RSLT_SUCCESS != result)
    {
        /* Handles possible errors. */
        CY_ASSERT(0);
    }
}
```

## Macros

```
CY_RSLT_ILI2511_ERR_BASE                               Error code base 
CY_RSLT_ILI2511_NO_TOUCH                               No touch detection error code
CY_RSLT_ILI2511_DEV_ERR                                Device error code
MTB_CTP_TOUCH_RESOLUTION_X                             X coordinate touch resolution                   
MTB_CTP_TOUCH_RESOLUTION_Y                             Y coordinate touch resolution        
MTB_CTP_ILI2511_I2C_SLAVE_ADDR                         Touch controller I2C slave address 
MTB_CTP_ILI2511_IRQ_PRIORITY                           Touch interrupt priority
MTB_CTP_ILI2511_TOUCH_INFO_CMD                         ILI2511 read touch coordinates command register
MTB_CTP_ILI2511_GET_FIRMWARE_VER_CMD                   ILI2511 read firmware command register
MTB_CTP_ILI2511_CHIP_ID_CODE                           ILI2511 chip ID
MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER                     ILI2511 major firmware version value
MTB_CTP_ILI2511_MINOR_FIRMWARE_VER                     ILI2511 minor firmware version value
MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER                   ILI2511 release firmware version value
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4           ILI2511 customer firmware version byte 4
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5           ILI2511 customer firmware version byte 5
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6           ILI2511 customer firmware version byte 6
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7           ILI2511 customer firmware version byte 7
MTB_CTP_ILI2511_CALIBRATION_OFFSET                     ILI2511 touch controller calibration offset
MTB_CTP_ILI2511_CALIBRATION_VALUE                      ILI2511 touch controller calibration value
MTB_CTP_ILI2511_CHIP_ID_POS                            ILI2511 touch controller CHIP ID position/register offset
MTB_CTP_ILI2511_MAJOR_FIRMWARE_VER_POS                 ILI2511 touch controller major firmware version position/register offset
MTB_CTP_ILI2511_MINOR_FIRMWARE_VER_POS                 ILI2511 touch controller minor firmware version position/register offset
MTB_CTP_ILI2511_RELEASE_FIRMWARE_VER_POS               ILI2511 touch controller release firmware version position/register offset
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_4_POS       ILI2511 touch controller customer firmware version byte 4 position/register offset
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_5_POS       ILI2511 touch controller customer firmware version byte 5 position/register offset
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_6_POS       ILI2511 touch controller customer firmware version byte 6 position/register offset
MTB_CTP_ILI2511_CUSTOMER_FIRMWARE_VER_BYTE_7_POS       ILI2511 touch controller customer firmware version byte 7 position/register offset
MTB_CTP_ILI2511_FIRMWARE_NUM_BYTES                     Total number of ILI2511 touch controller firmware data bytes
MTB_CTP_ILI2511_TOUCH_X_MSB_POS                        Position/register offset for MSB of the touch X coordinate 
MTB_CTP_ILI2511_TOUCH_X_LSB_POS                        Position/register offset for LSB of the touch X coordinate 
MTB_CTP_ILI2511_TOUCH_Y_MSB_POS                        Position/register offset for MSB of the touch Y coordinate 
MTB_CTP_ILI2511_TOUCH_Y_LSB_POS                        Position/register offset for LSB of the touch Y coordinate 
MTB_CTP_ILI2511_TOUCH_NUM_BYTES                        Total number of touch data bytes
```

## Data structures
```
struct mtb_ctp_ili2511_config_t    Structure holding ILI2511 touch controller configuration
```

## Functions
void `mtb_ctp_ili2511_irq_handler(void)`
- Touch panel GPIO interrupt handler to detect touch events on the panel

cy_rslt_t `mtb_ctp_ili2511_init(mtb_ctp_ili2511_config_t* ctp_ili2511_config)`
- Performs ILI2511 touch controller driver initialization using I2C interface

cy_rslt_t `mtb_ctp_ili2511_get_single_touch(int* touch_x, int* touch_y)`
- Reads raw touch coordinates, calibrate, and returns actual x, y touch coordinates

void `mtb_ctp_ili2511_deinit(void)`
- De-initializes the CTP driver

## Data structure documentation

- mtb_ctp_ili2511_config_t

  Data type                 |           Member variable      |    Description
  :-------                  |  :------------         |  :------------
  CySCB_Type*            | scb_inst             |  Pointer to the SCB instance register base address
  cy_stc_scb_i2c_context_t*                  | i2c_context              |  Pointer to the I2C context configuration structure
  GPIO_PRT_Type*               | rst_port              |  Pointer to the touch controller reset pin's port register base
  uint32_t  | rst_pin              |  Touch controller reset pin
  GPIO_PRT_Type*                  | irq_port              |  Pointer to the touch controller interrupt pin's port register base
  uint32_t  | irq_pin              |  Touch controller interrupt pin
  IRQn_Type  | irq_num              |  Touch controller interrupt number
  bool  | touch_event              |  Touch controller interrupt status

  <br>

## Function documentation

#### mtb_ctp_ili2511_irq_handler
- void `mtb_ctp_ili2511_irq_handler(void)`

  **Summary:** Touch panel GPIO interrupt handler to detect touch events on the panel. It is a weak function, which can be overridden in application as per requirement.

#### mtb_ctp_ili2511_init

- cy_rslt_t `mtb_ctp_ili2511_init(mtb_ctp_ili2511_config_t* ctp_ili2511_config)`

  **Summary:** Performs ILI2511 touch controller driver initialization using an I2C interface.
  
  **Parameter**

   Parameters            |  Description       
   :-------              |  :------------
   [in] ctp_ili2511_config    |  Pointer to the ILI2511 touch controller configuration structure
   
   <br>

   **Return**
   - cy_rslt_t: Initialization status. CY_RSLT_SUCCESS on success else CY_RSLT_ILI2511_DEV_ERR, I2C error(s)

#### mtb_ctp_ili2511_get_single_touch

- cy_rslt_t `mtb_ctp_ili2511_get_single_touch(int* touch_x, int* touch_y)`

  **Summary:** This function fetches raw touch coordinates using the internal "mtb_ctp_ili2511_read_raw_touch_data" function, applies calibration and returns actual X, Y touch coordinates.
 
  **Parameter**

   Parameters            |  Description       
   :-------              |  :------------
   [in] touch_x    |  Pointer to integer variable to hold touch X coordinate
   [in] touch_y    |  Pointer to integer variable to hold touch Y coordinate

   <br>

   **Return**
   - cy_rslt_t: Result of reading single touch data operation, CY_RSLT_SUCCESS on success else CY_RSLT_ILI2511_NO_TOUCH, I2C error(s)

#### mtb_ctp_ili2511_deinit

- void `mtb_ctp_ili2511_deinit(void)`

  **Summary:** De-initializes the CTP driver.

---
© 2025, Cypress Semiconductor Corporation (an Infineon company)
