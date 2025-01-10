/*
 * tsl2561.c
 *
 *  Created on: Jan 10, 2025
 *      Author: Nikolaos Grigoriadis
 *      Email : n.grigoriadis09@gmail.com
 *      Title : Embedded software engineer
 *      Degree: BSc and MSc in computer science, university of Ioannina
 */

#include "tsl2561.h"

/*Static function declaration*/
static HAL_StatusTypeDef TSL2561_Write_Byte(TSLTypeDef *tsl2561, uint8_t reg,
                                            uint8_t data);
static HAL_StatusTypeDef TSL2561_Read_Byte(TSLTypeDef *tsl2561, uint8_t reg,
                                           uint8_t *recData);

/**
 * @brief Initializes the TSL2561 light sensor.
 *
 * This function initializes the TSL2561 sensor by setting its I2C handle and
 * address, powering it on, and verifying that the device is communicating
 * properly by reading the control register.
 *
 * @param tsl2561 Pointer to the TSL2561 structure that holds the sensor address
 * and I2C handle.
 * @param i2c_handle Pointer to the I2C handle used for communication with the
 * sensor.
 * @param address The I2C address of the TSL2561 sensor.
 *
 * @return TSL2561StatusType The status of the initialization.
 *         - TSL2561_OK: The sensor was initialized successfully.
 *         - TSL2561_FAIL: The initialization failed (either powering on or
 * communication verification failed).
 */
TSL2561StatusType TSL2561_Init(TSLTypeDef *tsl2561,
                               I2C_HandleTypeDef *i2c_handle,
                               uint32_t address) {
  /*Local variables*/
  int tsl2561_found = 0;
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /*Declare the sensor address*/
  tsl2561->address = address;

  /*Declare the sensor handle*/
  tsl2561->handle = i2c_handle;

  /*Start scanning in the I2C bus, to ensure device is proper connected with the
   * hardware*/
  tsl2561_found = TSL2561_I2C_Scan(tsl2561, tsl2561->handle);
  if (tsl2561_found == 0) {
    printf("[WARNING] TSL2561_I2C_Scan() failed! Please check the hardware "
           "connections or verify the slave address.\n");
    return TSL2561_FAIL;
  }

  /*Power on the device, and clear any pending interrupt*/
  result = TSL2561_Write_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_CONTROL_REG,
                              TSL2561_STATE_POWER_ON);
  if (result == HAL_OK) {

    /*Verify that the device is communicating properly*/
    result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_CONTROL_REG,
                               &recData);
    if (result == HAL_OK) {
      if ((recData & TSL2561_STATE_POWER_ON) == TSL2561_STATE_POWER_ON) {
        /*Verification success, device is powered on*/
        result = TSL2561_OK;
      } else {
        /*Verification failed, device is powered down*/
        result = TSL2561_FAIL;
      }
    }
  }

  /*User Code Begin*/

  // TODO: Extend the initialization code here if needed.

  /*User Code End*/

  /*Return result*/
  return result;
}

/**
 * @brief Reads the ID register of the TSL2561 sensor and stores the part number
 * and revision number.
 *
 * This function reads the ID register (Ah) of the TSL2561 sensor, extracts the
 * part number (PARTNO) and revision number (REVNO), and stores them in the
 * provided `TSLTypeDef` structure. The part number and revision number are
 * determined by reading the appropriate bits in the register.
 *
 * @param tsl2561 Pointer to the TSL2561 structure where the part number and
 * revision number will be stored.
 *
 * @note If the ID register read fails, an error message will be printed.
 *
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Get_Id(TSLTypeDef *tsl2561) {
  /*Local variables*/
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /*Read ID register*/
  result =
      TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_ID_REG, &recData);
  if (result == HAL_OK) {
    /* Save part number (bits 7:4) */
    tsl2561->partno = (TSL2561_PartNoType)((recData & 0xF0) >> 4);

    /* Save revision number (bits 3:0) */
    tsl2561->revno = (recData & 0x0F);

    return TSL2561_OK;
  }

  printf("[ERROR] TSL2561_Get_Id() failed!\n");

  return TSL2561_FAIL;
}

/**
 * @brief Updates the gain setting of the TSL2561 sensor.
 *
 * This function updates the gain setting in the timing register of the TSL2561
 * sensor. The gain setting determines whether the sensor operates in low-gain
 * or high-gain mode.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param gain Gain setting to be applied (low or high).
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Set_Gain(TSLTypeDef *tsl2561, TSL2561GainType gain) {
  /*Local variables*/
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /* Read the timing register */
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                             &recData);
  if (result == HAL_OK) {
    /* Clear the current gain bits (bit 4) */
    recData &= ~(1 << 4);

    /* Set the new gain */
    recData |= (gain << 4);

    /* Write back to the timing register */
    result = TSL2561_Write_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                                recData);
    if (result != HAL_OK) {
      return TSL2561_FAIL;
    }
  }

  return TSL2561_OK;
}

/**
 * @brief Manually start or stop the TSL2561 ADC.
 *
 * This function sets the manual start or stop operation in the timing register
 * of the TSL2561 sensor. Manual operation allows direct control over the ADC.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param operation Manual operation to perform (start or stop).
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Manual_Start_Stop(TSLTypeDef *tsl2561,
                                            TSL2561ManualSetupType operation) {
  /*Local variables*/
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /* Read the timing register */
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                             &recData);
  if (result == HAL_OK) {
    /* Clear the manual operation bit (bit 3) */
    recData &= ~(1 << 3);

    /* Set the manual operation bit */
    recData |= (operation << 3);

    /* Write back to the timing register */
    result = TSL2561_Write_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                                recData);
    if (result != HAL_OK) {
      return TSL2561_FAIL;
    }
  }

  return TSL2561_OK;
}

/**
 * @brief Set the integration time for the TSL2561 sensor.
 *
 * This function configures the integration time of the TSL2561 sensor. The
 * integration time determines the duration for light accumulation, affecting
 * the sensitivity and range of measurements.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param integrationTime Desired integration time (one of
 * TSL2561IntegrationTimeType values).
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Set_Itegration(TSLTypeDef *tsl2561,
                                         TSL2561IntegrationTimeType time) {
  /*Local variables*/
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /* Read the timing register */
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                             &recData);
  if (result == HAL_OK) {
    /* Clear the integration time bits (bits 0–1) */
    recData &= ~0x03;

    /* Set the integration time bit */
    recData |= (uint8_t)time;

    /* Write back to the timing register */
    result = TSL2561_Write_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_TIMING_REG,
                                recData);
    if (result != HAL_OK) {
      return TSL2561_FAIL;
    }
  }

  return TSL2561_OK;
}

/**
 * @brief Scan the I2C bus for connected devices and verify the TSL2561 sensor.
 *
 * This function scans the I2C bus for devices and checks if the TSL2561 sensor
 * is detected at its specified address.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param hi2c Pointer to the I2C handle.
 * @return int Returns 1 if the TSL2561 sensor is found, 0 otherwise.
 */
int TSL2561_I2C_Scan(TSLTypeDef *tsl2561, I2C_HandleTypeDef *hi2c) {
  printf("Scanning...\n");
  int found = 0;

  /* Scan through all possible 7-bit I2C addresses (1 to 127) */
  for (uint8_t i = 1; i < 127; i++) {
    if (HAL_I2C_IsDeviceReady(hi2c, (i << 1), 1, 10) == HAL_OK) {
      /* Check if the address matches the TSL2561's address */
      printf("Device found at address: 0x%02X\n", i);
      if (i == tsl2561->address) {
        found = 1;
      }
    }
  }
  printf("Scan complete.\n");

  /* Verify if the TSL2561 sensor was found */
  if (found) {
    printf("TSL2561 sensor found at address: 0x%02lX\n", tsl2561->address);
  } else {
    printf("[ERROR] TSL2561 sensor not detected at address: 0x%02lX\n",
           tsl2561->address);
  }

  return found;
}

/**
 * @brief Enable interrupt control select.
 *
 * This function configures the interrupt mode as specified by the INTR field of
 * the INTERRUPT register.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param interruptMode The desired interrupt mode (e.g., level interrupt,
 * SMBAlert).
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType
TSL2561_Enable_Interrupt_Control(TSLTypeDef *tsl2561,
                                 TSL2561IntCtrlType interruptMode) {
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /* Read the current register value */
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_INTERRUPT_REG,
                             &recData);
  if (result == HAL_OK) {
    /* Set the INTR field */
    recData &= ~(0x30);              // Clear the INTR bits (bits 4 and 5)
    recData |= (interruptMode << 4); // Set the new interrupt mode

    /* Write the modified value back to the register */
    result = TSL2561_Write_Byte(
        tsl2561, TSL2561_STATE_CMD | TSL2561_INTERRUPT_REG, recData);
    if (result != HAL_OK) {
      return TSL2561_FAIL;
    }
  }

  return TSL2561_OK;
}

/**
 * @brief Set the interrupt persistence.
 *
 * This function configures the interrupt persistence based on the specified
 * number of cycles.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param persistence The desired persistence value (0 to 15 cycles).
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType
TSL2561_Set_Interrupt_Persistence(TSLTypeDef *tsl2561,
                                  TSL2561IntPersistType persistence) {
  uint8_t recData = 0;
  HAL_StatusTypeDef result = HAL_ERROR;

  /* Read the current register value */
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_INTERRUPT_REG,
                             &recData);
  if (result == HAL_OK) {
    /* Set the PERSIST field (bits 0 to 3) */
    recData &= ~(0x0F);              // Clear the PERSIST bits (bits 0-3)
    recData |= (persistence & 0x0F); // Set the new persistence value

    /* Write the modified value back to the register */
    result = TSL2561_Write_Byte(
        tsl2561, TSL2561_STATE_CMD | TSL2561_INTERRUPT_REG, recData);
    if (result != HAL_OK) {
      return TSL2561_FAIL;
    }
  }

  return TSL2561_OK;
}

/**
 * @brief Clear the interrupt.
 *
 * This function clears the interrupt by writing the COMMAND register with the
 * CLEAR bit set.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Clear_Interrupt(TSLTypeDef *tsl2561) {
  uint8_t command =
      TSL2561_STATE_CMD | TSL2561_STATE_CLEAR; // Command to clear interrupt
  TSL2561StatusType result = TSL2561_FAIL;

  /*Write the clear interrupt command (0xC0) to the sensor*/
  if (HAL_I2C_Master_Transmit(tsl2561->handle, (tsl2561->address << 1),
                              &command, 1, HAL_MAX_DELAY) == HAL_OK) {
    result = TSL2561_OK; // Successful clear
  } else {
    printf("[ERROR] Failed to clear interrupt on TSL2561.\n");
  }

  return result;
}

/**
 * @brief Reads the raw data from Channel 0 and Channel 1 of the TSL2561 sensor.
 *
 * This function reads the low and high bytes of both Channel 0 and Channel 1
 * from their respective registers and combines them into 16-bit values. These
 * values represent the unprocessed raw ADC counts for visible + IR (Channel 0)
 * and IR-only (Channel 1).
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param ch0_val Pointer to store the 16-bit raw value of Channel 0.
 * @param ch1_val Pointer to store the 16-bit raw value of Channel 1.
 * @return TSL2561StatusType Status of the operation
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Read_Raw_Data(TSLTypeDef *tsl2561, uint16_t *ch0_val,
                                        uint16_t *ch1_val) {
  /*Local variables*/
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t data0low = 0, data0high = 0;
  uint8_t data1low = 0, data1high = 0;

  /* Read Channel 0 low and high bytes*/
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_DATA0LOW_REG,
                             &data0low);
  if (result != HAL_OK) {
    printf("[ERROR] Unable to read the lower bytes of channel 0, "
           "TSL2561_Read_Raw_Data() failed.\n");
    return TSL2561_FAIL;
  }

  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_DATA0HIGH_REG,
                             &data0high);
  if (result != HAL_OK) {
    printf("[ERROR] Unable to read the higher bytes of channel 0, "
           "TSL2561_Read_Raw_Data() failed.\n");
    return TSL2561_FAIL;
  }

  /* Combine Channel 0 low and high bytes into a 16-bit value*/
  *ch0_val = (uint16_t)((data0high << 8) | data0low);

  /* Read Channel 1 low and high bytes*/
  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_DATA1LOW_REG,
                             &data1low);
  if (result != HAL_OK) {
    printf("[ERROR] Unable to read the lower bytes of channel 1, "
           "TSL2561_Read_Raw_Data() failed.\n");
    return TSL2561_FAIL;
  }

  result = TSL2561_Read_Byte(tsl2561, TSL2561_STATE_CMD | TSL2561_DATA1HIGH_REG,
                             &data1high);
  if (result != HAL_OK) {
    printf("[ERROR] Unable to read the higher bytes of channel 1, "
           "TSL2561_Read_Raw_Data() failed.\n");
    return TSL2561_FAIL;
  }

  /* Combine Channel 1 low and high bytes into a 16-bit value*/
  *ch1_val = (uint16_t)((data1high << 8) | data1low);

  return TSL2561_OK;
}

/**
 * @brief Calculates the Lux value based on raw ADC data from the TSL2561
 * sensor.
 *
 * This function reads the raw data from ADC channels 0 and 1, calculates the
 * ratio of channel data, and uses coefficients defined in the TSL2561 datasheet
 * to compute the Lux value.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param part_number The part number of the TSL2561 sensor.
 * @return TSL2561StatusType Status of the operation:
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Get_Lux(TSLTypeDef *tsl2561,
                                  TSL2561_PartNoType part_number) {
  /* Local variables */
  TSL2561StatusType result = TSL2561_FAIL;
  uint16_t ch0_val = 0x00;
  uint16_t ch1_val = 0x00;
  float coefficient = 0.0;

  /* Read raw data from sensors ADC channels 0 and 1 */
  result = TSL2561_Read_Raw_Data(tsl2561, &ch0_val, &ch1_val);
  if (result == TSL2561_FAIL) {
    printf(
        "[ERROR] Unable to read raw data, TSL2561_Read_Raw_Data() failed!\n");
    return result;
  }

  /* Prevent division by zero */
  if (ch0_val == 0) {
    printf("[ERROR] Division by zero in coefficient calculation!\n");
    tsl2561->lux = 0;
    return TSL2561_FAIL;
  }

  /* Calculate the ratio of channel 1 to channel 0 */
  coefficient = (float)ch1_val / ch0_val;

  /* Determine Lux based on part number and coefficient */
  switch (part_number) {
  case TSL2560CS:
  case TSL2561CS:
    if (coefficient > 0 && coefficient <= 0.52)
      tsl2561->lux =
          (0.0315 * ch0_val) - (0.0593 * ch0_val * (pow(coefficient, 1.4)));
    else if (coefficient <= 0.65)
      tsl2561->lux = (0.0229 * ch0_val) - (0.0291 * ch1_val);
    else if (coefficient <= 0.80)
      tsl2561->lux = (0.0157 * ch0_val) - (0.0180 * ch1_val);
    else if (coefficient <= 1.30)
      tsl2561->lux = (0.00338 * ch0_val) - (0.00260 * ch1_val);
    else
      tsl2561->lux = 0;
    break;

  case TSL2560TFNCL:
  case TSL2561TFNCL:
    if (coefficient > 0 && coefficient <= 0.50)
      tsl2561->lux =
          (0.0304 * ch0_val) - (0.062 * ch0_val * (pow(coefficient, 1.4)));
    else if (coefficient <= 0.61)
      tsl2561->lux = (0.0224 * ch0_val) - (0.031 * ch1_val);
    else if (coefficient <= 0.80)
      tsl2561->lux = (0.0128 * ch0_val) - (0.0153 * ch1_val);
    else if (coefficient <= 1.30)
      tsl2561->lux = (0.00146 * ch0_val) - (0.00112 * ch1_val);
    else
      tsl2561->lux = 0;
    break;

  default:
    printf("[WARNING] Invalid part number! Please verify the device part "
           "number.\n");
    return TSL2561_FAIL;
  }

  return TSL2561_OK;
}

/**
 * @brief Sets the low interrupt threshold for the TSL2561 sensor.
 *
 * This function configures the low threshold that determines when an interrupt
 * is triggered if the ADC Channel 0 value is below or equal to this threshold.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param threshold The 16-bit low threshold value (0x0000 to 0xFFFF).
 * @return TSL2561StatusType Status of the operation:
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Set_Low_Threshold(TSLTypeDef *tsl2561,
                                            uint16_t threshold) {
  /*Local variables*/
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t low_byte = 0x00;
  uint8_t high_byte = 0x00;

  /*Extract low byte*/
  low_byte = threshold & 0xFF;

  /*Extract high byte*/
  high_byte = (threshold >> 8) & 0xFF;

  /*Write low and high bytes consecutively to the low threshold registers*/
  result = TSL2561_Write_Byte(
      tsl2561, TSL2561_STATE_CMD | TSL2561_THRESHLOWLOW_REG, low_byte);
  if (result != HAL_OK) {
    printf("[ERROR] In setting low byte threshold, TSL2561_Set_Low_Threshold() "
           "failed!\n");
    return TSL2561_FAIL;
  }

  result = TSL2561_Write_Byte(
      tsl2561, TSL2561_STATE_CMD | TSL2561_THRESHLOWHIGH_REG, high_byte);
  if (result != HAL_OK) {
    printf("[ERROR] In setting high byte threshold, "
           "TSL2561_Set_Low_Threshold() failed!\n");
    return TSL2561_FAIL;
  }

  return TSL2561_OK;
}

/**
 * @brief Sets the high interrupt threshold for the TSL2561 sensor.
 *
 * This function configures the high threshold that determines when an interrupt
 * is triggered if the ADC Channel 0 value exceeds this threshold.
 *
 * @param tsl2561 Pointer to the TSL2561 structure.
 * @param threshold The 16-bit high threshold value (0x0000 to 0xFFFF).
 * @return TSL2561StatusType Status of the operation:
 *      - TSL2561_OK on success,
 *      - TSL2561_FAIL on failure.
 */
TSL2561StatusType TSL2561_Set_High_Threshold(TSLTypeDef *tsl2561,
                                             uint16_t threshold) {
  /*Local variables*/
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t low_byte = 0x00;
  uint8_t high_byte = 0x00;

  /*Extract low byte*/
  low_byte = threshold & 0xFF;

  /*Extract high byte*/
  high_byte = (threshold >> 8) & 0xFF;

  /*Write low and high bytes consecutively to the high threshold registers*/
  result = TSL2561_Write_Byte(
      tsl2561, TSL2561_STATE_CMD | TSL2561_THRESHHIGHLOW_REG, low_byte);
  if (result != HAL_OK) {
    printf("[ERROR] In setting low byte threshold, TSL2561_Set_Low_Threshold() "
           "failed!\n");
    return TSL2561_FAIL;
  }

  result = TSL2561_Write_Byte(
      tsl2561, TSL2561_STATE_CMD | TSL2561_THRESHHIGHHIGH_REG, high_byte);
  if (result != HAL_OK) {
    printf("[ERROR] In setting high byte threshold, "
           "TSL2561_Set_Low_Threshold() failed!\n");
    return TSL2561_FAIL;
  }

  return TSL2561_OK;
}

/**
 * @brief Writes a single byte to a specified register of the TSL2561 sensor.
 *
 * This function sends a byte of data to a specific register of the TSL2561
 * sensor over I2C. It utilizes the HAL I2C memory write function to communicate
 * with the sensor's I2C interface.
 *
 * @param tsl2561 Pointer to the TSLTypeDef structure that contains the I2C
 * handle and address of the TSL2561 sensor.
 * @param reg The register address within the TSL2561 where the byte of data
 * should be written.
 * @param data The byte of data to write to the specified register.
 *
 * @return HAL_StatusTypeDef This function returns the status of the I2C write
 * operation.
 *         - HAL_OK: The write was successful.
 *         - HAL_ERROR: An error occurred during the write operation.
 */
static HAL_StatusTypeDef TSL2561_Write_Byte(TSLTypeDef *tsl2561, uint8_t reg,
                                            uint8_t data) {
  return HAL_I2C_Mem_Write(tsl2561->handle, (tsl2561->address << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads a single byte from a specified register of the TSL2561 sensor.
 *
 * This function reads a byte of data from a specific register of the TSL2561
 * sensor over I2C. It uses the HAL I2C memory read function to communicate with
 * the sensor's I2C interface.
 *
 * @param tsl2561 Pointer to the TSLTypeDef structure that contains the I2C
 * handle and address of the TSL2561 sensor.
 * @param reg The register address within the TSL2561 from which the byte of
 * data should be read.
 * @param recData Pointer to a variable where the received data byte will be
 * stored.
 *
 * @return HAL_StatusTypeDef This function returns the status of the I2C read
 * operation.
 *         - HAL_OK: The read was successful.
 *         - HAL_ERROR: An error occurred during the read operation.
 */
static HAL_StatusTypeDef TSL2561_Read_Byte(TSLTypeDef *tsl2561, uint8_t reg,
                                           uint8_t *recData) {
  return HAL_I2C_Mem_Read(tsl2561->handle, (tsl2561->address << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, recData, 1, HAL_MAX_DELAY);
}
