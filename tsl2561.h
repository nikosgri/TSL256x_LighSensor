/*
 * tsl2561.h
 *
 *  Created on: Jan 10, 2025
 *      Author: Nikolaos Grigoriadis
 *      Email : n.grigoriadis09@gmail.com
 *      Title : Embedded software engineer
 *      Degree: BSc and MSc in computer science, university of Ioannina
 */

#ifndef INC_TSL2561_H_
#define INC_TSL2561_H_

#include "main.h"
#include "math.h"
#include "stdio.h"

/**
 * @defgroup TSL2561_Registers TSL2561 Register Addresses
 * @brief Register addresses for TSL2561 sensor.
 *
 * These defines are used to access specific sensor functionality via I2C
 * communication.
 * @{
 */
#define TSL2561_CONTROL_REG 0x00 /**< Power control register. */
#define TSL2561_TIMING_REG                                                     \
  0x01 /**< Integration time and gain control register. */
#define TSL2561_THRESHLOWLOW_REG                                               \
  0x02 /**< Low byte of low-threshold for interrupt. */
#define TSL2561_THRESHLOWHIGH_REG                                              \
  0x03 /**< High byte of low-threshold for interrupt. */
#define TSL2561_THRESHHIGHLOW_REG                                              \
  0x04 /**< Low byte of high-threshold for interrupt. */
#define TSL2561_THRESHHIGHHIGH_REG                                             \
  0x05 /**< High byte of high-threshold for interrupt. */
#define TSL2561_INTERRUPT_REG 0x06 /**< Interrupt control register. */
#define TSL2561_CRC_REG                                                        \
  0x08 /**< Factory test register (not used in normal operation). */
#define TSL2561_ID_REG 0x0A        /**< Identification register. */
#define TSL2561_DATA0LOW_REG 0x0C  /**< Low byte of channel 0 data. */
#define TSL2561_DATA0HIGH_REG 0x0D /**< High byte of channel 0 data. */
#define TSL2561_DATA1LOW_REG 0x0E  /**< Low byte of channel 1 data. */
#define TSL2561_DATA1HIGH_REG 0x0F /**< High byte of channel 1 data. */
/** @} */

/**
 * @defgroup TSL2561_Command_Register Command Register Definitions
 * @brief Defines for the TSL2561 command register and its bits.
 *
 * The command register specifies the address of the target register for
 * subsequent read and write operations. It uses the Send Byte protocol for
 * configuration and defaults to `0x00` at power-on.
 * @{
 */
#define TSL2561_STATE_CMD                                                      \
  0x80 /**< CMD: Selects the command register. Must always be set to 1. */
#define TSL2561_STATE_CLEAR                                                    \
  0x40 /**< CLEAR: Write 1 to clear pending interrupt. Self-clearing. */
#define TSL2561_STATE_WORD                                                     \
  0x20 /**< WORD: Enables SMBus Write/Read Word Protocol. */
#define TSL2561_STATE_BLOCK                                                    \
  0x10 /**< BLOCK: Enables SMBus Block Write/Read Protocol. */
#define TSL2561_STATE_ADDRESS                                                  \
  0x0F /**< Mask for address bits in the command register. */
#define TSL2561_STATE_POWER_ON 0x03  /**< Power on the sensor. */
#define TSL2561_STATE_POWER_OFF 0x00 /**< Power off the sensor. */
/** @} */

/**
 * @brief Enum for TSL2561 sensor part numbers.
 *
 * @note This enumeration defines the part numbers for the TSL2561 sensor
 * family. The part numbers are represented by 4-bit values as described in the
 * device's ID register. The part number is used to identify specific variants
 * of the TSL2561 sensor.
 */
typedef enum {
  TSL2560CS = 0x00,    // 0000
  TSL2561CS = 0x01,    // 0001
  TSL2560TFNCL = 0x04, // 0100
  TSL2561TFNCL = 0x05  // 0101
} TSL2561_PartNoType;

/**
 * @enum TSL2561StatusType
 * @brief Status codes for the TSL2561 sensor operations.
 *
 * @note This enum defines the possible outcomes of operations involving the
 * TSL2561 sensor.
 */
typedef enum {
  TSL2561_OK,  /**< Operation completed successfully. */
  TSL2561_FAIL /**< Operation failed. */
} TSL2561StatusType;

/**
 * @brief Enum for TSL2561 gain settings.
 *
 * @note This enum defines the gain modes for the TSL2561 sensor.
 * Gain settings control the sensitivity of the light sensor.
 */
typedef enum {
  TSL2561_LOW_GAIN = 0x00, /**< Writing a 0 selects low gain (1x). */
  TSL2561_HIGH_GAIN = 0x01 /**< writing a 1 selects high gain (16×). */
} TSL2561GainType;

/**
 * @brief Enum for TSL2561 manual setup operations.
 *
 * @note This enum defines the manual operations to control the integration
 * cycle of the TSL2561 sensor. These settings allow starting or stopping the
 * ADC integration process manually.
 */
typedef enum {
  TSL2561_MANUAL_STOP = 0x00, /**< Writing a 0 stops an integration cycle.*/
  TSL2561_MANUAL_BEGIN = 0x01 /**< Writing a 1 begins an integration cycle.*/
} TSL2561ManualSetupType;

/**
 * @brief Enum for TSL2561 nominal integration time settings.
 *
 * @note This enum defines the possible integration time settings for the
 * TSL2561 sensor. The integration time determines the duration over which the
 * sensor collects light data.
 */
typedef enum {
  TSL2561_INTEG_13MS =
      0x00, /**< Integration time of 13.7 ms (field value: 000). */
  TSL2561_INTEG_101MS =
      0x01, /**< Integration time of 101 ms (field value: 010). */
  TSL2561_INTEG_402MS =
      0x02, /**< Integration time of 402 ms (field value: 101). */
  TSL2561_INTEG_NA =
      0x03 /**< Reserved or invalid integration time (field value: 11). */
} TSL2561IntegrationTimeType;

/**
 * @brief Enumeration for Interrupt Control Select.
 *
 * @note Specifies the behavior of the interrupt output.
 */
typedef enum {
  TSL2561_INT_CTRL_DISABLED = 0x00,  /**< Interrupt output disabled. */
  TSL2561_INT_CTRL_LEVEL = 0x01,     /**< Level interrupt. */
  TSL2561_INT_CTRL_SMB_ALERT = 0x02, /**< SMBAlert compliant mode. */
  TSL2561_INT_CTRL_TEST_MODE =
      0x03 /**< Test mode: Sets interrupt and functions as SMBAlert mode. */
} TSL2561IntCtrlType;

/**
 * @brief Enumeration for Interrupt Persistence Select.
 *
 * @note Specifies the interrupt persistence function, determining how many
 * integration time periods the sensor waits before generating an interrupt when
 * a value is out of the threshold range.
 */
typedef enum {
  TSL2561_INT_PERSIST_EVERY_CYCLE =
      0x00, /**< Every ADC cycle generates an interrupt. */
  TSL2561_INT_PERSIST_OUTSIDE_RANGE =
      0x01, /**< Any value outside of threshold range. */
  TSL2561_INT_PERSIST_2_CYCLES =
      0x02, /**< 2 integration time periods out of range. */
  TSL2561_INT_PERSIST_3_CYCLES =
      0x03, /**< 3 integration time periods out of range. */
  TSL2561_INT_PERSIST_4_CYCLES =
      0x04, /**< 4 integration time periods out of range. */
  TSL2561_INT_PERSIST_5_CYCLES =
      0x05, /**< 5 integration time periods out of range. */
  TSL2561_INT_PERSIST_6_CYCLES =
      0x06, /**< 6 integration time periods out of range. */
  TSL2561_INT_PERSIST_7_CYCLES =
      0x07, /**< 7 integration time periods out of range. */
  TSL2561_INT_PERSIST_8_CYCLES =
      0x08, /**< 8 integration time periods out of range. */
  TSL2561_INT_PERSIST_9_CYCLES =
      0x09, /**< 9 integration time periods out of range. */
  TSL2561_INT_PERSIST_10_CYCLES =
      0x0A, /**< 10 integration time periods out of range. */
  TSL2561_INT_PERSIST_11_CYCLES =
      0x0B, /**< 11 integration time periods out of range. */
  TSL2561_INT_PERSIST_12_CYCLES =
      0x0C, /**< 12 integration time periods out of range. */
  TSL2561_INT_PERSIST_13_CYCLES =
      0x0D, /**< 13 integration time periods out of range. */
  TSL2561_INT_PERSIST_14_CYCLES =
      0x0E, /**< 14 integration time periods out of range. */
  TSL2561_INT_PERSIST_15_CYCLES =
      0x0F /**< 15 integration time periods out of range. */
} TSL2561IntPersistType;

/**
 * @struct TSLTypeDef
 * @brief Represents the TSL2561 sensor configuration and runtime data.
 *
 * @note This structure holds the light intensity measurement, the I2C address
 * of the sensor, and the I2C handle used for communication.
 */
typedef struct {
  float lux; /**< Measured light intensity in lux. This value represents the
                current light level detected by the sensor. */
  uint32_t address; /**< I2C address of the sensor. The address is used for
                       communication with the sensor over the I2C bus. */
  uint32_t revno;   /**< The revision number of the TSL2561 sensor. This
                       identifies the version of the sensor's silicon. */
  TSL2561_PartNoType partno; /**< The part number of the TSL2561 sensor. This
                                identifies the specific model of the sensor. */
  I2C_HandleTypeDef *
      handle; /**< Pointer to the I2C handle used for communication. This handle
                 is used to interface with the sensor over the I2C bus. */
} TSLTypeDef;

/*Function Prototypes*/

int TSL2561_I2C_Scan(TSLTypeDef *tsl2561, I2C_HandleTypeDef *hi2c);
TSL2561StatusType TSL2561_Get_Id(TSLTypeDef *tsl2561);
TSL2561StatusType TSL2561_Init(TSLTypeDef *tsl2561,
                               I2C_HandleTypeDef *i2c_handle, uint32_t address);
TSL2561StatusType TSL2561_Set_Gain(TSLTypeDef *tsl2561, TSL2561GainType gain);
TSL2561StatusType TSL2561_Manual_Start_Stop(TSLTypeDef *tsl2561,
                                            TSL2561ManualSetupType operation);
TSL2561StatusType TSL2561_Set_Itegration(TSLTypeDef *tsl2561,
                                         TSL2561IntegrationTimeType time);
TSL2561StatusType
TSL2561_Enable_Interrupt_Control(TSLTypeDef *tsl2561,
                                 TSL2561IntCtrlType interruptMode);
TSL2561StatusType
TSL2561_Set_Interrupt_Persistence(TSLTypeDef *tsl2561,
                                  TSL2561IntPersistType persistence);
TSL2561StatusType TSL2561_Clear_Interrupt(TSLTypeDef *tsl2561);
TSL2561StatusType TSL2561_Read_Raw_Data(TSLTypeDef *tsl2561, uint16_t *ch0_val,
                                        uint16_t *ch1_val);
TSL2561StatusType TSL2561_Get_Lux(TSLTypeDef *tsl2561,
                                  TSL2561_PartNoType part_number);
TSL2561StatusType TSL2561_Set_Low_Threshold(TSLTypeDef *tsl2561,
                                            uint16_t threshold);
TSL2561StatusType TSL2561_Set_High_Threshold(TSLTypeDef *tsl2561,
                                             uint16_t threshold);

#endif /* INC_TSL2561_H_ */
