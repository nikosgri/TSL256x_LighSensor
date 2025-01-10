# TSL2561 Sensor Library

This library provides an interface to control and interact with the **TSL2561 Light-to-Digital Converter**, a sensor that converts ambient light intensity into a digital signal for applications such as display brightness control and energy-efficient lighting systems.

## Features

- **I2C Communication**: Supports easy interfacing with microcontrollers.
- **16-bit Output**: Captures high-resolution light intensity data.
- **Gain Control**: Operates in low-gain or high-gain modes for a wide range of light conditions.
- **Programmable Integration Time**: Adjust sensitivity by configuring integration duration.
- **Interrupt Handling**: Built-in interrupt capabilities with customizable thresholds and persistence.
- **Dynamic Range**: Supports up to 1,000,000:1 dynamic range.
- **Power Management**: Low-power mode and configurable power states.

## Getting Started

### Prerequisites

- Microcontroller with I2C interface.
- TSL2561 sensor module.
- HAL drivers for STM32 or similar development environments.

### Installation

1. Clone this repository or copy the source files:
   ```bash
   git clone https://github.com/nikosgri/TSL256x_LighSensor.git
   ```
2. Include tsl2561.c and tsl2561.h in your project.
3. Ensure your project is configured to use the HAL drivers for I2C.
   
## Usage
```
/*       ADDR_PIN
 * If float 0x39 (Default)
 * If Connected to GND 0x29
 * If Connected to VDD 0x49
 */
#define TSL2561_ADDRESS     0x39

TSLTypeDef tsl2561;
TSL2561StatusType status;

/*Initialize the sensor*/
status = TSL2561_Init(&tsl2561, &hi2c1, TSL2561_ADDRESS);
if (status == TSL2561_OK)
{
  printf("[INFO] Device powered on, successful initialization!\n");
}
else if (status == TSL2561_FAIL)
{
  printf("[ERROR] Initialization failed, device powered down.\n");
}

/*Get sensor's part and revision number*/
TSL2561_Get_Id(&tsl2561);

/*Take sensor lux value*/
res = TSL2561_Get_Lux(&tsl2561, TSL2561TFNCL);
if (res == TSL2561_FAIL)
{
  printf("[ERROR] In reading LUX value, TSL2561_Get_Lux() failed\n");
  return 0;
}
printf("Lux: %f\n", tsl2561.lux);
```

## Interrupt Configuration
Set interrupt thresholds and enable interrupt mode:
```
TSL2561_Set_Low_Threshold(&tsl2561, 100);
TSL2561_Set_High_Threshold(&tsl2561, 1000);
TSL2561_Enable_Interrupt_Control(&tsl2561, TSL2561_INT_CTRL_LEVEL);
```
## API Reference
### Functions
- **Initialization**
    - TSL2561_Init
    - TSL2561_Get_Id
- **Data Reading**
    - TSL2561_Read_Raw_Data
    - TSL2561_Get_Lux
- **Configuration**
    - TSL2561_Set_Gain
    - TSL2561_Set_Itegration
    - TSL2561_Enable_Interrupt_Control
    - TSL2561_Set_Low_Threshold
    - TSL2561_Set_High_Threshold
    - TSL2561_Set_Interrupt_Persistence
    - TSL2561_Clear_Interrupt
- **Enumerations**
    - TSL2561GainType: Gain settings.
    - TSL2561IntegrationTimeType: Integration times.
    - TSL2561IntCtrlType: Interrupt control types.
- **Structures**
    - **TSLTypeDef**
        - lux: Measured light intensity in lux.
        - address: I2C address.
        - revno: Revision number.
        - partno: Part number.
        - handle: I2C handle.
## Contributing
Feel free to open issues or submit pull requests for improvements.
