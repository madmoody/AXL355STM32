/** Author :Pavan Moody
 **/

#ifndef ADXL335_I2C_DRIVER_H    //
#define ADXL335_I2C_DRIVER_H

#include "stm32f4xx_hal.h" //need for i2c

/*
 * Defines
 */

#define ADXL355_I2C_ADDR   (0x1D << 1)  // ASEL = 0 Device address 0x1D,ASEL = 1 Device Address 0x53 here address bit length 7 so we shift left

#define ADXL355_DEVICE_ID 0XAD
#define ADXL355_MEMS_ID   0X1D
#define ADXL355_PART_ID   0XED

/*
 * REGISTERS
 */
#define ADXL355_REG_DEVID_AD		0X00
#define ADXL355_REG_DEVID_MST		0X01
#define ADXL355_REG_PARTID			0X02
#define ADXL355_REG_REVID			0X03
#define ADXL355_REG_STATUS			0x04
#define ADXL355_REG_FIFO_ENTRIES    0x05
#define ADXL355_REG_TEMP2			0X06
#define ADXL355_REG_TEMP1			0x07
#define ADXL355_REG_XDATA3			0X08
#define ADXL355_REG_XDATA2			0X09
#define ADXL355_REG_XDATA1			0x0A
#define ADXL355_REG_YDATA3			0x0B
#define ADXL355_REG_YDATA2			0x0C
#define ADXL355_REG_YDATA1			0x0D
#define ADXL355_REG_ZDATA3			0X0E
#define ADXL355_REG_ZDATA2			0X0F
#define ADXL355_REG_ZDATA1			0X10
#define ADXL355_REG_FIFO_DATA		0X11
#define ADXL355_REG_OFFSET_X_H		0x1E
#define ADXL355_REG_OFFSET_X_L		0x1F
#define ADXL355_REG_FILTER			0x28
#define ADXL355_REG_POWER_CTL		0x2D

/**
 * Sesor struct
 */

typedef struct{

	I2C_HandleTypeDef *i2cHandle;

	float acc_mps2[3];

	float temp_C;

}ADXL355;

/**
 * INITIALISATIOM
 */
uint8_t ADXL355_Initialise(ADXL355 *dev,I2C_HandleTypeDef *i2cHandle);

/*
 * DATA ACQUIISTION
 */
HAL_StatusTypeDef ADXL355_ReadTemperature(ADXL355 *dev);
HAL_StatusTypeDef ADXL355_ReadAccelerations(ADXL355 *dev);

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355 *dev,uint8_t reg,uint8_t *data);
HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355 *dev,uint8_t reg,uint8_t *data,uint8_t length)
HAL_StatusTypeDef ADXL355_WriteRegister(ADXL355 *dev,uint8_t reg,uint8_t *data);
#endif
