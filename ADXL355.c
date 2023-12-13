#include "adxl.h"
uint8_t ADXL355_Initialise(ADXL355 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle		= i2cHandle;
	dev->acc_mps2[0]    = 0.0f;
	dev->acc_mps2[1]    = 0.0f;
	dev->acc_mps2[2] 	= 0.0f;

	dev->temp_c			= 0.0f;

	uint8_t errNum = 0;  // store number of transaction error
	HAL_StatusTypeDef status;

	uint8_t regData;

	status = ADXL355_ReadRegister(dev,ADXL355_REG_DEVID_AD, &regData); // check status if initialisation and device ,mems adn aparts IDs
	errNum += ( status != HAL_OK);

	if(regData != ADXL355_DEVICE_ID){
		return 255;
	}

	status = ADXL355_ReadRegister(dev,ADXL355_REG_DEVID_MST, &regData);
	errNum += ( status != HAL_OK);

		if(regData != ADXL355_MEMS_ID){
			return 255;
		}

	status = ADXL355_ReadRegister(dev,ADXL355_REG_DEVID_MST, &regData);
	errNum += ( status != HAL_OK);

		if(regData != ADXL355_MEMS_ID){
			return 255;
		}

/*
 * Set output data rate (ODR) and digital filters (N0 high pass filters, 125 Hx ODR, 31.25 Hz Low pass filter)
 */
regData = 0x05;

status = ADXL355_WriteRegister(dev, ADXL355_REG_FILTER, &regData);
errNum += (status != HAL_OK);
return errNum;

/*
 * Put Sensor into measurement Mode PwoerControlMode
 */

regData = 0x00;

status = ADXL355_WriteRegister(dev, ADXL355_REG_POWER_CTL, &regData);
errNum += (status != HAL_OK);
/* Return number of errors (0 id successful initialisation) */

return errNum;

}

/*
 * Data Acquistion
 */
HAL_StatusTypeDef ADXL355_ReadTemperature(ADXL355 *dev){

/*
 * Read Raw values From temperature
 */
	uint8_t regData[2];

	HAL_StatusTypeDef status = ADXL355_ReadRegister(dev,ADXL355_REG_TEMP2,regData, 2);

	/*
	 * regDATA[0] Temp2 regData[1] Temp1 combine register values to give raw temperature reading
	 */

	uint16_t tempRaw = (((regData[0] & 0x0F) << 8) | regData[1] );

	/*
	 * covert to deg c (offset @ 25 degC = 1852 LSb, slope = -9.05 LSB/degC)
	 */

	dev->temp_c = -0.11049723756f * ((float) tempRaw - 1852.0f) + 25.0f;
	return status;
}






/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355 *dev,uint8_t reg,uint8_t *data){
return HAL_I2C_Mem_Read(dev->i2cHandle,ADXL355_I2C_ADDR, reg,I2C_MEMEADD_SIZE_8BIT, data,1,HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355 *dev, uint8_t reg,uint8_t *data,uint8_t length){
return 	HAL_I2C_Mem_Read(dev->i2cHandle,ADXL355_I2C_ADDR,reg,I2C_MEMEADD_SIZE_8BIT,data,HAL_MAX_DELAY)
}

HAL_StatusTypeDef ADXL355_WriteRegister(ADXL355 *dev,uint_8 reg,uint_8 *data){
return HAL_I2C_Mem_Write(dev->i2cHandle, ADXL355_I2C_ADDR,reg,I2C_MEMEADD_SIZE_8BIT,data,1,HAL_MAX_DELAY);

}
