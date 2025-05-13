#include "i2c.h"

/**
 * @brief Checking if device is ready
 */
uint8_t I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint32_t TestCount, uint32_t timeout){

	HAL_Delay(1);
	if (HAL_I2C_IsDeviceReady(hi2c, DevAddress<<1, 3, 5) != HAL_OK) {
		 printf("\nDevice not found\n");
		 Error_Handler();
		 return HAL_ERROR;
	}
	else {
		printf("\nDevice found 0x%x\n", DevAddress);
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		return HAL_OK;
	}
}

/**
 * @brief Writing I2C data
 */
void I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* data, uint16_t Size, uint32_t Timeout){

	/* -> Start the transmission process */
	/* While the I2C in reception process, user can transmit data through "data" buffer */

	while (HAL_I2C_Master_Transmit(hi2c, DevAddress<<1, data, Size, Timeout)!= HAL_OK) {
	//while (HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress<<1, data, Size)!= HAL_OK) {
		/*
		 * Error_Handler() function is called when Timeout error occurs.
		 * When Acknowledge failure occurs (Slave don't acknowledge it's address)
		 * Master restarts communication
		 */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			//if (DEBUG_EEPROM == 1) printf("In I2C::WriteBuffer -> error");
			Error_Handler();
		}
	}
    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY){
	}

}

/**
 * @brief Reading I2C data
 */
void I2C_receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t startreg, uint8_t *data, uint16_t Size, uint32_t Timeout){

	I2C_Write(hi2c, DevAddress, &startreg, 1, Timeout);
	/* -> Put I2C peripheral in reception process */

	while (HAL_I2C_Master_Receive(hi2c, DevAddress<<1, data, Size, Timeout)!= HAL_OK) {
	//while (HAL_I2C_Master_Receive_DMA(hi2c, DevAddress<<1, data, Size)!= HAL_OK) {
		/* Error_Handler() function is called when Timeout error occurs.
		 * When Acknowledge failure occurs (Slave don't acknowledge it's address)
		 * Master restarts communication
		 */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			//if (DEBUG_EEPROM == 1) printf("In I2C::WriteBuffer -> error");
			Error_Handler();
		}
	}
	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	 * state of the peripheral; if it’s busy you need to wait for the end of current
	 * transfer before starting a new one.
	 * For simplicity reasons, this example is just waiting till the end of the
	 * transfer, but application may perform other tasks while transfer operation
	 * is ongoing.
	 **/
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
	}
}

void i2c_scanner(I2C_HandleTypeDef _i2c){
	int j = 0;
	uint8_t ret = 0;
    //[ I2C Bus Scanning ]
	printf("Starting I2C Scanning: \r\n");
    for(j=0; j<128; j++)
    {
        ret = HAL_I2C_IsDeviceReady(&_i2c, (uint16_t)(j<<1), 9, 50);
        if (ret != HAL_OK) // No ACK Received At That Address
        {
      	  printf(" - ");
        }
        else if(ret == HAL_OK)
        {
      	  printf("0x%x\n", j);
        }
    }
    printf("Done! \r\n\r\n");
    //[ Scanning Done ]
}
