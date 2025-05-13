#ifndef INC_ALTMAIN_HPP_
#define INC_ALTMAIN_HPP_

#ifdef __cplusplus
extern "C"
{
#endif
// START C-BLOCK IN C++ CODE

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

#include "stdio.h"
#include "string.h"

extern SD_HandleTypeDef hsd1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

/* Functions -----------------------------------------------------------------*/
void setup();
void loop();
// YOUR FUNCTIONS


// END C-BLOCK IN C++ CODE
#ifdef __cplusplus
}
#endif


#endif /* INC_ALTMAIN_HPP_ */
