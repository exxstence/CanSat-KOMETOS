#include "altMain.hpp"
#include "main.h"
#include "ICM42688.h"
#include "i2c.h"
#include <stdlib.h>
#include <time.h>


#ifdef __cplusplus
extern "C"
{
#endif
// START C-BLOCK IN C++ CODE

	#ifdef __GNUC__
	/* With GCC, small printf (option LD Linker->Libraries->Small printf
	   set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif /* __GNUC__ */

	/**
	  * @brief  Retargets the C library printf function to the USART.
	  * @param  None
	  * @retval None
	  */
	PUTCHAR_PROTOTYPE
	{
	  /* Place your implementation of fputc here */
	  /* e.g. write a character to the USART1 and Loop until the end of transmission */
	  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

	  return ch;
	}

// END C-BLOCK IN C++ CODE
#ifdef __cplusplus
}
#endif
/*
FATFS fs, FatFs;
FIL file;
FRESULT fresult, FR_Status;
UINT br, bw;
*/
#define BUFFER_SIZE 4096
char buffer[BUFFER_SIZE];
int buffer_index = 0;
/*
void Mount_SD(void) {
	if (f_mount(&fs, "", 1) != FR_OK) {
		// Handle the error
		printf("Fail to mount\n");
	} else {
		printf("Card mounted\n");
	}
}
*/
/*
void Open_File(void) {
	if (f_open(&file, "myfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		// Handle the error
	}
}
*/
/*
void Open_File(void) {
	if (f_open(&file, "myfile.txt", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
		// Handle the error
		//printf("Error to open file\n");
	} else {
		// Move to the end of the file to append data
		if (f_lseek(&file, f_size(&file)) != FR_OK) {
			// Handle the error
			//printf("Error to seek to end of file\n");
		}
	}
}


void Write_File(void) {
	char data[] = "Hello, Hell!";
	if (f_write(&file, data, sizeof(data)-1, &bw) != FR_OK) {
		// Handle the error
		printf("Error to write\n");
	}
	printf("Created file\n");
}


void write_data(const char* data_string) {
	int length = strlen(data_string);
	if (buffer_index + length > BUFFER_SIZE) {
		// Write buffer to file
		if (f_write(&file, buffer, buffer_index, &bw) != FR_OK) {
			// Handle the error
			//printf("Error to write\n");
		}
		buffer_index = 0;
	}
	memcpy(&buffer[buffer_index], data_string, length);
	buffer_index += length;
}


void Close_File(void) {
	if (f_close(&file) != FR_OK) {
		// Handle the error
		//printf("Error to close\n");
	}
	//printf("Closed file\n");
}

void Close_File1() {
	// Flush the buffer before closing the file
	if (buffer_index > 0) {
		if (f_write(&file, buffer, buffer_index, &bw) != FR_OK) {
			// Handle the error
			printf("Error to write\n");
		}
		buffer_index = 0;
	}
	f_close(&file);
}
*/
/*
static void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, "", 1);
    if (FR_Status != FR_OK)
    {
      printf("Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    printf("SD Card Mounted Successfully! \r\n\n");
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    printf("Total SD Card Size: %lu Bytes\r\n", TotalSize);
    printf("Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      printf("Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    printf("Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SDIO, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDIO, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      printf("Error! While Opening (MyTextFile.txt) File For Read.. \r\n");
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    printf("Data Read From (MyTextFile.txt) Using f_gets():%s", RW_Buffer);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    printf("Data Read From (MyTextFile.txt) Using f_read():%s", RW_Buffer);
    // Close The File
    f_close(&Fil);
    printf("File Closed! \r\n\n");
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      printf("Error! While Opening (MyTextFile.txt) File For Update.. \r\n");
      break;
    }
    // (2) Write New Line of Text Data To The File
    //FR_Status = f_puts("This New Line Was Added During File Update!\r\n", &Fil);
    f_puts("This New Line Was Added During File Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    printf("Data Read From (MyTextFile.txt) After Update:\r\n%s", RW_Buffer);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File

    //FR_Status = f_unlink(MyTextFile.txt);
    //if (FR_Status != FR_OK){
        //sprintf(TxBuffer, "Error! While Deleting The (MyTextFile.txt) File.. \r\n");
        //USC_CDC_Print(TxBuffer);
    //}

  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      printf("\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
  } else{
      printf("\r\nSD Card Un-mounted Successfully! \r\n");
  }
}
*/
ICM42688 IMU(hspi1, ICM42688_CS_GPIO_Port, ICM42688_CS_Pin);
volatile bool dataReady = false;

// EXTI Line4 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ICM42688_INT_Pin) // If The INT Source Is EXTI Line4
    {
    	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Toggle The Output (LED) Pin
    	dataReady = true;
    }
}


void generate_random_imu_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
	*ax = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // Random value between -1 and 1
	*ay = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
	*az = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
	*gx = ((float)rand() / RAND_MAX) * 250.0f - 125.0f; // Random value between -125 and 125 degrees/sec
	*gy = ((float)rand() / RAND_MAX) * 250.0f - 125.0f;
	*gz = ((float)rand() / RAND_MAX) * 250.0f - 125.0f;
}


#define RX_BUFFER_SIZE 256
uint8_t rx_buffer1[RX_BUFFER_SIZE];
uint8_t rx_buffer2[RX_BUFFER_SIZE];

char data_string[256];
uint32_t adc_val1 = 65000;
uint32_t adc_val2 = 0;
float temperature = 0;

float ax, ay, az, gx, gy, gz;
float typical_pressure = 1013.0f; // Typical pressure in hPa
float pressure = 0;

uint8_t DevAddress=0x48;
uint8_t data[2];
uint8_t startreg;
uint16_t temp;
float celsius, fahrenheit;

void setup() {
/*
	  // Initialize SD card in 1-bit mode
	  if (HAL_SD_Init(&hsd1) != HAL_OK) {
		  Error_Handler();
	  }

	  FR_Status = f_mount(&FatFs, "", 1);
	  if (FR_Status != FR_OK)
	  {
		  printf("Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
		  //break;
	  } else {
		  printf("SD Card Mounted Successfully! \r\n\n");
	  }

	  HAL_Delay(100);

	  Mount_SD();
*/
/*
	  Open_File();
	  Write_File();
	  Close_File();
*/
/*	SDIO_SDCard_Test(); */

	int status = IMU.begin();
	if (status < 0) {
		printf("IMU initialization unsuccessful\n");
	    printf("Check IMU wiring or try cycling power\n");
	    printf("Status: %d\n", status);
	    while(1) {}
	}

	// set output data rate to 12.5 Hz
	//IMU.setAccelODR(ICM42688::odr12_5);
	//IMU.setGyroODR(ICM42688::odr12_5);
	IMU.setAccelODR(ICM42688::odr200);
	IMU.setGyroODR(ICM42688::odr200);

	// enabling the data ready interrupt
	//IMU.enableDataReadyInterrupt();

	//i2c_scanner(hi2c1);
	I2C_IsDeviceReady(&hi2c1,  DevAddress, 3, 1000);

	// Start DMA reception for both UARTs
	HAL_UART_Receive_DMA(&huart1, rx_buffer1, RX_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // Enable IDLE line interrupt
	HAL_UART_Receive_DMA(&huart3, rx_buffer2, RX_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); // Enable IDLE line interrupt

	//srand(time(NULL)); // Seed the random number generator


}

void loop() {

	while (adc_val1 > (uint32_t) 60000) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc_val1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		printf("Light2 = %lu\n\r", adc_val1);
		HAL_Delay(300);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	}
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

		  HAL_ADC_Start(&hadc3);
		  HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
		  adc_val2 = HAL_ADC_GetValue(&hadc3);
		  HAL_ADC_Stop(&hadc3);
		  temperature = adc_val2*(3.3/65535.0);
		  temperature = temperature - 0.5;
		  temperature = temperature / 0.01;
		  //printf("A_temp= %f\n\r", temperature);
		  HAL_Delay(10);

		  //-----------------------------
		  data[0]=0x00;
		  data[1]=0x00;
		  startreg=0x00;

		  I2C_receive(&hi2c1, DevAddress, startreg, data, 2, 1000);

		  //it's a 12bit int, using two's compliment for negative
		  //so let's say that MSB is 10110110 and LSB is 11010000
		  //So first we move the MSB over to the left 8 bytes doing this: MSB << 8 and it becomes 1011011000000000
		  //The we OR that value with the LSB by doing this: | LSB so it becomes 1011011011010000
		  //The temp value is only 12 bits long, but we have 16 bits. So we need to get rid of the first 4 bits.
		  //And we do it by moving the value right by 4 bits like this: >> 4 so it becomes this 0000101101101101 aka 101101101101
		  temp = (uint16_t)((data[0]<<8 | data[1]) >> 4);	//temperatura yra 12 bit ilgio, bet mes idejome i 16 bitu, todel nurasius 4 bitus gauname tikraja verte

		  //correction if we got negative temperature
		  //The first bit on the MSB is 0 for positive value, and 1 for negative value.
		  //It's a little complicated, the way it works is if that first bit is 1, then the value is negitive, and inveted, so 0b1100 is actually 0b0011.
		  if(temp & 0x100000000000){
		  	temp -= 1;
		  	temp = ~ temp;
		  	temp = temp & 0x111111111111;

		  	celsius = (float)temp * (float)-0.0625;
		  }else{
		  	celsius = (float)temp * (float)0.0625;
		  }

		  celsius = (float)temp *(float)0.0625;	//aptinka 0.0625 Celsijaus pakitimus su 0.5 celsijaus paklaida. 0.0625 atitinka dalyba is 16 -> TEMPF/16
		  celsius = celsius - (float) 7.0;

		  //fahrenheit = ( (float) 1.8 * celsius) + (float) 32;
		  //-----------------------------

		  generate_random_imu_data(&ax, &ay, &az, &gx, &gy, &gz);
		  float variation = ((float)rand() / RAND_MAX) * 10.0f - 5.0f; // Random variation between -5 and 5 hPa
		  pressure = typical_pressure + variation;
		  sprintf(data_string, "$A_Temp: %fC\t D_Temp: %fC\t Pressure: %fhPa\n\r accX %f,accY %f, accZ %f, gyroX %f, gyroY %f, gyroZ %f#\n\r", temperature, celsius, pressure, ax, ay, az, gx, gy, gz);
		  //printf("%s", data_string); // Print the string to verify

		  //if (dataReady) {
			  // read the sensor
			  //IMU.getAGT();
			  //dataReady = false;
			  // display the data
			  //printf("accX %f\n",IMU.accX());
			  //printf("accY %f\n",IMU.accY());
			  //printf("accZ %f\n",IMU.accZ());
			  //printf("gyroX %f\n",IMU.gyrX());
			  //printf("gyroY %f\n",IMU.gyrY());
			  //printf("gyroZ %f\n",IMU.gyrZ());
			  //printf("temp %f\n",IMU.temp());
			  // Now data_string contains the formatted sensor data
			  //HAL_UART_Transmit(&huart3, (uint8_t *)data_string, strlen(data_string), HAL_MAX_DELAY);
		  //}

			  //sprintf(data_string, "$A_Temp: %f\n\r#", temperature);
			  //sprintf(data_string, "$A_Temp: %f\n\r accX %f, gyroX %f\n\r#", temperature, IMU.accX(), IMU.gyrX());
			  //HAL_UART_Transmit(&huart3, (uint8_t *)data_string, strlen(data_string), HAL_MAX_DELAY);
			  //sprintf(data_string, "$A_Temp: %fC\t D_Temp: %fC\t Pressure: %fhPa\n\r accX %f,accY %f, accZ %f, gyroX %f, gyroY %f, gyroZ %f, temp %f\n\r#", temperature, celsius, pressure, IMU.accX(), IMU.accY(), IMU.accZ(), IMU.gyrX(), IMU.gyrY(), IMU.gyrZ(), IMU.temp());
			  //printf("%s", data_string); // Print the string to verify
			  HAL_UART_Transmit(&huart3, (uint8_t *)data_string, strlen(data_string), HAL_MAX_DELAY);
			  //adc_val2 = 0;
			  //temperature = 0;
/*
			  Open_File();
			  if (f_write(&file, data_string, strlen(data_string), &bw) != FR_OK) {
				  // Handle the error
				  printf("Error to write\n");
			  }
			  //write_data(data_string);
			  Close_File();
*/

			  HAL_Delay(500);


}
