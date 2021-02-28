/*
 * FLASH_PAGE.h
 *
 *  Created on: 08-May-2020
 *      Author: controllerstech
 */

#ifndef INC_FLASH_PAGE_H_
#define INC_FLASH_PAGE_H_

#include "stm32f1xx_hal.h"

/*
2 different speed
Low  <  1200                                  // fixed advance ignition
High >= First breakpoint(with restriction %)  // according igntion map
%Due the sw conception, the first break point must be >= 1200rpm
Engine Speed: Stopped, Acceleration, Steady State, Decelerate
Engine > Cut_Ignition threshould -> Cut ignition complete in Overspeed
*/
typedef struct
{
    uint8_t  sensorAngDisplecement;
    uint16_t Max_Engine_Speed;
    uint16_t BP_Engine_Speed[12];
    uint8_t  BP_Timing_Advance[12];
    uint8_t  alpha;
    uint8_t  beta;
    uint8_t  gamma;
}dataCalibration;

#define blockSize sizeof (dataCalibration)

typedef union 
{
    dataCalibration Calibration_RAM;
    uint32_t array_Calibration_RAM[blockSize>>2];   //Divided in 4 (32/4 = 8 byte)	  
    uint8_t array_Calibration_RAM_UART[blockSize];
}calibrationBlock;

calibrationBlock calibFlashBlock;

static const calibrationBlock Initial_Calibration = { 28, 7500,
	                                            ////The first Engine Speed value in the array needs to be 1200 or greater than mandatory
                                              { 1300, 2000, 2500, 3000, 3500, 4000, 4500, 7000, 8000, 9000,12000,15000},
                                              //{  64,   64,   64,   64,   64,   64,   64,   64,   64,   64,    64,    64}, 90, 80, 10};
                                              //{  48,   48,   48,   48,   48,   48,   48,   48,   48,   48,    48,    48}, 90, 80, 10};
                                              //{  32,   32,   32,   32,   32,   32,   32,   32,   32,   32,    32,    32}, 90, 80, 10};
                                              //{  16,   16,   16,   16,   16,   16,   16,   16,   16,   16,    16,    16}, 90, 80, 10};
                                              //{   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0}, 90, 80, 10};
                                              //{  64,   54,   44,   39,   36,   32,   32,   36,   40,   45,     55,     64}, 90, 80, 10};
                                              {   64,   58,   48,   38,   25,   15,    0,    0,   40,   45,   55,   64}, 90, 80, 10};
                                              //64 -> 18 degree, calib_table = 64-ang_obj+18 <-> ang_obj = 64-calib_table+	

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t * DATA_32, uint32_t dataSize);
void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32);
void Convert_To_Str (uint32_t *data, char *str);
void saveCalibRamToFlash(void);
void copyCalibFlashToRam(void);



/********************  FLASH_Error_Codes   ***********************//*
HAL_FLASH_ERROR_NONE      0x00U  // No error
HAL_FLASH_ERROR_PROG      0x01U  // Programming error
HAL_FLASH_ERROR_WRP       0x02U  // Write protection error
HAL_FLASH_ERROR_OPTV      0x04U  // Option validity error
*/


#endif /* INC_FLASH_PAGE_H_ */
