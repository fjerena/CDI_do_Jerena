/*
 * SerialComm.h
 *
 *  Created on: 28-Fev-2021
 *      Author: Jerena
 */

#ifndef INC_SERIAL_COMM_H_
#define INC_SERIAL_COMM_H_

//Include
#include "stm32f1xx_hal.h"

//Cont
#define	CALIBBLOCKSIZE	44
#define	DATABLOCKSIZE	  11

//Var
enum Transmission_Status {TRANSMITING, TRANSMISSION_DONE} transmstatus;
enum Reception_Status {DATA_AVAILABLE_RX_BUFFER, RECEPTION_DONE} receptstatus;
uint8_t UART3_txBuffer[CALIBBLOCKSIZE];
uint8_t UART3_rxBuffer[CALIBBLOCKSIZE];
uint8_t UART3_rxBufferAlt[DATABLOCKSIZE];
uint8_t flgTransmition;

//Functions
void initializeCalibOnRAM(void);
void transmitSystemInfo(void);
void copyCalibUartToRam(void);
void copyCalibRamToUart(void);
void receiveData(void);
void transmitCalibToUART(void);

#endif /* INC_SERIAL_COMM_H_ */
