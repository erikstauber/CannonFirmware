/*
 * uart.h
 *
 *  Created on: Jun 26, 2024
 *      Author: eriks
 */

#ifndef INC_UART_H_
#define INC_UART_H_

typedef enum {
	kUartCom,
	kUartCount
} UartType;

extern bool UartBoot();
extern bool UartDMAWrite(UartType uart_type, uint8_t* buffer, uint32_t numbytes);
extern uint32_t UartDMARead(UartType uart_type, uint8_t* buffer, uint32_t maxsize);
extern uint32_t UartGetDMARxBytesAvailable(UartType uart_type);

#endif /* INC_UART_H_ */
