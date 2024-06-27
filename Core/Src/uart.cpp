/*
 * uart.cpp
 *
 *  Created on: Jun 26, 2024
 *      Author: eriks
 */


#include "main.h"
#include "uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>


// uart buffers

static uint8_t rxbuffers_[kUartCount][4096];
static uint8_t txbuffers_[kUartCount][4096];
//-----------------------------------------------------------COM--------------WIFICOMMAND---------COMMUNICATION---Tool
static uint32_t dma_tx_out_ptr_[kUartCount] =				{0};
static uint32_t dma_tx_in_ptr_[kUartCount] = 				{0};
static uint32_t dma_read_pos_[kUartCount] = 				{0};
static UART_HandleTypeDef* uart_handles[kUartCount] = 		{&huart1};
static DMA_HandleTypeDef* dma_rx_handles[kUartCount] = 		{&hdma_usart1_rx};

bool UartBoot() {
	for(int i=0;i<kUartCount;i++) {
		HAL_UART_Receive_DMA(uart_handles[i], (uint8_t*) rxbuffers_[i], sizeof(rxbuffers_)[i]);
	}

	for(int i=0;i<kUartCount;i++) {		// clear out rxbuffers
		uint8_t buffer[1];
		while(UartDMARead(static_cast<UartType>(i),buffer,sizeof(buffer))>0);
	}
	return true;
}

bool UartDMANextSendBuffer(UartType uart_type) {
	if(dma_tx_out_ptr_[uart_type] > dma_tx_in_ptr_[uart_type]) {
		HAL_UART_Transmit_DMA(uart_handles[uart_type],(uint8_t*)txbuffers_[uart_type]+dma_tx_out_ptr_[uart_type], (uint16_t)(sizeof(txbuffers_)[uart_type]-dma_tx_out_ptr_[uart_type]));
	}
	else if (dma_tx_out_ptr_[uart_type] < dma_tx_in_ptr_[uart_type]){
		HAL_UART_Transmit_DMA(uart_handles[uart_type],(uint8_t*)txbuffers_[uart_type]+dma_tx_out_ptr_[uart_type], (uint16_t)(dma_tx_in_ptr_[uart_type]-dma_tx_out_ptr_[uart_type]));
	}

	return true;
}

bool UartDMAWrite(UartType uart_type, uint8_t* buffer, uint32_t numbytes) {
	assert_param(uart_type<kUartCount);
	uint32_t bufsize = sizeof(txbuffers_)[uart_type];
	if(numbytes>bufsize) numbytes=bufsize;
	if(numbytes==0) return false;
	uint32_t remaining_bytes = bufsize - dma_tx_in_ptr_[uart_type];
	if(remaining_bytes>numbytes) {
		memcpy(&txbuffers_[uart_type][dma_tx_in_ptr_[uart_type]],buffer,(size_t)numbytes);
	}
	else {
		memcpy(&txbuffers_[uart_type][dma_tx_in_ptr_[uart_type]],buffer,remaining_bytes);
		memcpy(txbuffers_[uart_type],(buffer+remaining_bytes),(numbytes-remaining_bytes));
	}
	dma_tx_in_ptr_[uart_type]+=numbytes;
	if(dma_tx_in_ptr_[uart_type]>=bufsize) dma_tx_in_ptr_[uart_type] -= bufsize;
	UartDMANextSendBuffer(uart_type);
	return true;
}

uint32_t UartGetDMARxBytesAvailable(UartType uart_type) {
	assert_param(uart_type<kUartCount);
	uint32_t bufsize = sizeof(rxbuffers_)[uart_type];
	uint32_t readpos = bufsize - __HAL_DMA_GET_COUNTER(dma_rx_handles[uart_type]);
	if(dma_read_pos_[uart_type]<=readpos) return readpos - dma_read_pos_[uart_type];
	return bufsize + readpos - dma_read_pos_[uart_type];
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	for(int i=0;i<kUartCount;i++) {
		if (huart->Instance == uart_handles[i]->Instance) {
			dma_tx_out_ptr_[i] += huart->TxXferSize;
			if(dma_tx_out_ptr_[i]>=sizeof(txbuffers_)[i]) dma_tx_out_ptr_[i] -= sizeof(txbuffers_)[i];
			UartDMANextSendBuffer(static_cast<UartType>(i));
			break;
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	for(int i=0;i<kUartCount;i++) {
		if (huart->Instance == uart_handles[i]->Instance) {
			HAL_UART_Receive_DMA(uart_handles[i], (uint8_t*) rxbuffers_[i], sizeof(rxbuffers_)[i]);
			break;
		}
	}
}

uint32_t UartDMARead(UartType uart_type, uint8_t *buffer, uint32_t maxsize) {
	assert_param(uart_type<kUartCount);
	uint32_t bufsize = sizeof(rxbuffers_)[uart_type];
	uint32_t readbytes=0;
	uint32_t dma_counter_pos = __HAL_DMA_GET_COUNTER(dma_rx_handles[uart_type]);
	while(dma_read_pos_[uart_type] != (bufsize - dma_counter_pos)) {
		if(readbytes >= maxsize) break;
		buffer[readbytes] = rxbuffers_[uart_type][dma_read_pos_[uart_type]];
		dma_read_pos_[uart_type]++;
		if(dma_read_pos_[uart_type] >= bufsize) dma_read_pos_[uart_type] = 0;
		readbytes++;
	}
	return readbytes;
}

