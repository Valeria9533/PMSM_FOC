/* MODULE serial. */

#include "serial.h"
#include "usart.h"

#define FREEMASTER_huart huart2

#define hcan hcan1

#define DE
#define DE_set	HAL_GPIO_WritePin(USART2_DIR_GPIO_Port, USART2_DIR_Pin, GPIO_PIN_SET)
#define DE_clr	HAL_GPIO_WritePin(USART2_DIR_GPIO_Port, USART2_DIR_Pin, GPIO_PIN_RESET)

// variable for Freemaster over CAN
#define OVER_UART	0
#define OVER_CAN	1
#define OVER_USB	2

uint8_t FreemasterOver = OVER_UART;

uint8_t DataInFreemasterCAN[256];
uint8_t PointerDataInFreemasterCAN=0;
uint8_t PointerDataInReadFreemasterCAN=0;
uint8_t DataOutFreemasterCAN[256];
uint8_t PointerDataOutFreemasterCAN=0;
uint8_t PointerDataOutWriteFreemasterCAN=0;
uint8_t CanDataOut[8];

uint8_t DataInFreemasterUSB[256];
uint8_t PointerDataInFreemasterUSB=0;
uint8_t PointerDataInReadFreemasterUSB=0;
uint8_t DataOutFreemasterUSB[256];
uint8_t PointerDataOutFreemasterUSB=0;
uint8_t PointerDataOutWriteFreemasterUSB=0;
uint8_t USBDataOut[64];

void FMSTR_SCI_PUTCHAR(uint8_t _data) {	
	switch (FreemasterOver) {
		case OVER_UART:  
			FREEMASTER_huart.Instance->DR=_data;
			break;
		case OVER_CAN: 
			DataOutFreemasterCAN[++PointerDataOutFreemasterCAN]=_data;
			break;					
		case OVER_USB: 
			DataOutFreemasterUSB[++PointerDataOutFreemasterUSB]=_data;
			break;		
		default: break;			
   } 	
}

uint16_t FMSTR_SCI_GETCHAR() {
	switch (FreemasterOver) {
		case OVER_UART:  
			return FREEMASTER_huart.Instance->DR;
		case OVER_CAN: 
			return DataInFreemasterCAN[++PointerDataInReadFreemasterCAN];			
		case OVER_USB: 
		default:			
			return DataInFreemasterUSB[++PointerDataInReadFreemasterUSB];				
   } 	
}

void FMSTR_SCI_RE(void) {
	if (FreemasterOver == OVER_UART) {
		FREEMASTER_huart.Instance->CR1 |= USART_CR1_RE ;
		#ifdef DE
			DE_clr;
		#endif
	}
}

void FMSTR_SCI_RD(void) {
	if (FreemasterOver == OVER_UART) {
		FREEMASTER_huart.Instance->CR1 &= ~USART_CR1_RE ;
	}
}

void FMSTR_SCI_TE(void) {
	if (FreemasterOver == OVER_UART) {
		FREEMASTER_huart.Instance->CR1 |= USART_CR1_TE ;
		#ifdef DE
			DE_set;
		#endif	
	}		
}

void FMSTR_SCI_TD(void) {
	FREEMASTER_huart.Instance->CR1 &= ~USART_CR1_TE ;
}

FMSTR_SCISR FMSTR_SCI_RDCLRSR(void) {
	FMSTR_SCISR SciSR = 0;
		
	// For Freemaster over UART	
	if(__HAL_UART_GET_FLAG(&FREEMASTER_huart, UART_FLAG_RXNE) == SET) // UART receive buffer full 
	{
		SciSR = FMSTR_SCISR_RDRF; 
		FreemasterOver = OVER_UART;
	}
	
	// For Freemaster over CAN
	if (PointerDataInFreemasterCAN != PointerDataInReadFreemasterCAN) {
		SciSR = FMSTR_SCISR_RDRF; 
		FreemasterOver = OVER_CAN;				
	}	
	
	// For Freemaster over USB
	if (PointerDataInFreemasterUSB != PointerDataInReadFreemasterUSB) {
		SciSR = FMSTR_SCISR_RDRF; 
		FreemasterOver = OVER_USB;				
	}			
	
	switch (FreemasterOver) {
		case OVER_UART:  
			if (__HAL_UART_GET_FLAG(&FREEMASTER_huart, UART_FLAG_TC) == SET) {  // UART is transmitting data or transmit register full (UART busy) {   
				SciSR |= FMSTR_SCISR_TDRE; 
			} 
			break;
		case OVER_CAN: 
			if ((PointerDataOutFreemasterCAN - PointerDataOutWriteFreemasterCAN) < 32) {
				SciSR |= FMSTR_SCISR_TDRE;
			}
			break;					
		case OVER_USB: 
			if ((PointerDataOutFreemasterUSB - PointerDataOutWriteFreemasterUSB) < 32) {
				SciSR |= FMSTR_SCISR_TDRE;
			}
			break;					
   }	
	return SciSR;
}


/*
in file usbd_cdc_if.c you must add

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  // USER CODE BEGIN 6 
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	
	CDC_ReceiveCallBack(Buf, *Len);	//!!!!!!!!!!!!!!
	
  return (USBD_OK);
  // USER CODE END 6 
}
*/

void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len) {
int16_t i;	
	for (i = 0; i < Len; i++) {
		DataInFreemasterUSB[++PointerDataInFreemasterUSB] = *(Buf+i);
	}	
}



/* END Serial. */

