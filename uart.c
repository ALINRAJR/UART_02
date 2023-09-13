

#include "powercontrol.h"
#include "clock.h"
#include "uart.h"


/**
  * @brief     Setting power ON/OFF for UART Peripheral
  * @param[in] ucUARTNum value can be UART_0,UART_1,UART_2,UART_3
  * @param[in] mode value can be either POWERON/ POWEROFF
  * @return    No return value
  **/
void vUARTPowerControl(uint8_t ucUARTNum,uint8_t mode)
{
	switch(ucUARTNum)
	{
	case UART_0:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCUART0;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCUART0);
		break;
	case UART_1:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCUART1;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCUART1);
		break;
	case UART_2:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCUART2;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCUART2);
		break;
	case UART_3:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCUART3;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCUART3);
		break;
	default :
		/* DO NOTHING */
		break;
	}
}

/**
 * @brief     Setting clock for UART Peripheral
 * @param[in] ucUARTNum value can be UART_0,UART_1,UART_2,UART_3
 * @param[in] clockmode value can be ONEFOURTH,SAME,HALF,ONEEIGTH
 * @return    No return value
 **/
void vUARTClockControl(uint8_t ucUARTNum,uint8_t clockmode)
{
	switch(ucUARTNum)
	{
	case UART_0:
		LPC_SC->PCLKSEL0 &= ~(3 << BIT_PCLKSEL0_PCLK_UART0);
		LPC_SC->PCLKSEL0 |= clockmode << BIT_PCLKSEL0_PCLK_UART0;
		break;
	case UART_1:
		LPC_SC->PCLKSEL0 &= ~(3 << BIT_PCLKSEL0_PCLK_UART1);
		LPC_SC->PCLKSEL0 |= clockmode << BIT_PCLKSEL0_PCLK_UART1;
		break;
	case UART_2:
		LPC_SC->PCLKSEL1 &= ~(3 << BIT_PCLKSEL1_PCLK_UART2);
		LPC_SC->PCLKSEL1 |= clockmode << BIT_PCLKSEL1_PCLK_UART2;
		break;
	case UART_3:
		LPC_SC->PCLKSEL1 &= ~(3 << BIT_PCLKSEL1_PCLK_UART2);
		LPC_SC->PCLKSEL1 |= clockmode << BIT_PCLKSEL1_PCLK_UART2;
		break;
	default :
		/* DO NOTHING */
		break;
	}
}

/**
  * @brief     Enabling access to registers DLL and DLM for setting the baud rate for UART Peripheral
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] mode value can be either ACCESS_ENABLE or ACCESS_DISABLE
  * @return    No return value
  **/
void vUARTDivisorLatchEnable(LPC_UART_TypeDef* UARTx,uint8_t mode)
{
	if(mode==ACCESS_ENABLE)
		UARTx->LCR |= 1<<BIT_LCR_DLAB | 3<<BIT_LCR_WLS;
	else if(mode ==ACCESS_DISABLE)
		UARTx->LCR &= ~(1<<BIT_LCR_DLAB);
}

/**
  * @brief     Enabling FIFO for UART Peripheral
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] baudrate value can BAUDRATE_9600,BAUDRATE_38400,BAUDRATE_57600,BAUDRATE_115200
  * @return    No return value
  **/
void vUARTBaudrateSetup(LPC_UART_TypeDef* UARTx,uint8_t baudrate)
{
    switch(baudrate)
    {
		case BAUDRATE_9600:
			UARTx->DLL = BAUDRATE_9600_DLL;
			UARTx->DLM = BAUDRATE_9600_DLM;
			UARTx->FDR = BAUDRATE_9600_MULVAL <<BIT_FDR_MULVAL| BAUDRATE_9600_DIVVAL<<BIT_FDR_DIVADDVAL;
			break;

		case BAUDRATE_38400:
			UARTx->DLL = BAUDRATE_38400_DLL;
			UARTx->DLM = BAUDRATE_38400_DLM;
			UARTx->FDR = BAUDRATE_38400_MULVAL << BIT_FDR_MULVAL | BAUDRATE_38400_DIVVAL << BIT_FDR_DIVADDVAL;

			break;
		case BAUDRATE_57600:
			UARTx->DLL = BAUDRATE_57600_DLL;
			UARTx->DLM = BAUDRATE_57600_DLM;
			UARTx->FDR = BAUDRATE_57600_MULVAL << BIT_FDR_MULVAL | BAUDRATE_57600_DIVVAL << BIT_FDR_DIVADDVAL;
			break;

		case BAUDRATE_115200:
			UARTx->DLL = BAUDRATE_115200_DLL;
			UARTx->DLM = BAUDRATE_115200_DLM;
			UARTx->FDR = BAUDRATE_115200_MULVAL << BIT_FDR_MULVAL | BAUDRATE_115200_DIVVAL << BIT_FDR_DIVADDVAL;
				break;

		default:
			/* DO NOTHING */
			break;
    }
}


/**
  * @brief     Enabling FIFO for UART Peripheral
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] mode value can be either FIFO_ENABLE or FIFO_DISABLE
  * @return    No return value
  **/
void vUARTFIFOEnable(LPC_UART_TypeDef* UARTx,uint8_t mode)
{
	if(mode==FIFO_ENABLE)
		UARTx->FCR |= 1<<BIT_FCR_FIFO;
	else if(mode ==FIFO_DISABLE)
		UARTx->FCR &= ~(1<<BIT_FCR_FIFO);
}
/**
  * @brief     Initialize the selected UART Peripheral
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ucUARTNum value can be UART_0,UART_1,UART_2,UART_3
  * @param[in] baudrate value can BAUDRATE_9600,BAUDRATE_38400,BAUDRATE_57600,BAUDRATE_115200
  * @return    No return value
  **/
void vUARTInitialize(LPC_UART_TypeDef* UARTx,uint8_t ucUARTNum,uint8_t baudrate)
{
	/* Step 1: Powering the UART0 Peripheral */
	vUARTPowerControl(ucUARTNum,POWERON);

	/* Step 2: Clock select for UART0, here CCLK is 100MHz so PCLK going to use is ONEFOURTH, i.e 25MHz*/
	vUARTClockControl(ucUARTNum,ONEFOURTH);

	/* Step3 : Baud rate setup here setting Baud rate as 9600bps */
	vUARTDivisorLatchEnable(UARTx,ACCESS_ENABLE);
	vUARTBaudrateSetup(UARTx,baudrate);
	vUARTDivisorLatchEnable(UARTx,ACCESS_DISABLE);

	/* Step4: Enabling the Tx and Rx FIFO */
	vUARTFIFOEnable(UARTx,FIFO_ENABLE);

	/* Step5: Selected the pins for the selected UART  */
	  /* Already done inside vPinmuxInitialize */

	/* Step 6: Enabling Interrupt for ADC Peripheral */
		/* Polling method we don't need this feature */

	/* Step 7: Configuring GPDMA  */
		/* Not used in this program */
}

/**
  * @brief     Function to transmit characters in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] bytetosend value 8bit value to transmit over UART
  * @return    No return value
  **/
void vUARTPutCharBlocking(LPC_UART_TypeDef* UARTx,char bytetosend)
{
	/* Wait until the Transmit hold buffer is empty */
	while(!(UARTx->LSR & 1<<BIT_LSR_THRE));
	/* Placing the data to send in the THR register */
	UARTx->THR = bytetosend;
}

/**
  * @brief     Function to transmit characters in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ptrtoarrayofchars address of the array of characters to be send
  * @return    No return value
  **/
void vUARTPutStringBlocking(LPC_UART_TypeDef* UARTx,char* ptrtoarrayofchars)
{
	/* TODO Section */

}

/**
 * @brief     UART put number Function
 * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
 * @param[in] ucNum number to be written to the UART
 * @param[in] base base of the number to be written to the selected UART - {BINARY,OCTAL,DECIMAL,HEXADECIMAL};
 * @return    No return value
 **/
void vUARTPutNumBlocking(LPC_UART_TypeDef* UARTx, uint8_t ucNum,uint8_t base)
{

	/* TODO Section */


}

/**
 * @brief     Printf implementation on UART
 * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
 * @param[in] formatstring string which contains format specifiers
 * @return    No return value
 **/

void vUARTPrintfBlocking(LPC_UART_TypeDef* UARTx,char *formatstring,...)
{
	/* TODO Section */

}

/**
  * @brief     Function to receive character in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ptrtoreceivebyte address of the buffer to read the received value
  * @return    No return value
  **/
void vUARTGetCharBlocking(LPC_UART_TypeDef* UARTx,char* ptrtoreceivebyte)
{
	/* Wait until the data receive */
	while(!(UARTx->LSR & 1<<BIT_LSR_RDR));
	/* Getting the data from the RBR register */
	*ptrtoreceivebyte=UARTx->RBR;
}





