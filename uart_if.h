/*
 * uart_if.h
 *
 *  Created on: Oct 22, 2016
 *      Author: shuh
 */

#ifndef UART_IF_H_
#define UART_IF_H_


/****************************************************************************/
/*								MACROS										*/
/****************************************************************************/
#define UART_BAUD_RATE  115200
#define SYSCLK          80000000
#define CONSOLE         UARTA0_BASE
#define CONSOLE_PERIPH  PRCM_UARTA0
//
// Define the size of UART IF buffer for RX
//
#define UART_IF_BUFFER           64

//
// Define the UART IF buffer
//
extern unsigned char g_ucUARTBuffer[];


/****************************************************************************/
/*								FUNCTION PROTOTYPES							*/
/****************************************************************************/
extern void DispatcherUARTConfigure(void);
extern void DispatcherUartSendPacket(unsigned char *inBuff, unsigned short usLength);
extern int GetCmd(char *pcBuffer, unsigned int uiBufLen);
extern void InitTerm(void);
extern void ClearTerm(void);
extern void Message(char *format);
extern void Error(char *format,...);
extern int Report(char *format, ...);
void UartPutChar(unsigned char ch);
void UartPutCrlf(void);
extern void UartSendCString(char *format);
void UartPutByte(unsigned char ch);
void UartPutByteHex(unsigned char ch);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************


#endif /* UART_IF_H_ */
