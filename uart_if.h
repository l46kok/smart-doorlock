/*
 * uart_if.h
 *
 *  Created on: 2016. 8. 27.
 *      Author: Sokwhan
 */

#ifndef __uart_if_H__
#define __uart_if_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

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
extern void Message(const char *format);
extern void Error(char *format,...);
extern int TrimSpace(char * pcInput);
extern int Report(const char *format, ...);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
