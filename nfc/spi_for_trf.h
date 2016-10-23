/*
 * {spi_for_trf.h}
 *
 * {Header File}
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#ifndef _SPI_TRF_H_
#define _SPI_TRF_H_

#include "types.h"
#include "hw_types.h"
//================================================================





//===============================================================


extern void SpiDirectCommand(u08_t *pbuf);
extern void SpiDirectMode(void);
extern void SpiRawWrite(u08_t *pbuf, u08_t length);
extern void SpiReadCont(u08_t *pbuf, u08_t length);
extern void SpiReadSingle(u08_t *pbuf, u08_t number);
extern void SpiWriteCont(u08_t *pbuf, u08_t length);
extern void SpiWriteSingle(u08_t *pbuf, u08_t length);

//===============================================================

//*****************************************************************************
// Values that can be passed to SPIConfigSetExpClk() as ulMode parameter
//*****************************************************************************
#define SPI_MODE_MASTER    0x00000000
#define SPI_MODE_SLAVE     0x00000004

//*****************************************************************************
// Values that can be passed to SPIConfigSetExpClk() as ulSubMode parameter
//*****************************************************************************
#define SPI_SUB_MODE_0     0x00000000
#define SPI_SUB_MODE_1     0x00000001
#define SPI_SUB_MODE_2     0x00000002
#define SPI_SUB_MODE_3     0x00000003


//*****************************************************************************
// Values that can be passed to SPIConfigSetExpClk() as ulConfigFlags parameter
//*****************************************************************************
#define SPI_SW_CTRL_CS      0x01000000
#define SPI_HW_CTRL_CS      0x00000000
#define SPI_3PIN_MODE       0x02000000
#define SPI_4PIN_MODE       0x00000000
#define SPI_TURBO_ON        0x00080000
#define SPI_TURBO_OFF       0x00000000
#define SPI_CS_ACTIVEHIGH   0x00000000
#define SPI_CS_ACTIVELOW    0x00000040
#define SPI_WL_8            0x00000380
#define SPI_WL_16           0x00000780
#define SPI_WL_32           0x00000F80

//*****************************************************************************
// Values that can be passed to SPIFIFOEnable() and SPIFIFODisable()
//*****************************************************************************
#define SPI_TX_FIFO        0x08000000
#define SPI_RX_FIFO        0x10000000

//*****************************************************************************
// Values that can be passed to SPIDMAEnable() and SPIDMADisable()
//*****************************************************************************
#define SPI_RX_DMA         0x00008000
#define SPI_TX_DMA         0x00004000

//*****************************************************************************
// Values that can be passed to SPIIntEnable(), SPIIntDiasble(),
// SPIIntClear() or returned from SPIStatus()
//*****************************************************************************
#define SPI_INT_DMATX         0x20000000
#define SPI_INT_DMARX         0x10000000
#define SPI_INT_EOW           0x00020000
#define SPI_INT_WKS           0x00010000
#define SPI_INT_RX_OVRFLOW    0x00000008
#define SPI_INT_RX_FULL       0x00000004
#define SPI_INT_TX_UDRFLOW    0x00000002
#define SPI_INT_TX_EMPTY      0x00000001

//*****************************************************************************
// Values that can be passed to SPITransfer()
//*****************************************************************************
#define SPI_CS_ENABLE  0x00000001
#define SPI_CS_DISABLE 0x00000002


#endif
