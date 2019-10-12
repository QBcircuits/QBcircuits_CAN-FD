/***************************************************
  This is our library for the QBcircuits Arduino Pro mini/micro CAN FD shield
  ----> https://www.qbcircuits.com

  Check out the link above for our tutorials, wiring diagrams
  QBcircuits invests time and effort providing this open source code,
  please support QBcircuits and open-source hardware by purchasing
  products from QBcircuits - GoLong!

  Written by Brother Q for QBcircuits
  All text above must be included in any redistribution
 ****************************************************/
#ifndef	QB_MCP2517_H
#define	QB_MCP2517_H

#include <inttypes.h>
#include "qb_mcp2517_defs.h"
#include "qb_mcp2517_global.h"
#ifdef __cplusplus

extern "C"
{

#endif

/**************************************************************************************************
Algorithm error codes 
**************************************************************************************************/
#define ERR_TXQFULL		1				// Error Code = TXQ is full
#define ERR_NTXFIFO		2				// Error Code = FIFO not configured as TX FIFO
#define ERR_FIFOFULL	3				// Error Code = FIFO is full
#define ERR_TXQEMPTY	4				// Error Code = TXQ is empty
#define ERR_FIFOEMPTY	5				// Error Code = FIFO is empty

#define ERR_NRXFIFO		6				// Error Code = FIFO not configured as RX FIFO

/**************************************************************************************************
Algorithm variables 
**************************************************************************************************/
#define CSCNT			10				// ~ #of CPU clocks to wait between toggling SPI CS pin(s)
#define IDE     		1				// IDE bit
#define FDF     		1				// FDF bit
#define BRS     		1				// BRS bit
#define RTR     		1				// RTR bit

#define CANSPEED_125 	7				// CAN speed at 125 kbps
#define CANSPEED_250  	3				// CAN speed at 250 kbps
#define CANSPEED_500	1				// CAN speed at 500 kbps

#define FLTRBOTH		0				// input for mcp2517_fltr_setup() to apply filter to both extended (29 bit) & standard (11 bit) CAN FD/CAN 2.0 frames
#define FLTRSID			1				// input for mcp2517_fltr_setup() to apply filter to only standard (11 bit) CAN FD/CAN 2.0 frames
#define FLTREXID		2				// input for mcp2517_fltr_setup() to apply filter to only extended (29 bit) CAN FD/CAN 2.0 frames

#define FLTRNUM0		0				// input for mcp2517_fltr_setup() to setup filter number 0 of the MCP2517
#define FLTRNUM1		1				// input for mcp2517_fltr_setup() to setup filter number 1 of the MCP2517
#define FLTRNUM2		2				// input for mcp2517_fltr_setup() to setup filter number 2 of the MCP2517
#define FLTRNUM3		3				// input for mcp2517_fltr_setup() to setup filter number 3 of the MCP2517
#define FLTRNUM4		4				// input for mcp2517_fltr_setup() to setup filter number 4 of the MCP2517
#define FLTRNUM5		5				// input for mcp2517_fltr_setup() to setup filter number 5 of the MCP2517
#define FLTRNUM6		6				// input for mcp2517_fltr_setup() to setup filter number 6 of the MCP2517
#define FLTRNUM7		7				// input for mcp2517_fltr_setup() to setup filter number 7 of the MCP2517

#define FLTRIDX0		0				// input for mcp2517_fltr_setup() to setup filter index 0 of filters 0-7 of the MCP2517
#define FLTRIDX1		1				// input for mcp2517_fltr_setup() to setup filter index 1 of filters 0-7 of the MCP2517
#define FLTRIDX2		2				// input for mcp2517_fltr_setup() to setup filter index 2 of filters 0-7 of the MCP2517
#define FLTRIDX3		3				// input for mcp2517_fltr_setup() to setup filter index 3 of filters 0-7 of the MCP2517

#define C1FIFOCON(m)		0x050 + (m * 12)		// m=(1-31)
#define C1FIFOSTA(m)		0x054 + (m * 12)		// m=(1-31)
#define C1FIFOUA(m)			0x058 + (m * 12)		// m=(1-31)

#define C1FLTCON(m)			0x1D0 + (m * 4)			// m=(0-7)
#define C1FLTOBJ(m)			0x1F0 + (m * 8)			// m=(0-7)
#define C1MASK(m)			0x1F4 + (m * 8)			// m=(0-7)

/* Inputs for sub-routines with bufIdx* parameters (refer to examples & libraries for usage) ***************************************/
#define TXQ				0
#define FIFO0			0
#define FIFO1			1
#define FIFO2			2
#define FIFO3			3
#define FIFO4			4
#define FIFO5			5
#define FIFO6			6
#define FIFO7			7
#define FIFO8			8
#define FIFO9			9
#define FIFO10			10
#define FIFO11			11
#define FIFO12			12
#define FIFO13			13
#define FIFO14			14
#define FIFO15			15
#define FIFO16			16
#define FIFO17			17
#define FIFO18			18
#define FIFO19			19
#define FIFO20			20
#define FIFO21			21
#define FIFO22			22
#define FIFO23			23
#define FIFO24			24
#define FIFO25			25
#define FIFO26			26
#define FIFO27			27
#define FIFO28			28
#define FIFO29			29
#define FIFO30			30
#define FIFO31			31

typedef struct{
	uint8_t chnNum;
	uint8_t regWr[4];
	uint8_t regRd[4];
	struct {
		// R0/T0 ------------------------
		uint8_t sid07_00;
		
		uint8_t sid10_08 		: 3;
		uint8_t eid04_00 		: 5;
		
		uint8_t eid12_05;
		
		uint8_t eid17_13 		: 5;
		uint8_t sid11 			: 1;
		uint8_t 	 			: 2;
		
		// R1/T1 ------------------------
		uint8_t dlc 			: 4;
		uint8_t ide 			: 1;
		uint8_t rtr 			: 1;
		uint8_t brs 			: 1;
		uint8_t fdf 			: 1;
		
		union {
			struct{
				uint8_t esi 	: 1;
				uint8_t seq 	: 7;
			};
			struct{
				uint8_t 		: 3;
				uint8_t filhit 	: 5;
			};
		};
		
		uint8_t :8;
 		uint8_t :8;
		
		// R2/T2 ------------------------
		union{
			uint8_t txData[64];
			struct{
				uint8_t rxTstamp[4];
				uint8_t rxData[64];
			};
			uint8_t txTstamp[4];
		};
		unsigned long tStamp;
		uint8_t pLen;
	} msg;
} chnCAN;

uint8_t 		spi_putChr(uint8_t data);
uint8_t 		spi_putCmd(uint8_t cmd,uint16_t addr );

void 			mcp2517_cs_clr(uint8_t chnNum);
void 			mcp2517_cs_set(uint8_t chnNum);
void 			mcp2517_read_register(uint16_t addr,chnCAN *ptrChn,uint8_t dataNum);
void 			mcp2517_write_register(uint16_t addr,chnCAN *ptrChn,uint8_t dataNum);

uint8_t 		mcp2517_read_memory(uint8_t bufIdx,chnCAN *ptrChn);
uint8_t 		mcp2517_write_memory(uint8_t bufIdx,chnCAN *ptrChn);
uint8_t 		mcp2517_check_message(chnCAN *ptrChn);
uint8_t 		mcp2517_start_transmit(uint8_t bufIdx,chnCAN *ptrChn);
void 			mcp2517_msg_write(chnCAN *ptrChn,unsigned long id,uint8_t ide,uint8_t fdf,uint8_t brs,uint8_t rtr,uint8_t bufLen,uint8_t *buf_u8);

void 			mcp2517_init_hardware(uint8_t chnNum);
uint8_t 		mcp2517_init(uint8_t speed,chnCAN *ptrChn,uint8_t chnNum,uint8_t bufIdxTx,uint8_t bufIdxRx);
uint8_t 		mcp2517_reg_compr(uint8_t *ptrBuf0,uint8_t *ptrBuf1,uint8_t dataNum);
uint8_t 		mcp2517_fltr_setup(chnCAN *ptrChn,uint8_t bufIdx,uint8_t fltrNum,uint8_t fltrIdx,uint8_t fltrType,unsigned long msgId,unsigned long mskId);


uint8_t 		mcp2517_mem_payload(uint8_t fdf,uint8_t dlc);
uint8_t 		mcp2517_len_payload(uint8_t fdf,uint8_t dlc);
uint8_t 		mcp2517_dlc_payload(uint8_t fdf,uint8_t bufLen);
unsigned long 	mcp2517_id_calc(chnCAN *ptrChn);
void 			mcp2517_tstamp_calc(chnCAN *ptrChn);
void 			mcp2517_reg_prep(chnCAN *ptrChn, uint8_t bitRdWr, uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0);

#ifdef __cplusplus
}
#endif

#endif	// QB_MCP2517_H
