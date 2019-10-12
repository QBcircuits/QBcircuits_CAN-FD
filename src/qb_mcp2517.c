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

#include <avr/io.h>
#include <util/delay.h>

#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <stdint.h>
#include <avr/pgmspace.h>

#include "qb_mcp2517_global.h"
#include "qb_mcp2517.h"
#include "qb_mcp2517_defs.h"
#include "qb_mcp2517_defaults.h"

/**************************************************************************************************
Purpose: 	Writes 1 byte to the SPI line
Inputs:		data 	- 8 bit unsigned data
Outputs:	SPDR	- SPI data register contents
**************************************************************************************************/
uint8_t spi_putChr( uint8_t data ){
	SPDR = data;													// put byte in send-buffer
	while( !( SPSR & (1<<SPIF) ));									// wait until byte is sent
	return SPDR;													// return byte recieved in SPI Tx/Rx register
}
/**************************************************************************************************
Purpose: 	Writes 2 bytes to the SPI line
Inputs:		cmd 	- MCP2517 4 bit command (values in MCP2517_defs.h)
			addr 	- MCP2517 12 bit register address (values in MCP2517_defs.h)
Outputs:	SPDR	- SPI data register contents
**************************************************************************************************/
uint8_t spi_putCmd(uint8_t cmd, uint16_t addr ){
	uint8_t dummy;
	
	SPDR = (cmd<<4)|(addr>>8);										// put byte in send-buffer (MCP2517FD manual Table 4-1)
	while( !( SPSR & (1<<SPIF) ) );									// wait until byte is sent
	dummy = SPDR;													// read SPI RD/WR register to reset SPIF
	
	SPDR = addr & 0xFF;												// put byte in send-buffer (MCP2517FD manual Table 4-1)
	while( !( SPSR & (1<<SPIF) ) );									// wait until byte is sent	
	
	return SPDR;													// return byte recieved in SPI Tx/Rx register
}
/**************************************************************************************************
Purpose: 	Drive CS pin low for coresponding channel num
Inputs:		chnNum	- channel #
					  <= 1 	= Drives channel 1 CS low
					  > 1 	= Drives channel 2 CS low
Outputs:	None
**************************************************************************************************/
void mcp2517_cs_clr(uint8_t chnNum){
	if(chnNum <= 1)											// check if using channel 1
		RESET(MCP2517_CS1);									// drive channel 1 chip select low
	else 													// default to using channel 2
		RESET(MCP2517_CS2);
}
/**************************************************************************************************
Purpose: 	Drive CS pin high for coresponding channel num
Inputs:		chnNum	- channel #
					  <= 1 	= Drives channel 1 CS high
					  > 1 	= Drives channel 2 CS high
Outputs:	None
**************************************************************************************************/
void mcp2517_cs_set(uint8_t chnNum){
	if(chnNum <= 1)											// check if using channel 1
		SET(MCP2517_CS1);									// drive channel 1 chip select low
	else 													// default to using channel 2
		SET(MCP2517_CS2);
}
/**************************************************************************************************
Purpose: 	Reads 1-4 bytes from the SPI line (Writes 4bit Cmd, 12bit Addr, then reads 8-32bit of register data)
				Register read data is stored in ptrChn->regRd[] member
				regRd[3] = register bits 31:24
				regRd[2] = register bits 23:16
				regRd[1] = register bits 15:08
				regRd[0] = register bits 07:00
Inputs:		addr 	- MCP2517 12 bit address of lowest byte of 32bit register (values in MCP2517_defs.h)
			*ptrChn	- chnCAN pointer
			dataNum - data byte(s) to read into regBuf member in chnCAN variable
					  0 = Read & store register byte 0 (LSB)
					  1 = Read & store register byte 1
					  2 = Read & store register byte 2
					  3 = Read & store register byte 3 (MSB)
					  4 = Read & store all register bytes
Outputs:	None
**************************************************************************************************/
void mcp2517_read_register(uint16_t addr,chnCAN *ptrChn,uint8_t dataNum){
	uint8_t idx;													// used to step thru data buffer
	uint8_t len;													// used to hold #of bytes to read from SPI line
	
	for(idx=0;idx<4;idx++){											// loop thru buffer
		ptrChn->regRd[idx] = 0;										// clear register buffer data bytes
	}
	len = (dataNum > 4) ? 4 : dataNum;								// Cap data length or set to data # provided	
	mcp2517_cs_clr(ptrChn->chnNum);									// drive chn x chip select low (chip enable)
	
	if (len < 4){													// Read a specific byte in register
		spi_putCmd(SPI_READ,addr+len);								// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
		idx = len;													// update the index to byte of intereste
		ptrChn->regRd[idx] = spi_putChr(0xFF);						// clock out the register buffer data bytes	
	}
	else{															// Read all 4 data bytes in register
		spi_putCmd(SPI_READ,addr);									// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
		for(idx=0;idx<len;idx++){									// loop thru buffer
			ptrChn->regRd[idx] = spi_putChr(0xFF);					// clock out the register buffer data bytes
		}
	}
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)
}
/**************************************************************************************************
Purpose: 	Writes 3-6 bytes to the SPI line (4bit Cmd, 12bit Addr, 8-32bit of register data)
				Register write data is stored in ptrChn->regWr[] member
				regWr[3] = register bits 31:24
				regWr[2] = register bits 23:16
				regWr[1] = register bits 15:08
				regWr[0] = register bits 07:00
Inputs:		addr 	- MCP2517 12 bit address of lowest byte of 32bit register (values in MCP2517_defs.h)
			*ptrChn	- chnCAN pointer
			dataNum - data byte(s) to read into regBuf member in chnCAN variable
					  0 = Write to register byte 0 (LSB)
					  1 = Write to register byte 1
					  2 = Write to register byte 2
					  3 = Write to register byte 3 (MSB)
					  4 = Write to all register bytes
Outputs:	None
**************************************************************************************************/
void mcp2517_write_register(uint16_t addr,chnCAN *ptrChn,uint8_t dataNum){
	uint8_t idx;													// used to step thru data buffer
	uint8_t len;													// used to hold #of bytes to write to SPI line
	
	len = (dataNum > 4) ? 4 : dataNum;								// Cap data length or set to data # provided
	mcp2517_cs_clr(ptrChn->chnNum);									// drive chn x chip select low (chip enable)
	if (len < 4){													// Write a specific byte in register
		spi_putCmd(SPI_WRITE,addr+len);								// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
		idx = len;													// update the index to byte of intereste
		spi_putChr(ptrChn->regWr[idx]);								// clock out the register buffer data bytes	
	}
	else{															// Read all 4 data bytes in register
		spi_putCmd(SPI_WRITE,addr);									// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
		for(idx=0;idx<len;idx++){									// loop thru buffer
			spi_putChr(ptrChn->regWr[idx]);							// clock out the register buffer data bytes
		}
	}
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)
}
/**************************************************************************************************
Purpose: 	Reads message object from RX FIFO message object memory
Inputs:		bufIdx 	- selects which FIFO memory to write
			*ptrChn	- chnCAN pointer
Outputs:	result	- error code (defined in qb_mcp2517.h)
					ERR_NRXFIFO		= FIFO not configured as RX FIFO
					ERR_FIFOEMPTY 	= FIFO is empty
**************************************************************************************************/
uint8_t mcp2517_read_memory(uint8_t bufIdx,chnCAN *ptrChn){
	uint8_t idx;													// used to step thru data buffer
	uint8_t len;													// used to hold #of bytes to write to SPI line
	uint8_t *ptr_u8;												// used to point to step thru byte members chnCAN object pointed to by *ptrChn
	uint8_t bufNum;													// used to hold the calculated buffer reference
	uint8_t tStamp;
	uint16_t memAddr;												// used to hold the calculated memeroy address
	uint8_t	temp[4];												// temporary storage
	
	bufNum = (bufIdx > 31) ? 31 : (bufIdx + !bufIdx);				// calculate RX buffer number 1 to 31=FIFO1 to FIFO31

	mcp2517_read_register(C1FIFOCON(bufNum),ptrChn,4);				// read in contents of FIFO config register
	if((ptrChn->regRd[0]>>TXEN) & 1)								// check if FIFO is a TX FIFO
		return ERR_NTXFIFO;											// return error code
	
	tStamp = (ptrChn->regRd[0]>>RXTSEN) & 1;						// Save RX timestamp enable bit
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(C1FIFOSTA(bufNum),ptrChn,4);				// read in contents of FIFO status register
	if(!((ptrChn->regRd[0]>>TFNRFNIF) & 1))							// check if FIFO is full
		return ERR_FIFOEMPTY;										// return error code
		
	mcp2517_read_register(C1FIFOUA(bufNum),ptrChn,4);				// read in contents of user address register

	memAddr = ptrChn->regRd[1];										// set upper byte of buffer memory address
	memAddr = ((memAddr << 8) | ptrChn->regRd[0]) + 0x400;			// finalize the buffer memory address
	ptr_u8 = &ptrChn->msg.sid07_00;									// point to the 1st byte of a message object
	
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_cs_clr(ptrChn->chnNum);									// drive chn x chip select low (chip enable)
	spi_putCmd(SPI_READ,memAddr);									// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
	for(idx=0;idx<8;idx++){											// loop thru 1st 8 bytes of RX msg object
		*(ptr_u8 + idx) = spi_putChr(0xFF);							// clock out dummy bytes to read in RX msg object bytes
	}
	
	if(tStamp){														// FIFO configured to store RX msg object timestamp
		ptr_u8 = &ptrChn->msg.rxTstamp[0];							// point to the 0th byte of timestamp buffer
		len = 4;													// 4 bytes of the timestamp of RX msg obj
	}
	else{															// FIFO not configured to store RX msg object timestamp
		ptr_u8 = &ptrChn->msg.rxData[0];							// point to the 0th byte of RX buffer
		len = 0;													// 0 bytes of the timestamp of RX msg obj
	}
	len += mcp2517_mem_payload(ptrChn->msg.fdf,ptrChn->msg.dlc);	// calculate the total length of the message object in bytes
	for(idx=0;idx<len;idx++){										// loop thru 1st 8 bytes of RX msg object
		*(ptr_u8 + idx) = spi_putChr(0xFF);							// clock out dummy bytes to read in RX msg object bytes
	}
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)
	
	// Increment head of FIFO
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	ptrChn->regWr[1] = 0x01;										// FRESET=TXREQ=0;UINC=1
	mcp2517_write_register(C1FIFOCON(bufNum),ptrChn,1);				// read register data bytes 

	mcp2517_tstamp_calc(ptrChn);									// update the timestamp for the message
	temp[0] = ptrChn->msg.fdf;										// store FDF value
	temp[1] = ptrChn->msg.dlc;										// store DLC value
	ptrChn->msg.pLen = mcp2517_len_payload(temp[0],temp[1]);		// calculate the pLen
	
	return 0;
}
/**************************************************************************************************
Purpose: 	Writes message object to either TXQ or TX FIFO
Inputs:		bufIdx 	- selects which TX buffer memory to write
			*ptrChn	- chnCAN pointer
Outputs:	result	- error code (defined in qb_mcp2517.h)
					ERR_TXQFULL 	= TXQ is full
					ERR_NTXFIFO 	= FIFO not configured as TX FIFO
					ERR_FIFOFULL	= FIFO is full
**************************************************************************************************/
uint8_t mcp2517_write_memory(uint8_t bufIdx,chnCAN *ptrChn){
	uint8_t idx;													// used to step thru data buffer
	uint8_t len;													// used to hold #of bytes to write to SPI line
	uint8_t *ptr_u8;												// used to point to step thru byte members chnCAN object pointed to by *ptrChn
	uint8_t bufNum;													// used to hold the calculated buffer reference
	uint16_t memAddr;												// used to hold the calculated memeroy address
	
	bufNum = (bufIdx > 31) ? 31 : bufIdx;							// calculate TX buffer number 0=TXQ;1 to 31=FIFO1 to FIFO31	
	if(!bufNum){													// working with TXQ buffer
		mcp2517_read_register(ADDR_C1TXQSTA,ptrChn,4);				// read in contents of TXQ status register
		if(!((ptrChn->regRd[0]>>TXQNIF) & 1))						// check if TXQ buffer is full
			return ERR_TXQFULL;										// return error code
			
		mcp2517_read_register(ADDR_C1TXQUA,ptrChn,4);				// read in contents of TXQ user address register
	}
	else{															// working with TX FIFO
		mcp2517_read_register(C1FIFOCON(bufNum),ptrChn,4);			// read in contents of FIFO config register
		if(!((ptrChn->regRd[0]>>TXEN) & 1))							// check if FIFO is a TX FIFO
			return ERR_NTXFIFO;										// return error code
		
		for(idx=0;idx<CSCNT;idx++);									// delay for toggling CS
		mcp2517_read_register(C1FIFOSTA(bufNum),ptrChn,4);			// read in contents of FIFO status register
		if(!((ptrChn->regRd[0]>>TFNRFNIF) & 1))						// check if FIFO is full
			return ERR_FIFOFULL;									// return error code
			
		mcp2517_read_register(C1FIFOUA(bufNum),ptrChn,4);			// read in contents of user address register
	}
	memAddr = ptrChn->regRd[1];										// set upper byte of buffer memory address
	memAddr = ((memAddr << 8) | ptrChn->regRd[0]) + 0x400;			// finalize the buffer memory address
	len = 8 + mcp2517_mem_payload(ptrChn->msg.fdf,ptrChn->msg.dlc);	// calculate the total length of the message object in bytes
	ptr_u8 = &ptrChn->msg.sid07_00;										// point to the 1st byte of a message object
	
	mcp2517_cs_clr(ptrChn->chnNum);									// drive chn x chip select low (chip enable)
	spi_putCmd(SPI_WRITE,memAddr);									// clock out SPI MCP2517FD 4 bit cmd & 12 bit addr
	for(idx=0;idx<len;idx++){										// loop thru buffer
		spi_putChr(*(ptr_u8 + idx));									// clock out the register msg object bytes
	}
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)
	
	// Increment head of TXQ or FIFO
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	ptrChn->regWr[1] = 0x01;										// FRESET=TXREQ=0;UINC=1
	if(!bufNum)														// working with TXQ buffer
		mcp2517_write_register(ADDR_C1TXQCON,ptrChn,1);				// write register data byte 1 
	else															// working with TX FIFO
		mcp2517_write_register(C1FIFOCON(bufNum),ptrChn,1);			// read register data bytes 

	return 0;
}
/**************************************************************************************************
Purpose: 	Checks if MCP2517 interrupt pin is active
Inputs:		*ptrChn	- chnCAN pointer
						
Outputs:	result	- status of the interrupt pin of channel pointed to by ptrChn
					0 = channel interrupt pin not active
					1 = channel interrupt pin active
**************************************************************************************************/
uint8_t mcp2517_check_message(chnCAN *ptrChn) {
	if(ptrChn->chnNum <= 1)
		return (!IS_SET(MCP2517_INT1));
	else
		return (!IS_SET(MCP2517_INT2));
}
/**************************************************************************************************
Purpose: 	Requests message(s) to be transmit from in TXQ or TX FIFO
Inputs:		bufIdx 	- selects which TX buffer memory to write
			*ptrChn	- chnCAN pointer
Outputs:	result	- error code (defined in qb_mcp2517.h)
					ERR_NTXFIFO 	= FIFO not configured as TX FIFO
					ERR_TXQEMPTY 	= TXQ is empty
					ERR_FIFOEMPTY 	= FIFO is empty	
**************************************************************************************************/
uint8_t mcp2517_start_transmit(uint8_t bufIdx,chnCAN *ptrChn){
	uint8_t idx;													// used to step thru data buffer
	uint8_t bufNum;													// used to hold the calculated buffer reference
	
	bufNum = (bufIdx > 31) ? 31 : bufIdx;							// calculate TX buffer number 0=TXQ;1 to 31=FIFO1 to FIFO31
	if(!bufNum){													// working with TXQ buffer
		mcp2517_read_register(ADDR_C1TXQSTA,ptrChn,4);				// read in contents of TXQ status register
		if((ptrChn->regRd[0]>>TXQEIF) & 1)							// check if TXQ buffer is empty
			return ERR_TXQEMPTY;									// return error code
			
		mcp2517_read_register(ADDR_C1TXQUA,ptrChn,4);				// read in contents of TXQ user address register
	}
	else{															// working with TX FIFO
		mcp2517_read_register(C1FIFOCON(bufNum),ptrChn,4);			// read in contents of FIFO config register
		if(!((ptrChn->regRd[0]>>TXEN) & 1))							// check if FIFO is a TX FIFO
			return ERR_NTXFIFO;										// return error code
			
		for(idx=0;idx<CSCNT;idx++);									// delay for toggling CS
		mcp2517_read_register(C1FIFOSTA(bufNum),ptrChn,4);			// read in contents of FIFO status register
		if((ptrChn->regRd[0]>>TFERFFIF) & 1)						// check if FIFO is empty
			return ERR_FIFOEMPTY;									// return error code
	}

	// Transmit request
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	ptrChn->regWr[1] = 0x02;										// FRESET=UINC=0;TXREQ=1
	if(!bufNum)														// working with TXQ buffer
		mcp2517_write_register(ADDR_C1TXQCON,ptrChn,1);				// write register data byte 1 
	else															// working with TX FIFO
		mcp2517_write_register(C1FIFOCON(bufNum),ptrChn,1);			// read register data bytes 
		
	return 0;
}
/**************************************************************************************************
Purpose: 	Aids in preparing the chnCAN message object parameters
Inputs:		*ptrChn	- chnCAN pointer
			id		- message ID
			fdf		- message FDF field
			bufLen	- message DLC field
			*buf_u8	- pointer to a payload u8 buffer array
						
Outputs:	none
**************************************************************************************************/
void mcp2517_msg_write(chnCAN *ptrChn,unsigned long id,uint8_t ide,uint8_t fdf,uint8_t brs,uint8_t rtr,uint8_t bufLen,uint8_t *buf_u8){
	uint8_t idx;													// used to step thru data buffer
	uint8_t len;													// used to hold #of bytes to write to SPI line
	uint8_t *ptr_u8;												// used to point to step thru byte members chnCAN object pointed to by *ptrChn
	
	ptr_u8 = &ptrChn->msg.sid07_00;									// point to sid07_00 member
	*(ptr_u8 + 0) = (id >> 0);										// sid07_00 
	*(ptr_u8 + 1) = (id >> 8);										// eid04_00,sid10_80
	*(ptr_u8 + 2) = (id >> 16);										// eid12_05
	*(ptr_u8 + 3) = (id >> 24);										// sid11,eid17_13
	ptrChn->msg.fdf = (fdf > 0);									// calculate the FDF
	ptrChn->msg.brs = (brs > 0);									// calculate the BRS
	ptrChn->msg.rtr = (rtr > 0);									// calculate the RTR
	ptrChn->msg.ide = (id > 0x7FF)||(ide > 0);						// calculate the IDE
	ptrChn->msg.dlc = mcp2517_dlc_payload(fdf,bufLen);				// calculate the DLC
	for(idx=0;idx<bufLen;idx++){									// loop thru buffer
		ptrChn->msg.txData[idx] = buf_u8[idx];
	}
//	ptrChn->msg.r0 = 0xee;
//	ptrChn->msg.r1 = 0xcc;
}
/**************************************************************************************************
Purpose: 	Sets up the GPIO interface/pins for both MCP2517 channels and configures the SPI hardware
Inputs:		chnNum	- channel #(s)
					  < 1 = Both
					  = 1 = channel 1
					  > 1 = channel 2
Outputs:	None
**************************************************************************************************/
void mcp2517_init_hardware(uint8_t chnNum){	
	if(chnNum <= 1){										// need to setup channel 1
		SET(MCP2517_CS1);									// default channel 1 chip select high 
		SET_OUTPUT(MCP2517_CS1);							// configure channel 1 chip select as an output 
		SET_INPUT(MCP2517_INT1);							// configure channel 1 interrupt pin as an input
		SET(MCP2517_INT1);
	}
	if(!chnNum || chnNum > 1){								// need to setup channel 2
		SET(MCP2517_CS2);									// default channel 2 chip select high 
		SET_OUTPUT(MCP2517_CS2);							// configure channel 2 chip select as an output
		SET_INPUT(MCP2517_INT2);							// configure channel 2 interrupt pin as an input
		SET(MCP2517_INT2);
	}
	
	RESET(P_SCK);											// default SPI SCK line low
	RESET(P_MOSI);											// default SPI MOSI line low
	RESET(P_MISO);											// default SPI MISO line low
	
	SET_OUTPUT(P_SS);										// configure SPI SS as an output
	SET_OUTPUT(P_SCK);										// configure SPI SCK as an output
	SET_OUTPUT(P_MOSI);										// configure SPI MOSI as an output
	SET_INPUT(P_MISO);										// configure SPI MISO as an output
	
	// active SPI master interface
	SPCR = (1<<SPE)|(1<<MSTR);								// config SPI - (SPI enable)|(SPI master)
	SPSR = (1<<SPI2X);										// config SPI - (Fosc/2) = 16Mhz/2 = 8Mhz
}	
/**************************************************************************************************
Purpose: 	Initializes the specified MCP2517 channel as a CAN2.0 channel
Inputs:		speed		- speed for the CAN channel
			*ptrChn		- chnCAN pointer
			chnNum		- channel #(s)
						< 1 = Both
						= 1 = channel 1
						> 1 = channel 2
			bufIdxTx	- index to Tx Que/FIFO to transmit msgs from (0-31)
			bufIdxRx	- index to Rx FIFO to filter msgs into (1-31)
Outputs:	None
**************************************************************************************************/
uint8_t mcp2517_init(uint8_t speed,chnCAN *ptrChn,uint8_t chnNum,uint8_t bufIdxTx,uint8_t bufIdxRx){
	uint8_t idx, temp, txIdx, rxIdx;
	
	txIdx = (bufIdxTx > 31) ? 31 : bufIdxTx;						// calculate TX buffer number 0 to 31=TXQ - FIFO1 to FIFO31
	rxIdx = (bufIdxRx > 31) ? 31 : (bufIdxRx + !bufIdxRx);			// calculate RX buffer number 1 to 31=FIFO1 to FIFO31
	if(txIdx==rxIdx)												// TX buffer equals RX buffer
		return 1;													// return fault code for this algorithm error
	
	ptrChn->chnNum = 1 + (chnNum > 1);								// calculate and set the CAN FD channel number (1 or 2)
	mcp2517_init_hardware(chnNum);									// initialize the specified CAN FD channel hardware
	
	mcp2517_cs_clr(ptrChn->chnNum);									// drive chn x chip select low (chip enable)
	spi_putCmd(SPI_RESET,0x000);									// reset chn x, uses address 0x000 for a reset
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)	
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	
	// Setup register write packet - OSC ------------------------------------------------------------------------------------------------------------
	ptrChn->regWr[0] = 0x40;										// CLKODIV=10;SCLKDIV=OSCDIS=PLLEN=0
	mcp2517_write_register(ADDR_OSC,ptrChn,0);						// write register data bytes
	for(idx=0;idx<CSCNT;idx++);
	mcp2517_read_register(ADDR_OSC,ptrChn,0);						// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,0))			// data did not write to the MCP2517
		return 10;													// return fault code for this register write error
	
	// Setup register write packet - IOCON ----------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x41,0x00,0x00,0x40);					// B3(SOF=TXCANOD=PM1=0;INTOD=PM0=1) B2(GPIO1=0;GPIO0=1) B1(read only) B0(XSTBYEN=1)
	mcp2517_write_register(ADDR_IOCON,ptrChn,4);					// write register data bytes
	mcp2517_cs_set(ptrChn->chnNum);									// drive chn x chip select high (chip disable)
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_IOCON,ptrChn,4);						// read register data bytes 
	if(ptrChn->regWr[3]!=ptrChn->regRd[3])							// data did not write to the MCP2517
		return 11;													// return fault code for this register write error
	if(ptrChn->regWr[0]!=ptrChn->regRd[0])							// data did not write to the MCP2517
		return 11;													// return fault code for this register write error

	// Setup register write packet - C1NBTCFG -------------------------------------------------------------------------------------------------------
	if(speed == CANSPEED_125)										// CANbus speed 125k
		mcp2517_reg_prep(ptrChn,1,0x00,0xFE,0x3F,0x3F);				// B3(BRP=0) B2(TSEG1=254) B1(TSEG2=63) B0(SJW=63)
	else if(speed == CANSPEED_250)									// CANbus speed 250k
		mcp2517_reg_prep(ptrChn,1,0x00,0x7E,0x1F,0x1F);				// B3(BRP=0) B2(TSEG1=126) B1(TSEG2=31) B0(SJW=31)
	else															// Default CANbus speed 500k
		mcp2517_reg_prep(ptrChn,1,0x00,0x3E,0x0F,0x0F);				// B3(BRP=0) B2(TSEG1=62) B1(TSEG2=15) B0(SJW=15)
	mcp2517_write_register(ADDR_C1NBTCFG,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1NBTCFG,ptrChn,4);					// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 101;													// return fault code for this register write error

	// Setup register write packet - C1DBTCFG -------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x00,0x1E,0x07,0x07);					// B3(BRP=0) B2(TSEG1=30) B1(TSEG2=7) B0(SJW=7)
	mcp2517_write_register(ADDR_C1DBTCFG,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1DBTCFG,ptrChn,4);					// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 102;													// return fault code for this register write error
		
	// Setup register write packet - C1TDC ----------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x00,0x00,0x00,0x00);					// B3(EDGFLTEN=SID11EN=0) B2(TDCMOD=0) B1(TDCO=16) B0(TDCV=0)
	mcp2517_write_register(ADDR_C1TDC,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1TDC,ptrChn,4);						// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 103;													// return fault code for this register write error
	
	// Setup register write packet - C1TSCON --------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x00,0x01,0x00,0x00);					// B3(RESERVED=0) B2(TSRES=TSEOF=0;TBCEN=1) B1(TBCPRE[9:8]=0) B0(TBCPRE[7:0]=0)
	mcp2517_write_register(ADDR_C1TSCON,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1TSCON,ptrChn,4);					// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 105;													// return fault code for this register write error
		
	// Setup register write packet - C1INT ----------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x00,0x02,0x00,0x00);					// B3(IVMIE=WAKIE=CERRIE-SERRIE=RXOVIE=TXATIE=SPICRCIE=ECCIE=0) B2(TEFIE=MODIE=TBCIE=0;RXIE=1;TXIE=0) B1(read only) B0(read only)
	mcp2517_write_register(ADDR_C1INT,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1INT,ptrChn,4);						// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,3))			// data did not write to the MCP2517
		return 107;													// return fault code for this register write error
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,2))			// data did not write to the MCP2517
		return 107;													// return fault code for this register write error
	
	// Setup register write packet - C1TEFCON -------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x1F,0x00,0x04,0x20);					// B3(FSIZE=0) B2(RESERVED=0) B1(FRESET=1;UNIC=0) B0(TEFTSEN=1;TEFOVIE=TEFFIE=TEFHIE=TEFNEIE=0)
	mcp2517_write_register(ADDR_C1TEFCON,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1TEFCON,ptrChn,4);					// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 116;													// return fault code for this register write error
	
	// Setup register write packet - C1TXQCON/C1FIFOCONn for Transmits  -----------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0xE0,0x60,0x04,0x80);					// B3(PLSIZE=7;FSIZE=0) B2(TXAT=3;TXPRI=0) B1(FRESET=1;TXREQ=UNIC=0) B0(TXEN=1;TXATIE=TXQEIE=TXQNIE=0)
	mcp2517_write_register(C1FIFOCON(txIdx),ptrChn,4);				// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(C1FIFOCON(txIdx),ptrChn,4);				// read register data bytes
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 119;													// return fault code for this register write error
		
	// Setup register write packet - C1FIFOCONn -----------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0xE3,0x60,0x04,0x21);					// B3(PLSIZE=7;FSIZE=3) B2(TXAT=3;TXPRI=0) B1(FRESET=1;TXREQ=UNIC=0) B0(TXEN=RTREN=TXATIE=RXOVIE=TFERFFIE=TFHRFHIE=0;TRNRFNIE=RXTSEN=1)
	mcp2517_write_register(C1FIFOCON(rxIdx),ptrChn,4);				// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(C1FIFOCON(rxIdx),ptrChn,4);				// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 122;													// return fault code for this register write error
	
	// Setup register write packet - C1CON ----------------------------------------------------------------------------------------------------------
	mcp2517_reg_prep(ptrChn,1,0x00,0x98,0x07,0x40);					// B3(TXBWS=ABAT=REQOP=0) B2(OPMOD=4;TXQEN=STEF=1;SERR2LOM=ESIGM=RTXAT=0) B1(BRSDIS=0;WFT=3;WAKFIL=1) B0(PXEDIS=1;ISOCRCEN=DNCNT=0)
	mcp2517_write_register(ADDR_C1CON,ptrChn,4);					// write register data bytes
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(ADDR_C1CON,ptrChn,4);						// read register data bytes 
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 199;													// return fault code for this register write error

	return 0;														// return value for success
}
/**************************************************************************************************
Purpose: 	Compares the contents of 2 uint8_t buffers
Inputs:		*ptrBuf0	- pointer to a uint8_t buffer0
			*ptrBuf1	- pointer to a uint8_t buffer1
			dataNum 	- data byte(s) to be compared in buffer0 & buffer1
						  0 = Compare byte 0 (LSB)
						  1 = Compare byte 1
						  2 = Compare byte 2
						  3 = Compare byte 3 (MSB)
						  4 = Compare all bytes
						
Outputs:	result		- result of the comparison
						0 = byte or bytes mismatch
						1 = byte or bytes match
**************************************************************************************************/
uint8_t mcp2517_reg_compr(uint8_t *ptrBuf0,uint8_t *ptrBuf1,uint8_t dataNum){
	uint8_t idx;											// variable for array indexing
	uint8_t len;											// used to hold #of bytes or byte# to compare
	
	len = (dataNum > 4) ? 4 : dataNum;						// Cap data length or set to data # provided

	if (len < 4){											// compare a specific byte in buffers
		idx = len;											// update the index to byte of interest
		return ptrBuf0[idx] == ptrBuf1[idx];				// return results of comparision
	}
	
	for(idx=0;idx<len;idx++){								// step thru elements
		if(ptrBuf0[idx] != ptrBuf1[idx])					// compare the buffers
			return 0;										// return 0 on 1st byte mismatch
	}
	return 1;												// return 1 if byte(s) match
}
/**************************************************************************************************
Purpose: 	Sets up an Rx message ID filter for specified MCP2517 channel
Inputs:		*ptrChn		- chnCAN pointer
			bufIdx		- index to Rx FIFO to filter msgs into (1-31)
			fltrNum 	- index to filter register (0-7)
			fltrIdx		- filter number of filter register to setup (0-3)
			fltrType	- msg ID type (0=filter applies to standard or extended; 1=applies to standard; 2=applies to extended)
			msgId 		- ID of msg to filter (standard or extended in length)
			mskId		- Bits in msgId to mask for comparison (0=don't care; 1=exact match)
			fltrNum 	- filter number of filter register to setup
			
Outputs:	None
**************************************************************************************************/
uint8_t mcp2517_fltr_setup(chnCAN *ptrChn,uint8_t bufIdx,uint8_t fltrNum,uint8_t fltrIdx,uint8_t fltrType,unsigned long msgId,unsigned long mskId){
	uint8_t idx, temp, bufIdx2, fltrIdx2, fltrNum2, fltrObj, fltrMsk;
	
	bufIdx2 = (bufIdx > 31) ? 31 : (bufIdx + !bufIdx);				// calculate RX buffer number 1 to 31=FIFO1 to FIFO31
	fltrNum2 = (fltrNum > 7) ? 7 : fltrNum;							// calculate filter num of the filter register
	fltrIdx2 = (fltrIdx > 3) ? 3 : fltrIdx;							// calculate filter register index
	fltrObj = 4*fltrNum2 + fltrIdx2;								// calculate filter object index
	fltrMsk = fltrObj;												// calculate filter mask index

	// Write to C1FLTCONn to disable appropriate filter number ----------------------------------------------------------------------------
	ptrChn->regWr[fltrIdx2] = bufIdx2;								// FLTEN=0;F0BP=bufIdx2
	mcp2517_write_register(C1FLTCON(fltrNum2),ptrChn,fltrIdx2);		// write register data bytes
	
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS	
	mcp2517_read_register(C1FLTCON(fltrNum2),ptrChn,fltrIdx2);		// read register data bytes 
	
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,fltrIdx2))	// data did not write to the MCP2517
		return 11;													// return fault code for this register write error
		
	// Write to C1FLTOBJn the msg ID to filter on -----------------------------------------------------------------------------------------
	temp = (fltrType>=2) || !fltrType;								// Calculate EXIDE bit
	ptrChn->regWr[3] = (msgId >> 24) & 0x1F | (temp << 6);			// EXIDE;SID11;EID<17:13>
	ptrChn->regWr[2] = (msgId >> 16);								// EID<12:5>
	ptrChn->regWr[1] = (msgId >> 8);								// EID<4:0>;SID<7:0>
	ptrChn->regWr[0] = (msgId >> 0);								// SID<7:0>
	mcp2517_write_register(C1FLTOBJ(fltrObj),ptrChn,4);				// write register data bytes	

	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS	
	mcp2517_read_register(C1FLTOBJ(fltrObj),ptrChn,4);				// read register data bytes 
	
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 21;													// return fault code for this register write error
	
	// Write to C1MASKn the mask of msg ID to filter on -----------------------------------------------------------------------------------
	temp = (fltrType>=1);											// Calculate MIDE bit
	ptrChn->regWr[3] = (mskId >> 24) & 0x1F | (temp << 6);			// MIDE;MSID11;MEID<17:13>
	ptrChn->regWr[2] = (mskId >> 16);								// MEID<12:5>
	ptrChn->regWr[1] = (mskId >> 8);								// MEID<4:0>;MSID<7:0>
	ptrChn->regWr[0] = (mskId >> 0);								// MSID<7:0>
	mcp2517_write_register(C1MASK(fltrMsk),ptrChn,4);				// write register data bytes
	
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(C1MASK(fltrMsk),ptrChn,4);				// read register data bytes 
	
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,4))			// data did not write to the MCP2517
		return 31;
		
	// Write to C1FLTCONn to enable appropriate filter number -----------------------------------------------------------------------------
	ptrChn->regWr[fltrIdx2] = bufIdx2 | 0x80;						// FLTEN=1;F0BP=bufIdx2
	mcp2517_write_register(C1FLTCON(fltrNum2),ptrChn,fltrIdx2);		// write register data bytes
	
	for(idx=0;idx<CSCNT;idx++);										// delay for toggling CS
	mcp2517_read_register(C1FLTCON(fltrNum2),ptrChn,fltrIdx2);		// read register data bytes 
	
	if(!mcp2517_reg_compr(&ptrChn->regWr,&ptrChn->regRd,fltrIdx2))	// data did not write to the MCP2517
		return 12;													// return fault code for this register write error
	
	return 0;														// return value for success
}
/**************************************************************************************************
Purpose: 	Returns #of payload bytes to write to MCP2517 memory for a CAN message
Inputs:		fdf	- message FDF field
			dlc	- message DLC field
						
Outputs:	result	- #of payload bytes in multiples of 4
					if(0<=dlc<=8) 
						result = 0,4,8
					else if(!fdf) 
						result = 8
					else
						result = 12,16,20,24,32,48,64
**************************************************************************************************/
uint8_t mcp2517_mem_payload(uint8_t fdf,uint8_t dlc){
	if(dlc <= 8){	return dlc;								// valid for both CAN2.0 & CAN FD frames
		if		(dlc == 0) 		return 0;
		else if	(dlc <= 4) 		return 4;
		else if	(dlc <= 8) 		return 8;
	}
	else if(!fdf)	return 8;								// valid for CAN2.0 frames	
	else{													// CAN FD frame and DLC > 8
		if		(dlc == 9) 		return 12;
		else if	(dlc == 10) 	return 16;
		else if	(dlc == 11) 	return 20;
		else if	(dlc == 12) 	return 24;
		else if	(dlc == 13) 	return 32;
		else if	(dlc == 14) 	return 48;
		else if	(dlc == 15) 	return 64;
	}
}
/**************************************************************************************************
Purpose: 	Returns #of payload bytes based on DLC and FDF fields
Inputs:		fdf	- message FDF field
			dlc	- message DLC field
						
Outputs:	result	- #of payload bytes
					if(0<=dlc<=8) 
						result = 0-8
					else if(!fdf) 
						result = 8
					else
						result = 12-64
**************************************************************************************************/
uint8_t mcp2517_len_payload(uint8_t fdf,uint8_t dlc){
    if(dlc <= 8){   return dlc;                             // valid for both CAN2.0 & CAN FD frames
        return dlc;											// DLC = payload length
    }
    else if(!fdf)   return 8;                               // valid for CAN2.0 frames  
    else{                                                   // CAN FD frame and DLC > 8
        if      (dlc == 9)      return 12;					// CAN FD standard
        else if (dlc == 10)     return 16;					// CAN FD standard
        else if (dlc == 11)     return 20;					// CAN FD standard
        else if (dlc == 12)     return 24;					// CAN FD standard
        else if (dlc == 13)     return 32;					// CAN FD standard
        else if (dlc == 14)     return 48;					// CAN FD standard
        else if (dlc == 15)     return 64;					// CAN FD standard
    }
}
/**************************************************************************************************
Purpose: 	Returns DLC for a CAN message
Inputs:		fdf		- message FDF field
			bufLen	- #of payload bytes
						
Outputs:	result	- DLC (0-15)				
**************************************************************************************************/
uint8_t mcp2517_dlc_payload(uint8_t fdf,uint8_t bufLen){
	if(bufLen <= 8)	return bufLen;							// valid for both CAN2.0 & CAN FD frames		
	else if(!fdf)	return 8;								// valid for CAN2.0 frames	
	else{													// CAN FD frame and payload lengths > 8
		if		(bufLen <= 12) 	return 9;
		else if	(bufLen <= 16) 	return 10;
		else if	(bufLen <= 20) 	return 11;
		else if	(bufLen <= 24) 	return 12;
		else if	(bufLen <= 32) 	return 13;
		else if	(bufLen <= 48) 	return 14;
		else					return 15;
	}
}
/**************************************************************************************************
Purpose: 	Calculates CANbus msg ID
Inputs:		*ptrChn	- chnCAN pointer
						
Outputs:	result	- msg ID
**************************************************************************************************/
unsigned long mcp2517_id_calc(chnCAN *ptrChn) {
	unsigned long data, temp;
	
	temp = (temp << 3) | ptrChn->msg.sid10_08;
	data = (temp << 8) | ptrChn->msg.sid07_00;
	if(ptrChn->msg.ide){
		temp = ptrChn->msg.eid17_13;
		temp = (temp << 8) | ptrChn->msg.eid12_05;
		temp = (temp << 5) | ptrChn->msg.eid04_00;
		return ((data << 18) | temp);
	}
	else{
		return data;
	}
}
/**************************************************************************************************
Purpose: 	Calculates the timestamp of an Rx message from timestamp register bytes
Inputs:		*ptrChn	- chnCAN pointer
			
Outputs:	None
**************************************************************************************************/
void mcp2517_tstamp_calc(chnCAN *ptrChn){
	ptrChn->msg.tStamp = ptrChn->msg.rxTstamp[3];
	ptrChn->msg.tStamp = (ptrChn->msg.tStamp << 8) | ptrChn->msg.rxTstamp[2];
	ptrChn->msg.tStamp = (ptrChn->msg.tStamp << 8) | ptrChn->msg.rxTstamp[1];
	ptrChn->msg.tStamp = (ptrChn->msg.tStamp << 8) | ptrChn->msg.rxTstamp[0];
}
/**************************************************************************************************
Purpose: 	Prepares the regWr & regRd member in the chnCAN object pointed to by ptrChn
Inputs:		*ptrChn	- chnCAN pointer
			bitRdWr - read/write flag (0=write to regRd;1=write to regWr)
			byte3	- value to write to either regRd[3] or regWr[3]
			byte2	- value to write to either regRd[2] or regWr[2]
			byte1	- value to write to either regRd[1] or regWr[1]
			byte0	- value to write to either regRd[0] or regWr[0]
			
Outputs:	None
**************************************************************************************************/
void mcp2517_reg_prep(chnCAN *ptrChn, uint8_t bitRdWr, uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0){
	if(bitRdWr){
		ptrChn->regWr[3] = byte3;
		ptrChn->regWr[2] = byte2;
		ptrChn->regWr[1] = byte1;
		ptrChn->regWr[0] = byte0;
	}
	else{
		ptrChn->regRd[3] = byte3;
		ptrChn->regRd[2] = byte2;
		ptrChn->regRd[1] = byte1;
		ptrChn->regRd[0] = byte0;
	}
}












