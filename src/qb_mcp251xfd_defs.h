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
#ifndef	QB_MCP251XFD_DEFS_H
#define	QB_MCP251XFD_DEFS_H

/**************************************************************************************************
SPI Instructions from MCP2517FD manual Table 4-1 
**************************************************************************************************/
#define SPI_RESET			0x0		// modified from mcp2515 0xC0 -> 0x00
#define	SPI_READ			0x3		// carry over from mcp2515
#define	SPI_WRITE			0x2		// carry over from mcp2515
#define SPI_READ_CRC		0xB		// MCP2517
#define	SPI_WRITE_CRC		0xA		// MCP2517
#define	SPI_WRITE_SAFE		0xC		// MCP2517

/**************************************************************************************************
MCP2517FD Special Function Registers (SFR) addresses (LSB) from MCP2517FD manual Table 3-1 
**************************************************************************************************/
#define ADDR_OSC			0xE00
#define ADDR_IOCON			0xE04
#define ADDR_CRC			0xE08
#define ADDR_ECCCON			0xE0C
#define ADDR_ECCSTAT		0xE10

/**************************************************************************************************
MCP2517FD Special Function Registers (SFR) from MCP2517FD manual Table 3-1 
**************************************************************************************************/
// Register - OSC
#define PLLEN				0x00
#define OSCDIS				0x02
#define SCLKDIV				0x04
#define CLKODIV				0x05
#define PLLRDY				0x00
#define OSCRDY				0x02
#define SCLKRDY				0x04

// Register - IOCON
#define TRIS0				0x00
#define TRIS1				0x01
#define XSTBYEN				0x06
#define LAT0				0x00
#define LAT1				0x01
#define GPIO0				0x00
#define GPIO1				0x01
#define PM0					0x00
#define PM1					0x01
#define TXCANOD				0x04
#define SOF					0x05
#define INTOD				0x06

// Register - CRC
#define CRCL				0x00
#define CRCH				0x00
#define CRCERRIF			0x00
#define FERRIF				0x01
#define CRCERRIE			0x00
#define FERRIE				0x01

// Register - ECCCON
#define ECCEN				0x00
#define SECIE				0x01
#define DEDIE				0x02
#define PARITY				0x00

// Register - ECCSTAT
#define DEDIF				0x01
#define SECIF				0x02
#define ERRADDRL			0x00
#define ERRADDRH			0x00

/**************************************************************************************************
CAN FD Controller Module (SFR) addresses (LSB) from MCP2517FD manual Table 3-2 
**************************************************************************************************/
#define ADDR_C1CON			0x000
#define ADDR_C1NBTCFG		0x004
#define ADDR_C1DBTCFG		0x008
#define ADDR_C1TDC			0x00C
#define ADDR_C1TBC			0x010
#define ADDR_C1TSCON		0x014
#define ADDR_C1VEC			0x018
#define ADDR_C1INT			0x01C
#define ADDR_C1RXIF			0x020
#define ADDR_C1TXIF			0x024
#define ADDR_C1RXOVIF		0x028
#define ADDR_C1TXATIF		0x02C
#define ADDR_C1TXREQ		0x030
#define ADDR_C1TREC			0x034
#define ADDR_C1BDIAG0		0x038
#define ADDR_C1BDIAG1		0x03C

#define ADDR_C1TEFCON		0x040
#define ADDR_C1TEFSTA		0x044
#define ADDR_C1TEFUA		0x048

#define ADDR_RESERVED		0x04C

#define ADDR_C1TXQCON		0x050
#define ADDR_C1TXQSTA		0x054
#define ADDR_C1TXQUA		0x058

#define C1FIFOCON(m)		0x050 + (m * 12)		// m=(1-31)
#define C1FIFOSTA(m)		0x054 + (m * 12)		// m=(1-31)
#define C1FIFOUA(m)			0x058 + (m * 12)		// m=(1-31)

#define C1FLTCON(m)			0x1D0 + (m * 4)			// m=(0-7)
#define C1FLTOBJ(m)			0x1F0 + (m * 8)			// m=(0-7)
#define C1MASK(m)			0x1F4 + (m * 8)			// m=(0-7)

/**************************************************************************************************
CAN FD Controller Module (SFR) from MCP2517FD manual Table 3-2 
**************************************************************************************************/
// Register - C1CON $$$
#define DNCNT				0x00
#define ISOCRCEN			0x05
#define PXEDIS				0x06
#define WAKFIL				0x00
#define WFT					0x01
#define BUSY				0x03
#define BRSDIS				0x04
#define RTXAT				0x00
#define ESIGM				0x01
#define SERR2LOM			0x02
#define STEF				0x03
#define TXQEN				0x04
#define OPMOD				0x05
#define REQOP				0x00
#define ABAT				0x03
#define TXBWS				0x04

// Register - C1NBTCFG,C1DBTCFG $$$
#define SJW					0x00
#define TSEG2				0x00
#define TSEG1				0x00
#define BRP					0x00

// Register - C1TDC $$$
#define TDCV				0x00
#define TDCO				0x00
#define TDCMOD				0x00
#define SID11EN				0x00
#define EDGFLTEN			0x01

// Register - C1TBC $$$
#define TBC					0x00

// Register - C1TSCON $$$
#define TBCPREL				0x00
#define TBCPREH				0x00
#define TBCEN				0x00
#define TSEOF				0x01
#define TSRES				0x02

// Register - C1VEC $$$
#define ICODE				0x00
#define FILHIT				0x00
#define TXCODE				0x00
#define RXCODE				0x00

// Register - C1INT $$$
#define TXIF				0x00
#define RXIF				0x01
#define TBCIF				0x02
#define MODIF				0x03
#define TEFIF				0x04
#define ECCIF				0x00
#define SPICRCIF			0x01
#define TXATIF				0x02
#define RXOVIF				0x03
#define SERRIF				0x04
#define CERRIF				0x05
#define WAKIF				0x06
#define IVMIF				0x07
#define TXIE				0x00
#define RXIE				0x01
#define TBCIE				0x02
#define MODIE				0x03
#define TEFIE				0x04
#define ECCIE				0x00
#define SPICRCIE			0x01
#define TXATIE				0x02
#define RXOVIE				0x03
#define SERRIE				0x04
#define CERRIE				0x05
#define WAKIE				0x06
#define IVMIE				0x07

// Register - C1RXIF $$$
#define RFIF0				0x00	// RFIF    08,16,24
#define RFIF1				0x01	// RFIF 01,09,17,25
#define RFIF2				0x02	// RFIF 02,10,18,26
#define RFIF3				0x03	// RFIF 03,11,19,27
#define RFIF4				0x04	// RFIF 04,12,20,28
#define RFIF5				0x05	// RFIF 05,13,21,29
#define RFIF6				0x06	// RFIF 06,14,22,30
#define RFIF7				0x07	// RFIF 07,15,23,31

// Register - C1RFOVIF $$$
#define RFOVIF0				0x00	// RFOVIF    08,16,24
#define RFOVIF1				0x01	// RFOVIF 01,09,17,25
#define RFOVIF2				0x02	// RFOVIF 02,10,18,26
#define RFOVIF3				0x03	// RFOVIF 03,11,19,27
#define RFOVIF4				0x04	// RFOVIF 04,12,20,28
#define RFOVIF5				0x05	// RFOVIF 05,13,21,29
#define RFOVIF6				0x06	// RFOVIF 06,14,22,30
#define RFOVIF7				0x07	// RFOVIF 07,15,23,31

// Register - C1TXIF $$$
#define TFIF0				0x00	// TFIF 00,08,16,24
#define TFIF1				0x01	// TFIF 01,09,17,25
#define TFIF2				0x02	// TFIF 02,10,18,26
#define TFIF3				0x03	// TFIF 03,11,19,27
#define TFIF4				0x04	// TFIF 04,12,20,28
#define TFIF5				0x05	// TFIF 05,13,21,29
#define TFIF6				0x06	// TFIF 06,14,22,30
#define TFIF7				0x07	// TFIF 07,15,23,31

// Register - C1TXATIF $$$
#define TFATIF0				0x00	// TFATIF 00,08,16,24
#define TFATIF1				0x01	// TFATIF 01,09,17,25
#define TFATIF2				0x02	// TFATIF 02,10,18,26
#define TFATIF3				0x03	// TFATIF 03,11,19,27
#define TFATIF4				0x04	// TFATIF 04,12,20,28
#define TFATIF5				0x05	// TFATIF 05,13,21,29
#define TFATIF6				0x06	// TFATIF 06,14,22,30
#define TFATIF7				0x07	// TFATIF 07,15,23,31

// Register - C1TXREQ $$$
#define TXREQ0				0x00	// TXREQ 00,08,16,24
#define TXREQ1				0x01	// TXREQ 01,09,17,25
#define TXREQ2				0x02	// TXREQ 02,10,18,26
#define TXREQ3				0x03	// TXREQ 03,11,19,27
#define TXREQ4				0x04	// TXREQ 04,12,20,28
#define TXREQ5				0x05	// TXREQ 05,13,21,29
#define TXREQ6				0x06	// TXREQ 06,14,22,30
#define TXREQ7				0x07	// TXREQ 07,15,23,31

// Register - C1TREC $$$
#define REC					0x00
#define TEC					0x00
#define EWARN				0x00
#define RXWARN				0x01
#define TXWARN				0x02
#define RXBP				0x03
#define TXBP				0x04
#define TXBO				0x05

// Register - C1BDIAG0 $$$
#define NRERRCNT			0x00
#define NTERRCNT			0x00
#define DRERRCNT			0x00
#define DTERRCNT			0x00

// Register - C1BDIAG1 $$$
#define EFMSGCNTL			0x00
#define EFMSGCNTH			0x00
#define NBIT0ERR			0x00
#define NBIT1ERR			0x01
#define NACKERR				0x02
#define NFORMERR			0x03
#define NSTUFERR			0x04
#define NCRCERR				0x05
#define TXBOERR				0x07
#define DBIT0ERR			0x00
#define DBIT1ERR			0x01
#define DFORMERR			0x03
#define DSTUFERR			0x04
#define DCRCERR				0x05
#define ESI					0x06
#define DLCMM				0x07

// Register - C1TEFCON $$$
#define TEFNEIE				0x00
#define TEFHIE				0x01
#define TEFFIE				0x02
#define TEFOVIE				0x03
#define TEFTSEN				0x05
#define UINC				0x00
#define FRESET				0x02
#define FSIZE				0x00

// Register - C1TEFSTA $$$
#define TEFNEIF				0x00
#define TEFHIF				0x01
#define TEFFIF				0x02
#define TEFOVIF				0x03

// Register - C1TEFUA $$$
#define TEFUA				0x00

// Register - C1TXQCON $$$
#define TXQNIE				0x00
#define TXQEIE				0x02
#define XQTXATIE			0x04
#define TXEN				0x07
#define XQUINC				0x00	
#define XQTXREQ				0x01
#define XQFRESET			0x02	
#define TXPRI				0x00
#define TXAT				0x05
#define XQFSIZE				0x00
#define PLSIZE				0x05

// Register - C1TXQSTA $$$
#define TXQNIF				0x00
#define TXQEIF				0x02
#define XQTXATIF			0x04
#define XQTXERR				0x05
#define TXLARB				0x06
#define TXABT				0x07
#define TXQCI				0x00

// Register - C1TXQUA $$$
#define TXQUA				0x00

// Register - C1FIFOCON1-C1FIFOCON31 $$$
#define TFNRFNIE			0x00
#define TFHRFHIE			0x01
#define TFERFFIE			0x02
#define FFRXOVIE			0x03
#define FFTXATIE			0x04
#define RXTSEN				0x05
#define RTREN				0x06
#define TXEN				0x07
#define UINC				0x00
#define TXREQ				0x01
#define FRESET				0x02
#define TXPRI				0x00
#define TXAT				0x05
#define FSIZE				0x00
#define PLSIZE				0x05

// Register - C1FIFOSTA1-31 $$$
#define TFNRFNIF			0x00
#define TFHRFHIF			0x01
#define TFERFFIF			0x02
#define FFRXOVIF			0x03
#define FFTXATIF			0x04
#define TXERR				0x05
#define TXLARB				0x06
#define TXABT				0x07
#define FIFOCI				0x00

// Register - C1FIFOUA1-31 $$$
#define FIFOUA				0x00

// Register - C1FLTCON0 $$$
#define F0BP				0x00
#define FLTEN0				0x07
#define F1BP				0x00
#define FLTEN1				0x07
#define F2BP				0x00
#define FLTEN2				0x07
#define F3BP				0x00
#define FLTEN3				0x07

// Register - C1FLTCON1 $$$
#define F4BP				0x00
#define FLTEN4				0x07
#define F5BP				0x00
#define FLTEN5				0x07
#define F6BP				0x00
#define FLTEN6				0x07
#define F7BP				0x00
#define FLTEN7				0x07

// Register - C1FLTCON2 $$$
#define F8BP				0x00
#define FLTEN8				0x07
#define F9BP				0x00
#define FLTEN9				0x07
#define F10BP				0x00
#define FLTEN10				0x07
#define F11BP				0x00
#define FLTEN11				0x07

// Register - C1FLTCON3 $$$
#define F12BP				0x00
#define FLTEN12				0x07
#define F13BP				0x00
#define FLTEN13				0x07
#define F14BP				0x00
#define FLTEN14				0x07
#define F15BP				0x00
#define FLTEN15				0x07

// Register - C1FLTCON4 $$$
#define F16BP				0x00
#define FLTEN16				0x07
#define F17BP				0x00
#define FLTEN17				0x07
#define F18BP				0x00
#define FLTEN18				0x07
#define F19BP				0x00
#define FLTEN19				0x07

// Register - C1FLTCON5 $$$
#define F20BP				0x00
#define FLTEN20				0x07
#define F21BP				0x00
#define FLTEN21				0x07
#define F22BP				0x00
#define FLTEN22				0x07
#define F23BP				0x00
#define FLTEN23				0x07

// Register - C1FLTCON6 $$$
#define F24BP				0x00
#define FLTEN24				0x07
#define F25BP				0x00
#define FLTEN25				0x07
#define F26BP				0x00
#define FLTEN26				0x07
#define F27BP				0x00
#define FLTEN27				0x07

// Register - C1FLTCON7 $$$
#define F28BP				0x00
#define FLTEN28				0x07
#define F29BP				0x00
#define FLTEN29				0x07
#define F30BP				0x00
#define FLTEN30				0x07
#define F31BP				0x00
#define FLTEN31				0x07

// Register - C1FLTOBJ0-31 $$$
#define SIDL				0x00
#define SIDU				0x00
#define EIDL				0x03
#define EIDM				0x00
#define EIDU				0x00
#define SID11				0x05
#define EXIDE				0x06

// Register - C1MASK0-31 $$$
#define MSIDL				0x00
#define MSIDU				0x00
#define MEIDL				0x03
#define MEIDM				0x00
#define MEIDU				0x00
#define MSID11				0x05
#define MIDE				0x06

#endif	// QB_MCP2517XFD_DEFS_H
