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
#ifndef	QB_MCP2517_DEFAULTS_H
#define	QB_MCP2517_DEFAULTS_H

// Arduino Pro Micro
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define	P_MOSI				B,2			// PB2
#define	P_MISO				B,3			// PB3
#define	P_SCK				B,1			// PB1
#define	P_SS				B,0			// PB0
#define	MCP2517_CS1			B,6 		// PB6
#define	MCP2517_INT1		E,6			// PE6
#define	MCP2517_CS2			F,4 		// PF4
#define	MCP2517_INT2		B,4			// PB4

// Arduino Pro Mini, Uno, mega
#else
#define	P_MOSI				B,3			// PB3
#define	P_MISO				B,4			// PB4
#define	P_SCK				B,5			// PB5
#define	P_SS				B,2			// PB2
#define	MCP2517_CS1			B,2			// PB2
#define	MCP2517_INT1		D,7			// PD7
#define	MCP2517_CS2			C,3 		// PC3
#define	MCP2517_INT2		B,0			// PB0	
#endif

#endif	// QB_MCP2517_DEFAULTS_H