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
#ifndef	QB_MCP2517_GLOBAL_H
#define	QB_MCP2517_GLOBAL_H

// ----------------------------------------------------------------------------
#define	RESET(x)		_XRS(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)
#define	SET(x)			_XS(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)
#define	TOGGLE(x)		_XT(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)
#define	SET_OUTPUT(x)	_XSO(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)
#define	SET_INPUT(x)	_XSI(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)
#define	IS_SET(x)		_XR(x)							// x = (@,#); @=Port letter,#=Port number; Ex. x=B,0(Refers to PB0)

#define	PORT(x)			_port2(x)						// x = @; @=Port letter; Ex. x=B (PORT(x)= PORTB)
#define	DDR(x)			_ddr2(x)						// x = @; @=Port letter; Ex. x=B (DDR(x)= DDRB)
#define	PIN(x)			_pin2(x)						// x = @; @=Port letter; Ex. x=B (PIN(x)= PINB)

#define	_XRS(x,y)		PORT(x) &= ~(1<<y)				// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XRS(x,y)=PORTB &= ~(1<<0)
#define	_XS(x,y)		PORT(x) |= (1<<y)				// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XS(x,y)=PORTB |= (1<<0)
#define	_XT(x,y)		PORT(x) ^= (1<<y)				// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XT(x,y)=PORTB ^= (1<<0)

#define	_XSO(x,y)		DDR(x) |= (1<<y)				// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XSO(x,y)=DDRB |= (1<<0)
#define	_XSI(x,y)		DDR(x) &= ~(1<<y)				// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XSI(x,y)=DDRB &= ~(1<<0)

#define	_XR(x,y)		((PIN(x) & (1<<y)) != 0)		// x=@,y=#; @=Port letter,#=Port number; Ex. x=B,y=0(_XR(x,y)=(PINB & (1<<0)) != 0

#define	_port2(x)		PORT ## x						// ##(concatenation) x=@; @=Port letter; Ex. x=B (_port2(x)=PORTB)
#define	_ddr2(x)		DDR ## x						// ##(concatenation) x=@; @=Port letter; Ex. x=B (_ddr2(x)=DDRB)
#define	_pin2(x)		PIN ## x						// ##(concatenation) x=@; @=Port letter; Ex. x=B (_pin2(x)=PINB)

#endif	// QB_MCP2517_GLOBAL_H
