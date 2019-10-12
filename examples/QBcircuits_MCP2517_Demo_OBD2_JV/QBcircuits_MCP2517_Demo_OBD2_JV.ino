/************************************************************************
CAN FD Write Demo for the QBcircuits CAN FD Shield. 

Written by BrotherQ. 
Original tutorial available here: http://www.qbcircuits.com

Distributed as-is; no warranty is given.

Demo summary:
This demo originally is setup to configure CAN FD channel 1
to send an OBD service 0x01 message PID 0x0C to ECU ID 0x7E0 
on the monitored CANbus network.
*************************************************************************/
#include <qb_mcp2517_defs.h>
#include <qb_mcp2517.h>
#include <qb_obd2.h>

// Global Variables *************************************************************************************************************************************************************//
byte i_u8               = 0;                                                                    // array indexing
byte j_u8               = 0;                                                                    // array indexing
byte chnIdx             = 0;                                                                    // storing CANbus message channel index
byte txData[8]          = {0};                                                                  // storing CANbus message payload data byte(s)
byte rVal[4]            = {0};                                                                  // sub-routines return values

chnCAN can1;                                                                                    // CAN FD channel struct

// Setup Function ***************************************************************************************************************************************************************//
void setup() {
  Serial.begin(115200);                                                                         // setup serial UART 
  while (!Serial){}                                                                             // ** pro micro only **

  rVal[0] = mcp2517_init(CANSPEED_500,&can1,1,TXQ,FIFO1);                                       // setup CAN FD chn 1 for CAN FD & uses TXQ for transmits & FIFO 1 as Rx FIFO
  rVal[1] = mcp2517_fltr_setup(&can1,FIFO1,FLTRNUM0,FLTRIDX0,FLTRSID,0x7E8,0xFF8);              // setup CAN FD chn 1 Rx filter to only recieve standard msg IDs 0x7E8-0x7EF

  if(rVal[0] || rVal[1]){                                                                       // 1 or more CAN channels failed to initialize
    Serial.println("Failed to configure CAN channel");                                          // format/print failure diagnostic message
    Serial.println("------------------------------------------------");                         // format/print seperator
    Serial.println("CAN1 channel status     = Enabled");                                        // format/print CAN1 enabled
    Serial.print("CAN1 init status code   = ");Serial.println(rVal[0]);                         // format/print channel 1 setup failure code
    Serial.print("CAN1 filter status code = ");Serial.println(rVal[1]);                         // format/print channel 1 filter setup failure code
    while(1){}                                                                                  // wait here forever
  }
}

// Main Loop ********************************************************************************************************************************************************************//
void loop(){                                                                                    // system main
  txData[0] = 2;                                                                                // #of additional data bytes
  txData[1] = 0x01;                                                                             // service code
  txData[2] = 0x3C;                                                                             // Engine speed
  mcp2517_msg_write(&can1,0x7E0,!IDE,!FDF,!BRS,!RTR,8,txData);                                  // populate channel struct with necessary transmit info
  mcp2517_write_memory(TXQ,&can1);                                                              // write the Tx message info to MCP2517 TXQ
  mcp2517_start_transmit(TXQ,&can1);                                                            // initiate a transmit onto the CANbus network
  
  delay(500);                                                                                   // delay to allow currently polled ECU to respond
  
  if(mcp2517_check_message(&can1)){                                                             // check if current chnCAN has recieved a msg
    mcp2517_read_memory(FIFO1,&can1);                                                           // read current chnCAN message stored in FIFO1
    if(can1.msg.rxData[1] == (txData[1] + 0x40)){                                               // check for +response from ECU OBD2 service 0
      obd2s1PidDecrypt(can1.msg.rxData);                                                        // convert OBD2 data to value(s)
    }
    else{                                                                                       // ECU negative response
      Serial.print("Data [");                                                                   // format/print OBD2 data 
      for(j_u8=0;j_u8<8;j_u8++){                                                                // loop thru all data byte in OBD2 response packet
        if(can1.msg.rxData[j_u8] < 0x10){Serial.print("0");}                                    // format/print HEX data leading zero
        Serial.print(can1.msg.rxData[j_u8], HEX);                                               // format/print OBD2 response packet data bytes
        if(j_u8!=7){Serial.print(" ");}                                                         // format/print space
      }
      Serial.println("] - ECU negative response");                                              // format/print ECU negative response
    }
  }
  else{                                                                                         // ECU no response
    Serial.println("[] - ECU no response");                                                     // format/print ECU negative response
  }
}
