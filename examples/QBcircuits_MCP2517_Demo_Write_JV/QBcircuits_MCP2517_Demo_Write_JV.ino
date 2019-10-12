/************************************************************************
CAN FD Write Demo for the QBcircuits CAN FD Shield. 

Written by BrotherQ. 
Original tutorial available here: http://www.qbcircuits.com

Distributed as-is; no warranty is given.

Demo summary:
This demo originally is setup to configure both CAN FD channels 1 & 2
to write demo CAN 2.0/CAN FD dummy messages from channel 1 onto the 
monitored CANbus network and recieve only extended ID 0.
*************************************************************************/
#include <qb_mcp2517_defs.h>
#include <qb_mcp2517.h>

// Global Variables *************************************************************************************************************************************************************//
#define MSGMAX  6                                                                               // array sizing {can be altered} ** must be >0
#define CAN1    0                                                                               // system critical {do not change}
#define CAN2    1                                                                               // system critical {do not change}
#define DBG_BL  0                                                                               // debug flag {can be altered 0=no debugging;1=debugging}

byte i_u8       = 0;                                                                            // array indexing
byte j_u8       = 0;                                                                            // array indexing
byte idx        = 0;                                                                            // array indexing
byte mode_u8    = 0;                                                                            // system state machine control
byte chnIdx     = 0;                                                                            // storing CANbus message channel index
byte bfIde      = 0;                                                                            // storing CANbus message IDE value
byte bfFdf      = 0;                                                                            // storing CANbus message FDF value
byte bfBrs      = 0;                                                                            // storing CANbus message BRS value
byte bfDlc      = 0;                                                                            // storing CANbus message DLC value
byte pLen       = 0;                                                                            // storing CANbus message payload length
byte pData      = 0;                                                                            // storing CANbus message payload data byte
byte txData[64] = {0};                                                                          // storing CANbus message payload data byte(s)
byte rVal[4]    = {0};                                                                          // sub-routines return values

// stores all CANbus message IDs {can be altered}
unsigned long msgId[MSGMAX] = {0x111,0x333,0x555,0x777,0x0,0x0};
                                    
chnCAN can1,can2,*ptrChn[2];                                                                    // CAN FD channel structs & pointers
boolean can1_bL = 1;                                                                            // enable/disable for CAN FD chn 1 (change to 0/1 to disable/enable)
boolean can2_bL = 1;                                                                            // enable/disable for CAN FD chn 2 (change to 0/1 to disable/enable)

// Setup Function ***************************************************************************************************************************************************************//
void setup() {
  Serial.begin(115200);                                                                         // setup serial UART 
  while (!Serial){}                                                                             // ** pro micro only **
  
  if(can1_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[0] = mcp2517_init(CANSPEED_500,&can1,1,TXQ,FIFO1);                                     // setup CAN FD chn 1 for CAN FD & uses TXQ for transmits & FIFO 1 as Rx FIFO
    rVal[1] = mcp2517_fltr_setup(&can1,FIFO1,FLTRNUM0,FLTRIDX0,FLTREXID,0x000,0xFFFFFFF);       // setup CAN FD chn 1 Rx filter to only recieve standard or extended msg ID 0x000
  }
  if(can2_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[2] = mcp2517_init(CANSPEED_500,&can2,2,TXQ,FIFO1);                                     // setup CAN FD chn 2 for CAN FD & uses TXQ for transmits & uses FIFO 1 as Rx FIFO
    rVal[3] = mcp2517_fltr_setup(&can2,FIFO1,FLTRNUM0,FLTRIDX0,FLTREXID,0x000,0xFFFFFFF);       // setup CAN FD chn 2 Rx filter to only recieve standard or extended msg ID 0x000
  }
  if(!rVal[0] && !rVal[1] && !rVal[2] && !rVal[3] && (can1_bL || can2_bL)){                     // check that setup functions were successfull & at least one channel is enabled
      ptrChn[0] = &can1;                                                                        // assign chnCAN pointer
      ptrChn[1] = &can2;                                                                        // assign chnCAN pointer
  }
  else{                                                                                         // 1 or more CAN channels failed to initialize
      Serial.println("Failed to configure CAN channels 1 & 2");                                 // format/print failure diagnostic message
      Serial.println("------------------------------------------------");                       // format/print seperator
      if(can1_bL) Serial.println("CAN1 channel status     = Enabled");                          // format/print can1_bL=1 decoded value
      else          Serial.println("CAN1 channel status     = Disabled");                       // format/print can1_bL=0 decoded value
      Serial.print("CAN1 init status code   = ");Serial.println(rVal[0]);                       // format/print channel 1 setup failure code
      Serial.print("CAN1 filter status code = ");Serial.println(rVal[1]);                       // format/print channel 1 filter setup failure code
      Serial.println("------------------------------------------------");                       // format/print seperator
      if(can2_bL) Serial.println("CAN2 channel status     = Enabled");                          // format/print can1_bL=1 decoded value
      else          Serial.println("CAN2 channel status     = Disabled");                       // format/print can1_bL=0 decoded value
      Serial.print("CAN2 init status code   = ");Serial.println(rVal[2]);                       // format/print channel 2 setup failure code
      Serial.print("CAN2 filter status code = ");Serial.println(rVal[3]);                       // format/print channel 2 filter setup failure code
      while(1){}                                                                                // wait here forever
  }
}

// Main Loop ********************************************************************************************************************************************************************//
void loop(){                                                                                    // system main
  j_u8 = 0;                                                                                     // reset to zero
  delay(20);                                                                                    // wait to achieve 100ms refresh rate
  
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  bfIde = IDE;                                                                                  // extended id
  bfFdf = FDF;                                                                                  // CAN FD frame
  bfBrs = BRS;                                                                                  // bit rate switching enabled
  pLen  = 24;                                                                                   // 24 byte payload
  txData[0] += 1;                                                                               // increment byte
  Serial.print("Transmitting message ID: 0x");                                                  // format/print label
  Serial.println(msgId[j_u8],HEX);                                                              // format/print msg ID
  mcp2517_msg_write(&can1,msgId[j_u8++],bfIde,bfFdf,bfBrs,!RTR,pLen,txData);                    // populate channel struct with necessary transmit info
  mcp2517_write_memory(TXQ,&can1);                                                              // write the Tx message info to MCP2517 TXQ
  mcp2517_start_transmit(TXQ,&can1);                                                            // initiate a transmit onto the CANbus network
  delay(500);                                                                                   // wait for message to transmit & screen updating
  if(DBG_BL){                                                                                   // debug flag set to 1
    if(mcp2517_check_message(&can2)){                                                           // check if current chnCAN has recieved a msg
      mcp2517_read_memory(FIFO1,&can2);                                                         // read current chnCAN message stored in FIFO1
      Serial.print("Recieved message ID: 0x");                                                  // format/print timestamp
      Serial.println(mcp2517_id_calc(&can2),HEX);                                               // format/print ID field
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  bfIde = !IDE;                                                                                 // standard id
  bfFdf = FDF;                                                                                  // CAN FD frame
  bfBrs = !BRS;                                                                                 // bit rate switching disabled
  pLen  = 8;                                                                                    // 8 byte payload
  txData[1] += 2;                                                                               // increment byte
  Serial.print("Transmitting message ID: 0x");                                                  // format/print label
  Serial.println(msgId[j_u8],HEX);                                                              // format/print msg ID
  mcp2517_msg_write(&can1,msgId[j_u8++],bfIde,bfFdf,bfBrs,!RTR,pLen,txData);                    // populate channel struct with necessary transmit info
  mcp2517_write_memory(TXQ,&can1);                                                              // write the Tx message info to MCP2517 TXQ
  mcp2517_start_transmit(TXQ,&can1);                                                            // initiate a transmit onto the CANbus network
  delay(500);                                                                                   // wait for message to transmit & screen updating
  if(DBG_BL){                                                                                   // debug flag set to 1
    if(mcp2517_check_message(&can2)){                                                           // check if current chnCAN has recieved a msg
      mcp2517_read_memory(FIFO1,&can2);                                                         // read current chnCAN message stored in FIFO1
      Serial.print("Recieved message ID: 0x");                                                  // format/print timestamp
      Serial.println(mcp2517_id_calc(&can2),HEX);                                               // format/print ID field
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  bfIde = IDE;                                                                                  // extended id
  bfFdf = !FDF;                                                                                 // CAN 2.0 frame
  bfBrs = !BRS;                                                                                 // bit rate switching disabled
  pLen  = 6;                                                                                    // 6 byte payload
  txData[2] += 4;                                                                               // increment byte
  Serial.print("Transmitting message ID: 0x");                                                  // format/print label
  Serial.println(msgId[j_u8],HEX);                                                              // format/print msg ID
  mcp2517_msg_write(&can1,msgId[j_u8++],bfIde,bfFdf,bfBrs,!RTR,pLen,txData);                    // populate channel struct with necessary transmit info
  mcp2517_write_memory(TXQ,&can1);                                                              // write the Tx message info to MCP2517 TXQ
  mcp2517_start_transmit(TXQ,&can1);                                                            // initiate a transmit onto the CANbus network
  delay(500);                                                                                   // wait for message to transmit & screen updating
  if(DBG_BL){                                                                                   // debug flag set to 1
    if(mcp2517_check_message(&can2)){                                                           // check if current chnCAN has recieved a msg
      mcp2517_read_memory(FIFO1,&can2);                                                         // read current chnCAN message stored in FIFO1
      Serial.print("Recieved message ID: 0x");                                                  // format/print timestamp
      Serial.println(mcp2517_id_calc(&can2),HEX);                                               // format/print ID field
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  bfIde = !IDE;                                                                                 // standard id
  bfFdf = !FDF;                                                                                 // CAN 2.0 frame
  bfBrs = !BRS;                                                                                 // bit rate switching disabled
  pLen  = 4;                                                                                    // 4 byte payload
  txData[3] += 1;                                                                               // increment byte
  Serial.print("Transmitting message ID: 0x");                                                  // format/print label
  Serial.println(msgId[j_u8],HEX);                                                              // format/print msg ID
  mcp2517_msg_write(&can1,msgId[j_u8++],bfIde,bfFdf,bfBrs,!RTR,pLen,txData);                    // populate channel struct with necessary transmit info
  mcp2517_write_memory(TXQ,&can1);                                                              // write the Tx message info to MCP2517 TXQ
  mcp2517_start_transmit(TXQ,&can1);                                                            // initiate a transmit onto the CANbus network
  delay(500);                                                                                   // wait for message to transmit & screen updating
  if(DBG_BL){                                                                                   // debug flag set to 1
    if(mcp2517_check_message(&can2)){                                                           // check if current chnCAN has recieved a msg
      mcp2517_read_memory(FIFO1,&can2);                                                         // read current chnCAN message stored in FIFO1
      Serial.print("Recieved message ID: 0x");                                                  // format/print timestamp
      Serial.println(mcp2517_id_calc(&can2),HEX);                                               // format/print ID field
    }
  }
}
