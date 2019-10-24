/************************************************************************
CAN FD Read Demo for the QBcircuits CAN FD Shield. 

Written by BrotherQ. 
Original tutorial available here: http://www.qbcircuits.com

Distributed as-is; no warranty is given.

Demo summary:
This demo originally is setup to configure both CAN FD channels 1 & 2
to read any CAN 2.0/CAN FD messages on the monitored CANbus network.
*************************************************************************/
#include <qb_mcp251xfd_defs.h>
#include <qb_mcp251xfd.h>

// Global Variables *************************************************************************************************************************************************************//
byte i_u8       = 0;                                                                            // array indexing
byte j_u8       = 0;                                                                            // array indexing
byte min_u8     = 0;                                                                            // array indexing
byte max_u8     = 0;                                                                            // array indexing
byte mode_u8    = 0;                                                                            // system state machine control
byte rVal[4]    = {0};                                                                          // sub-routines return values

chnCAN can1,can2,*ptrChn[2];                                                                    // CAN FD channel structs & pointers
boolean can1_bL = 1;                                                                            // enable/disable for CAN FD chn 1 (change to 0/1 to disable/enable)
boolean can2_bL = 1;                                                                            // enable/disable for CAN FD chn 2 (change to 0/1 to disable/enable)
unsigned long cntOvFlw_uL[2] = {0};                                                             // #of times timer based counter of a CAN FD channel has overflowed

// Setup Function ***************************************************************************************************************************************************************//
void setup() {
  Serial.begin(115200);                                                                         // setup serial UART 
  while (!Serial){}                                                                             // ** pro micro only **
  
  if(can1_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[0] = mcp251xfd_init(CANSPEED_500,&can1,1,TXQ,FIFO1);                                   // setup CAN FD chn 1 for CAN FD & uses TXQ for transmits & FIFO 1 as Rx FIFO
    rVal[1] = mcp251xfd_fltr_setup(&can1,FIFO1,FLTRNUM0,FLTRIDX0,FLTRBOTH,0x000,0x000);         // setup CAN FD chn 1 Rx filter (*ptrChn,bufIdx,fltrNum,fltrIdx,fltrType,msgId,mskId)
  }
  if(can2_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[2] = mcp251xfd_init(CANSPEED_500,&can2,2,TXQ,FIFO1);                                   // setup CAN FD chn 2 for CAN FD & uses TXQ for transmits & uses FIFO 1 as Rx FIFO
    rVal[3] = mcp251xfd_fltr_setup(&can2,FIFO1,FLTRNUM0,FLTRIDX0,FLTRBOTH,0x000,0x000);         // setup CAN FD chn 2 Rx filter (*ptrChn,bufIdx,fltrNum,fltrIdx,fltrType,msgId,mskId)
  }
  if(!rVal[0] && !rVal[1] && !rVal[2] && !rVal[3] && (can1_bL || can2_bL)){                     // check that setup functions were successfull & at least one channel is enabled
    ptrChn[0] = &can1;                                                                          // assign chnCAN pointer
    ptrChn[1] = &can2;                                                                          // assign chnCAN pointer
    min_u8 = (1 >> can1_bL);                                                                    // calculate min array index for looping thru chnCAN pointers
    max_u8 = (1 << can2_bL);                                                                    // calculate max array index for looping thru chnCAN pointers

    Serial.println("[s]   = Start/Stop Logging");                                               // hotkeys set 1
    Serial.println();                                                                           // format/print new line
    Serial.print("Channel,Time(s),#of overflows,#of (1/40MHz) periods,ID,IDE,FDF,BRS,RTR,DLC,Length,Data");     // format/print .csv header
    Serial.println();                                                                           // format/print new line
  }
  else{                                                                                         // 1 or more CAN channels failed to initialize
    Serial.println("Failed to configure CAN channels 1 & 2");                                   // format/print failure diagnostic message
    Serial.println("------------------------------------------------");                         // format/print seperator
    if(can1_bL) Serial.println("CAN1 channel status     = Enabled");                            // format/print can1_bL=1 decoded value
    else          Serial.println("CAN1 channel status     = Disabled");                         // format/print can1_bL=0 decoded value
    Serial.print("CAN1 init status code   = ");Serial.println(rVal[0]);                         // format/print channel 1 setup failure code
    Serial.print("CAN1 filter status code = ");Serial.println(rVal[1]);                         // format/print channel 1 filter setup failure code
    Serial.println("------------------------------------------------");                         // format/print seperator
    if(can2_bL) Serial.println("CAN2 channel status     = Enabled");                            // format/print can2_bL=1 decoded value
    else          Serial.println("CAN2 channel status     = Disabled");                         // format/print can2_bL=0 decoded value
    Serial.print("CAN2 init status code   = ");Serial.println(rVal[2]);                         // format/print channel 2 setup failure code
    Serial.print("CAN2 filter status code = ");Serial.println(rVal[3]);                         // format/print channel 2 filter setup failure code
    while(1){}                                                                                  // wait here forever
  }
}

// Main Loop ********************************************************************************************************************************************************************//
void loop(){                                                                                        // system main
  if(Serial.available()){                                                                           // recieved user selected option (take 1st char recieved)
    rVal[0] = Serial.read();                                                                        // 1st option starts at ASCII "0" so subtract 48 from recieved char to get numeric value
    if(rVal[0]=='s' || rVal[0]=='S'){                                                               // check if recieve start/stop command
      mode_u8 = !mode_u8;                                                                           // go to state1
    }
    while(Serial.available()){rVal[0] = Serial.read();}                                             // clear out serial Rx buffer
  }
  
  if(mode_u8){                                                                                      // stateX [start logging & wait for user to stop logging]
    for(j_u8=min_u8;j_u8<max_u8;j_u8++){                                                            // loop thru all chnCAN pointers
      if(mcp251xfd_check_message(ptrChn[j_u8])){                                                    // check if current chnCAN has recieved a msg
        
        mcp251xfd_read_register(ADDR_C1INT,ptrChn[j_u8],0);                                         // read MCP2517 C1INT register (byte 0) for TBCIF
        if((ptrChn[j_u8]->regRd[0] >> TBCIF) & 1){                                                  // check if TBCIF is set (indicates if MCP2517 timer has overflowed)
          cntOvFlw_uL[j_u8]++;                                                                      // increment timer overflow counter
          for(i_u8=0;i_u8<CSCNT;i_u8++);                                                            // wait some time before toggling MCP2517 chip select
          ptrChn[j_u8]->regWr[0] = ptrChn[j_u8]->regRd[0] & ~(1 << TBCIF);                          // clear TCBIF field
          mcp251xfd_write_register(ADDR_C1INT,ptrChn[j_u8],0);                                      // write MCP2517 C1INT register (byte 0) to clear TBCIF
        }
        for(i_u8=0;i_u8<CSCNT;i_u8++);                                                              // wait some time before toggling MCP2517 chip select
        mcp251xfd_read_memory(FIFO1,ptrChn[j_u8]);                                                  // read current chnCAN message stored in FIFO1 

        Serial.print("CAN"); Serial.print(ptrChn[j_u8]->chnNum);Serial.print(",");                  // format/print chn#

        Serial.print("=1/(40e6)*(2^32*");  Serial.print(cntOvFlw_uL[j_u8]); Serial.print("+");      // format/print time absolute formula
        Serial.print(ptrChn[j_u8]->msg.tStamp); Serial.print("),");                                 // format/print time absolute formula (cont.)          
        
        Serial.print(cntOvFlw_uL[j_u8]);  Serial.print(",");                                        // format/print #of timer overflows
        Serial.print(ptrChn[j_u8]->msg.tStamp); Serial.print(",0x");                                // format/print timestamp
        
        Serial.print(mcp251xfd_id_calc(ptrChn[j_u8]),HEX);  Serial.print(",");                      // format/print ID field
        Serial.print(ptrChn[j_u8]->msg.ide);  Serial.print(",");                                    // format/print IDE field
        Serial.print(ptrChn[j_u8]->msg.fdf);  Serial.print(",");                                    // format/print FDF field
        Serial.print(ptrChn[j_u8]->msg.brs);  Serial.print(",");                                    // format/print BRS field
        Serial.print(ptrChn[j_u8]->msg.rtr);  Serial.print(",");                                    // format/print RTR field
        Serial.print(ptrChn[j_u8]->msg.dlc);  Serial.print(",");                                    // format/print DLC field
        Serial.print(ptrChn[j_u8]->msg.pLen); Serial.print(",");                                    // format/print payload length field
        for(i_u8=0;i_u8<ptrChn[j_u8]->msg.pLen;i_u8++){                                             // loop thru all payoad bytes
          if(ptrChn[j_u8]->msg.rxData[i_u8]<16)                                                     // check if need to add leading HEX zero
              Serial.print("0");                                                                    // format/print leading HEX zero
              
          Serial.print(ptrChn[j_u8]->msg.rxData[i_u8],HEX);                                         // format/print payload byte
          Serial.print(" ");                                                                        // format/print space
        }
        Serial.println();                                                                           // format/print new line
      }
    }
  }
}
