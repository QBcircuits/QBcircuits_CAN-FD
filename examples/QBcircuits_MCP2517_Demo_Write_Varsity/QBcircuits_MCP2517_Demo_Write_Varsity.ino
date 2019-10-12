/************************************************************************
CAN FD Write Demo for the QBcircuits CAN FD Shield. 

Written by BrotherQ. 
Original tutorial available here: http://www.qbcircuits.com

Distributed as-is; no warranty is given.

Demo summary:
This demo originally is setup to configure both CAN FD channels 1 & 2
to write demo CAN 2.0/CAN FD dummy messages on the monitored CANbus network
and recieve only extended ID 0.
*************************************************************************/
#include <qb_mcp2517_defs.h>
#include <qb_mcp2517.h>

// Global Variables *************************************************************************************************************************************************************//
#define MSGMAX  6                                                                               // array sizing {can be altered} ** must be >0
#define CAN1    0                                                                               // system critical {do not change}
#define CAN2    1                                                                               // system critical {do not change}

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
unsigned long msgId[MSGMAX] = {0x1111,0x0222,0x0333,0x0444,0x0555,0x9999};

// stores all CANbus message info   {chn#(0/1),ide(0/1),fdf(0/1),brs(0/1),#of payload bytes(0-n),payload bytes 0-20} {can be altered}
byte msgData[MSGMAX][25] = {{CAN1, IDE, FDF, BRS,20,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14},
                            {CAN1, IDE, FDF, BRS,16,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x10,0x11,0x12,0x13,0x14},
                            {CAN1,!IDE, FDF, BRS,12,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x10,0x11,0x12,0x13,0x14},
                            {CAN2,!IDE, FDF,!BRS, 8,0x31,0x32,0x33,0x34,0x35,0x06,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x10,0x11,0x12,0x13,0x14},
                            {CAN2,!IDE,!FDF,!BRS, 8,0x41,0x42,0x43,0x44,0x45,0x06,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x10,0x11,0x12,0x13,0x14},
                            {CAN2, IDE,!FDF,!BRS, 8,0x51,0x52,0x53,0x54,0x55,0x06,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,0x10,0x11,0x12,0x13,0x14}};
                                    
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
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if(mode_u8 == 0){                                                                             // state0 [Prompt user of available options]
    Serial.println("------------------------------------------------");                         // format/print seperator
    Serial.println("List of defined messages to transmit:");                                    // format/print label
    for(j_u8=0;j_u8<MSGMAX;j_u8++){                                                             // loop thru all defined CANbus messages
        chnIdx = msgData[j_u8][0];                                                              // store current msg channel index
        bfIde = msgData[j_u8][1];                                                               // store current msg IDE value
        bfFdf = msgData[j_u8][2];                                                               // store current msg FDF value
        bfBrs = msgData[j_u8][3];                                                               // store current msg BRS value
        pLen   = msgData[j_u8][4];                                                              // store current msg payload length
        bfDlc = mcp2517_dlc_payload(bfFdf,pLen);                                                // calculate and store current msg DLC value
        Serial.write(48+j_u8);Serial.print(") ");printHex(msgId[j_u8],8);                       // print current msg ID and option#
        Serial.print(" [CAN");Serial.print(chnIdx+1);Serial.print("]");                         // print msg channel index
        
        if(bfIde)   Serial.print(" [Extended]");                                                // print msg IDE=1 decoded value
        else        Serial.print(" [Standard]");                                                // print msg IDE=0 decoded value

        if(bfFdf)   Serial.print(" [CAN FD ]");                                                 // print msg FDF=1 decoded value
        else        Serial.print(" [CAN 2.0]");                                                 // print msg FDF=0 decoded value

        Serial.print(" [BRS = ");Serial.print(bfBrs);Serial.print("]");                         // print msg BRS value
        
        Serial.print(" [DLC = ");                                                               // print msg DLC value
        if(bfDlc<10) Serial.print(" ");                                                         // print msg DLC value (cont.)
        Serial.print(bfDlc);Serial.print("]");                                                  // print msg DLC value (cont.)

        Serial.print(" [Length = ");                                                            // print msg payload length
        if(pLen<10) Serial.print(" ");                                                          // print msg payload length (cont.)
        Serial.print(pLen);Serial.print("]");                                                   // print msg payload length (cont.)

        Serial.print(" [");                                                                     // print msg payload bytes
        for(i_u8=0;i_u8<pLen;i_u8++){                                                           // print msg payload bytes (cont.)
          pData = msgData[j_u8][5 + i_u8];                                                      // print msg payload bytes (cont.)
          if(pData<16) Serial.print("0");                                                       // print msg payload bytes (cont.)
          Serial.print(pData,HEX);                                                              // print msg payload bytes (cont.)
          if(i_u8 != (pLen-1)) Serial.print(" ");                                               // print msg payload bytes (cont.)
        }
        Serial.print("]");                                                                      // print msg payload bytes (cont.)
        Serial.println();                                                                       // print msg payload bytes (cont.)
    }
    Serial.println();                                                                           // print line feed
    Serial.print("Select message to transmit: ");                                               // prompt user for selection
    mode_u8 = 1;                                                                                // go to state1
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == 1){                                                                        // state1 [wait for user to make a selection]
    if(Serial.available()){                                                                     // recieved user selected option (take 1st char recieved)
      rVal[0] = Serial.read() - 48;                                                             // 1st option starts at ASCII "0" so subtract 48 from recieved char to get numeric value
      rVal[0] = (rVal[0] > MSGMAX-1) ? MSGMAX-1 : rVal[0];                                      // insure char recieved not greater than array max & cap at array max if greater
      Serial.write(rVal[0] + 48);                                                               // print the user's selection
      mode_u8 = 2;                                                                              // go to state2
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == 2){                                                                        // state2 [transmit the user selected message]
    idx    = rVal[0];                                                                           // store the index selected by user
    chnIdx = msgData[idx][0];                                                                   // store current msg channel index
    bfIde = msgData[idx][1];                                                                    // store current msg IDE value
    bfFdf = msgData[idx][2];                                                                    // store current msg FDF value
    bfBrs = msgData[idx][3];                                                                    // store current msg BRS value
    pLen   = msgData[idx][4];                                                                   // store current msg payload length
    bfDlc = mcp2517_dlc_payload(bfFdf,pLen);                                                    // calculate and store current msg DLC value    
    if((can1_bL && chnIdx==CAN1) || (can2_bL && chnIdx==CAN2)){                                 // check that channel index for the chosen message is enabled
      for(j_u8=0;j_u8<pLen;j_u8++){txData[j_u8]=msgData[idx][5+j_u8];}                          // copy selected msg payloads bytes into temporary buffer
      mcp2517_msg_write(ptrChn[chnIdx],msgId[idx],bfIde,bfFdf,bfBrs,!RTR,pLen,txData);          // populate channel struct with necessary transmit info
      mcp2517_write_memory(TXQ,ptrChn[chnIdx]);                                                 // write the Tx message info to MCP2517 TXQ
      mcp2517_start_transmit(TXQ,ptrChn[chnIdx]);                                               // initiate a transmit onto the CANbus network
      while(Serial.available()){                                                                // clear out any excess characters before restarting the state machine
        rVal[0] = Serial.read();                                                                // read excess chars into dummy variable
      }
      mode_u8 = 3;                                                                              // go to state3
    }
    else{                                                                                       // channel index for the chosen message is disabled
      Serial.println(" - Failure [Channel utilized for chosen message is disabled]");           // print failure message
      mode_u8 = 0;                                                                              // go to state0
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == 3){                                                                        // state3 [debug enable/disable state]
//    mode_u8 = 4;                                                                                // uncomment this line to go to debug state
    if(mode_u8==3){                                                                             // check mode variable is unchanged
      mode_u8 = 5;                                                                              // go to state5
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == 4){                                                                        // state4 [debug state only if CAN1 & CAN2 are on same network & filtering on chosen msg(s)]
    Serial.println(" #GoLong");                                                                 // print acknowledgement
    delay(100);                                                                                 // wait to allow opposite channel to recieve the message
    if(mcp2517_check_message(ptrChn[!chnIdx])){                                                 // check if opposite CAN channel has recieved a msg
      mcp2517_read_memory(FIFO1,ptrChn[!chnIdx]);                                               // read the msg recieved by the opposite CAN channel 
      Serial.print("Recieved message: 0x");                                                     // format/print label
      Serial.println(mcp2517_id_calc(ptrChn[!chnIdx]),HEX);                                     // format/print ID field
    }
    else                                                                                        // no message read from opposite channel
      Serial.println("No message recieved #GoTryAgain");                                        // format/print label
    
    Serial.println();                                                                           // print line feed
    mode_u8 = 0;                                                                                // go to state0
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == 5){                                                                        // state5 [success/reset state]
    Serial.println(" #GoLong");                                                                 // print acknowledgement
    Serial.println();                                                                           // print line feed
    mode_u8 = 0;                                                                                // go to state0
  }
}

void printHex(int num, int precision) {
     char tmp[16];
     char format[20];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}
