/************************************************************************
CAN FD Write Demo for the QBcircuits CAN FD Shield. 

Written by BrotherQ. 
Original tutorial available here: http://www.qbcircuits.com

Distributed as-is; no warranty is given.

Demo summary:
This demo originally is setup to configure CAN FD channel 1
to send OBD OBD service 0x01 or 0x04 messages to ECUs 0x7E0-0x7E7 
on the monitored CANbus network.
*************************************************************************/
#include <qb_mcp251xfd_defs.h>
#include <qb_mcp251xfd.h>
#include <qb_obd2.h>

// Global Variables *************************************************************************************************************************************************************//
#define STATE_POLL            0                                                                 // intial state
#define STATE_INPUT           1                                                                 // user input state
#define STATE_POLLING         2                                                                 // polling for ECUs state
#define STATE_ECU             3                                                                 // online ECUs select state
#define STATE_SERVICE         4                                                                 // OBD2 services to perform
#define STATE_SERV_S1         5                                                                 // OBD2 service 0x01 prompt header
#define STATE_SERV_S1a        6                                                                 // OBD2 service 0x01 prompt
#define STATE_SERVING_S1      7                                                                 // OBD2 service 0x01 passive state
#define STATE_SERVING_S1a     8                                                                 // OBD2 service 0x01 active state
#define STATE_SERVING_S4      9                                                                 // OBD2 service 0x04 running state

#define ERR_NONE              0                                                                 // ECU no error
#define ERR_RSPNO             1                                                                 // ECU no response error
#define ERR_RSPNEG            2                                                                 // ECU negative response error

#define PROMPT_NONE           0                                                                 // prompt default
#define PROMPT_GHME           1                                                                 // prompt for "#GoHome"
#define PROMPT_GTRY           2                                                                 // prompt for "#GoTryAgain"
#define PROMPT_GPOL           3                                                                 // prompt for "#GoPolling"
#define PROMPT_GLNG           4                                                                 // prompt for "#GoLong"
#define PROMPT_GBCK           5                                                                 // prompt for "#GoBack"
#define PROMPT_GLNE           6                                                                 // prompt for "#GoToLine*"

byte i_u8               = 0;                                                                    // array indexing
byte j_u8               = 0;                                                                    // array indexing
byte mode_u8            = STATE_POLL;                                                           // system state machine control
byte modePrev_u8        = STATE_POLL;                                                           // system state machine control
byte modeNext_u8        = STATE_POLL;                                                           // system state machine control
byte chnIdx             = 0;                                                                    // storing CANbus message channel index
byte txData[8]          = {0};                                                                  // storing CANbus message payload data byte(s)
byte rVal[4]            = {0};                                                                  // sub-routines return values

byte prompt_u8          = PROMPT_NONE;                                                          // trigger for serial acknowledgement prompts
byte odb2serv[2]        = {0x01, 0x04};                                                         // OBD2 services allowed
byte s1PidIdx_u8        = 0;                                                                    // OBD2 service 0x01 PID index 
byte s1PgIdx_u8         = 0;                                                                    // OBD2 service 0x01 page index (page of supported PIDs)
byte s1PgMax_u8         = 4;                                                                    // OBD2 service 0x01 page index (page of supported PIDs)
byte ecuError_u8        = 0;                                                                    // generic ECU error flag
byte ecuCnt_u8          = 0;                                                                    // stores max # of ECUs present on the network
byte ecuIdx[8]          = {0};
byte s1Idx[16][2]       = {0};                                                                  // {ASCII index, OBD2 s1 PID}
word ecuIdVal           = 0;
word ecuId[8]           = {0x7E0,0x7E1,0x7E2,0x7E3,0x7E4,0x7E5,0x7E6,0x7E7};                    // stores all OBD2 IDs
word ecuOnline[8]       = {0};                                                                  // stores all OBD2 IDs that are present on the network
word s1PidSpt_u16       = 0;                                                                    // temp storage for service 0x01 enabled PID codes

// stores all service 1 PID payload lengths
byte s1PidPlen[11][16] = {{4,2,2,1,1,1,1,1,1,1,1,2,1,1,1,2},
                          {1,1,1,2,2,2,2,2,2,2,2,1,1,1,2,4},
                          {2,2,2,4,4,4,4,4,4,4,4,1,1,1,1,1},
                          {2,2,1,4,4,4,4,4,4,4,4,2,2,2,2,4},
                          {4,2,2,2,1,1,1,1,1,1,1,1,2,2,4,4},
                          {1,1,2,2,2,2,2,2,2,1,1,1,2,2,1,4},
                          {1,1,2,5,2,5,3,7,7,5,5,5,6,5,3,9},
                          {5,5,5,5,7,7,5,9,9,7,7,9,1,1,13,4},
                          {21,21,5,1,10,5,5,13,41,41,7,16,1,1,5,3},
                          {5,2,3,12,9,9,6,4,17,4,2,9,4,9,2,9},
                          {9,2,9,4,4,4,0,0,0,0,0,0,0,0,0,0}};

char promptBuf[4][34] = {"---------------------------------",
                          "Enter selection: ",
                          "[.]   = Go back to poll page",
                          "[-]   = Go back to previous page",
                         };
                              
chnCAN can1,can2,*ptrChn[2];                                                                    // CAN FD channel structs & pointers
boolean can1_bL = 1;                                                                            // enable/disable for CAN FD chn 1 (change to 0/1 to disable/enable)
boolean can2_bL = 0;                                                                            // enable/disable for CAN FD chn 2 (change to 0/1 to disable/enable)

// Setup Function ***************************************************************************************************************************************************************//
void setup() {
  char rMsg[5]     = "CAN*";                                                                    // used printing out correct CAN channel msg
  Serial.begin(115200);                                                                         // setup serial UART 
  while (!Serial){}                                                                             // ** pro micro only **

  can2_bL = !can1_bL;                                                                           // insure only 1 channel is enabled
  if(can1_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[0] = mcp251xfd_init(CANSPEED_500,&can1,1,TXQ,FIFO1);                                   // setup CAN FD chn 1 for CAN FD & uses TXQ for transmits & FIFO 1 as Rx FIFO
    rVal[1] = mcp251xfd_fltr_setup(&can1,FIFO1,FLTRNUM0,FLTRIDX0,FLTRSID,0x7E8,0xFF8);          // setup CAN FD chn 1 Rx filter to only recieve standard msg IDs 0x7E8-0x7EF
  }
  if(can2_bL){                                                                                  // CAN FD chn1 is enabled
    rVal[2] = mcp251xfd_init(CANSPEED_500,&can2,2,TXQ,FIFO1);                                   // setup CAN FD chn 2 for CAN FD & uses TXQ for transmits & uses FIFO 1 as Rx FIFO
    rVal[3] = mcp251xfd_fltr_setup(&can2,FIFO1,FLTRNUM0,FLTRIDX0,FLTRSID,0x7E8,0xFF8);          // setup CAN FD chn 2 Rx filter to only recieve standard msg IDs 0x7E8-0x7EF
  }
  if(!rVal[0] && !rVal[1] && !rVal[2] && !rVal[3] && (can1_bL || can2_bL)){                     // check that setup functions were successfull & at least one channel is enabled
      ptrChn[0] = &can1;                                                                        // assign chnCAN pointer
      ptrChn[1] = &can2;                                                                        // assign chnCAN pointer
      chnIdx = (1 >> can1_bL);                                                                  // set index to correct chnCan pointer
      rMsg[3] = '1' + chnIdx;                                                                   // set to "CAN1" or "CAN2"
  }
  else{                                                                                         // 1 or more CAN channels failed to initialize
      Serial.println("Failed to configure CAN channel");                                        // format/print failure diagnostic message
      Serial.println(promptBuf[0]);                                                             // format/print seperator

      Serial.print(rMsg);                                                                       // format/print "CANn" text
      if(can1_bL)   Serial.println(" channel status     = Enabled");                            // format/print can1_bL=1 decoded value
      else          Serial.println(" channel status     = Disabled");                           // format/print can1_bL=0 decoded value

      Serial.print(rMsg);                                                                       // format/print "CANn" text
      Serial.print(" init status code   = ");Serial.println(rVal[chnIdx+0+can2_bL]);            // format/print channel 1 setup failure code
      
      Serial.print(rMsg);                                                                       // format/print "CANn" text
      Serial.print(" filter status code = ");Serial.println(rVal[chnIdx+1+can2_bL]);            // format/print channel 1 filter setup failure code
      while(1){}                                                                                // wait here forever
  }
}

// Main Loop ********************************************************************************************************************************************************************//
void loop(){                                                                                    // system main
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if(mode_u8 == STATE_POLL){                                                                    // stateX [Prompt user to start polling for ECUs]
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[0]);                                                               // format/print seperator
    Serial.println("Command list:");                                                            // format/print label
    Serial.println("p) Poll for ECUs");                                                         // format/print label
    Serial.println();                                                                           // format/print label
    Serial.println(promptBuf[1]);                                                               // format/print label

    for(j_u8=0;j_u8<8;j_u8++){ecuOnline[j_u8] = 0;}                                             // reset ECU online status flags
    s1PgIdx_u8 = 0;                                                                             // reset page index
    s1PgMax_u8 = 0;                                                                             // reset page index max value
    ecuError_u8 = ERR_NONE;                                                                     // reset ECU error flag
    ecuCnt_u8 = 0;                                                                              // reset online ECU counter
    modePrev_u8 = mode_u8;                                                                      // update previous to state to current
    mode_u8 = STATE_INPUT;                                                                      // go to stateX [user input]
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_INPUT){                                                              // stateX [wait for user input]
    modeNext_u8 = mode_u8;                                                                      // set next mode to current to prevent premature exit from this state
    
    if(Serial.available()){                                                                     // recieved user selected option (take 1st char recieved)
      rVal[0] = Serial.read();                                                                  // 1st option starts at ASCII "0" so subtract 48 from recieved char to get numeric value
      if(rVal[0] < 32){rVal[0] = ' ';}                                                          // replace non-printable chars with space char
      Serial.write(rVal[0]);                                                                    // echo back the selection
      delay(100);                                                                               // delay to recieve in any trailing chars LFs, CRs, etc.
      while(Serial.available()){rVal[1] = Serial.read();}                                       // clear out serial Rx buffer
    }
    else                                                                                        // no chars recieved
      rVal[0] = 0;                                                                              // keep at 0 to prevent following code execution
    
    if(rVal[0]){                                                                                // rVal[0] has a char value
      if(modePrev_u8 == STATE_POLL){                                                            // came from poll prompt state
        if((rVal[0] == 'p') || (rVal[0] == 'P')){                                               // valid option recieved
          prompt_u8 = PROMPT_GPOL;                                                              // update prompt flag
          modeNext_u8 = STATE_POLLING;                                                          // go to polling for ECUs state
        }
        else{                                                                                   // invalid option recieved
          prompt_u8 = PROMPT_GTRY;                                                              // update prompt flag
        }
      }
      else if(modePrev_u8 == STATE_ECU){                                                        // came from ECU select state
        if((rVal[0] == '.')){                                                                   // "." so go back to homepage
          prompt_u8 = PROMPT_GHME;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }        
        else if((rVal[0] == '-')){                                                              // "-" so go back to poll prompt state
          prompt_u8 = PROMPT_GBCK;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }
        else if((rVal[0] >= '0') && (rVal[0] <= ('0' + ecuCnt_u8 - 1))){                        // valid option recieved  
          prompt_u8 = PROMPT_GLNG;                                                              // update prompt flag
          ecuIdVal = ecuId[ecuIdx[rVal[0]-48]];                                                 // save selected ECU ID
          modeNext_u8 = STATE_SERVICE;                                                          // go to polling for ECUs state
        }
        else{                                                                                   // invalid option
          prompt_u8 = PROMPT_GTRY;                                                              // update prompt flag
        }
      }
      else if(modePrev_u8 == STATE_SERVICE){                                                    // came from OBD2 service state
        if((rVal[0] == '.')){                                                                   // "." so go back to poll prompt state
          prompt_u8 = PROMPT_GHME;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }
        else if((rVal[0] == '-')){                                                              // "-" so go back to poll prompt state
          prompt_u8 = PROMPT_GBCK;                                                              // update prompt flag
          modeNext_u8 = STATE_ECU;                                                              // go to ECU prompt state
        }
        else if((rVal[0] >= '0') && (rVal[0] <= '1')){                                          // valid option recieved
          prompt_u8 = PROMPT_GLNG;                                                              // update prompt flag
          if(rVal[0] == '0')
            modeNext_u8 = STATE_SERV_S1;                                                        // go to polling for ECUs state
          else                                                                                  // OBD2 service 0x04 prompt
            modeNext_u8 = STATE_SERVING_S4;                                                     // go to polling for ECUs state
        } 
        else{                                                                                   // invalid option
          prompt_u8 = PROMPT_GTRY;                                                              // update prompt flag
        }
      }
      else if(modePrev_u8 == STATE_SERV_S1a){                                                   // came from OBD2 service state
        if((rVal[0] == '.')){                                                                   // "." so go back to poll prompt state
          prompt_u8 = PROMPT_GHME;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }
        else if((rVal[0] == '-')){                                                              // "-" so go back to poll prompt state
          prompt_u8 = PROMPT_GBCK;                                                              // update prompt flag
          modeNext_u8 = STATE_SERVICE;                                                          // go to OBD2 service state
        }
        else if((rVal[0] == '>')){                                                              // ">" so go to next service 0x01 PID page or 1st page
          if(s1PgIdx_u8 < s1PgMax_u8-1){
            s1PgIdx_u8++;
          }
          else{
            s1PgIdx_u8 = 0;
          }
          prompt_u8 = PROMPT_GLNE;                                                              // update prompt flag
          modeNext_u8 = STATE_SERV_S1a;                                                         // go to OBD2 service 0x01 state
        }
        else if((rVal[0] == '<')){                                                              // "<" so go to previous service 0x01 PID page or last page
          if(s1PgIdx_u8 > 0){
            s1PgIdx_u8--;
          }
          else{
            s1PgIdx_u8 = s1PgMax_u8-1;
          }
          prompt_u8 = PROMPT_GLNE;                                                              // update prompt flag
          modeNext_u8 = STATE_SERV_S1a;                                                         // go to OBD2 service 0x01 state
        }
        else if(((rVal[0] >= 'A') && (rVal[0] <= 'P')) || 
                ((rVal[0] >= 'a') && (rVal[0] <= 'p'))){                                        // valid option recieved
          for(j_u8=0;j_u8<16;j_u8++){
            if(s1Idx[j_u8][0] == rVal[0] || s1Idx[j_u8][0]+32 == rVal[0]){
              s1PidIdx_u8 = s1Idx[j_u8][1];
            }
          }
          if(!s1PidIdx_u8 || (s1PidIdx_u8 == 0xFF)){
            prompt_u8 = PROMPT_GTRY;                                                            // update prompt flag
          }
          else{
            prompt_u8 = PROMPT_GLNG;                                                            // update prompt flag
            modeNext_u8 = STATE_SERVING_S1;                                                     // go to OBD2 service 0x01 active state
          }
        }
        else{                                                                                   // invalid option
          prompt_u8 = PROMPT_GTRY;                                                              // update prompt flag
        }
      }
      else if(modePrev_u8 == STATE_SERVING_S1){                                                 // came from OBD2 service state
        if((rVal[0] == '.')){                                                                   // "." so go back to poll prompt state
          prompt_u8 = PROMPT_GHME;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }
        else if((rVal[0] == '-')){                                                              // "-" so go back to poll prompt state
          prompt_u8 = PROMPT_GBCK;                                                              // update prompt flag
          modeNext_u8 = STATE_SERV_S1;                                                          // go to OBD2 service state
        }
        else if((rVal[0] == 's') || (rVal[0] == 'S')){                                          // valid option
          prompt_u8 = PROMPT_GLNG;                                                              // update prompt flag
          modeNext_u8 = STATE_SERVING_S1a;                                                      // go to OBD2 service 0x01 active state
        }
        else{                                                                                   // invalid option recieved
          prompt_u8 = PROMPT_GTRY;                                                              // update prompt flag
        }
      }
      else if(modePrev_u8 == STATE_SERVING_S1a){                                                // came from OBD2 service state
        if((rVal[0] == '.')){                                                                   // "." so go back to poll prompt state
          prompt_u8 = PROMPT_GHME;                                                              // update prompt flag
          modeNext_u8 = STATE_POLL;                                                             // go to poll prompt state
        }
        else if((rVal[0] == '-')){                                                              // "-" so go back to poll prompt state
          prompt_u8 = PROMPT_GBCK;                                                              // update prompt flag
          modeNext_u8 = STATE_SERV_S1;                                                          // go to OBD2 service state
        }
        else if((rVal[0] == 's') || (rVal[0] == 'S')){                                          // valid option
          prompt_u8 = PROMPT_GLNG;                                                              // update prompt flag
          modeNext_u8 = STATE_SERVING_S1;                                                       // go to OBD2 service 0x01 passive state
        }
        else{                                                                                   // invalid option recieved
          modeNext_u8 = STATE_SERVING_S1a;                                                      // go to OBD2 service 0x01 active state
        }
      }
    }
    if(prompt_u8 != PROMPT_NONE){                                                               // need to printout prompt acknowledgements
      if(prompt_u8 == PROMPT_GTRY){                                                             // invalid option acknowledgement
        Serial.println(F(" #GoTryAgain"));                                                      // prompt that choice is invalid
        Serial.println(promptBuf[1]);                                                           // format/print label
      }
      else if(prompt_u8 == PROMPT_GPOL){                                                        // polling menu acknowledgement
        Serial.println(F(" #GoPolling"));                                                       // prompt that choice is valid
      }
      else if(prompt_u8 == PROMPT_GHME){                                                        // home menu acknowledgement
        Serial.println(F(" #GoHome"));                                                          // prompt that going back to homepage
      }
      else if(prompt_u8 == PROMPT_GLNG){                                                        // advance a menu acknowledgement
        Serial.println(F(" #GoLong"));                                                          // prompt that choice is valid
      }
      else if(prompt_u8 == PROMPT_GBCK){                                                        // backup a menu acknowledgement
        Serial.println(F(" #GoBack"));                                                          // prompt that going back to previous prompt
      }
      else if(prompt_u8 == PROMPT_GLNE){                                                        // OBD2 service 0x01 page acknowledgements
        Serial.print(F(" #GoToLine"));                                                          // prompt that going back to previous prompt
        Serial.println(s1PgIdx_u8+1);                                                           // prompt that going back to previous prompt
      }
      prompt_u8 = PROMPT_NONE;                                                                  // reset prompt flag
    }
    if(modeNext_u8 != mode_u8){                                                                 // mode change needed
      modePrev_u8 = mode_u8;                                                                    // update previous to state to current
      mode_u8 = modeNext_u8;                                                                    // go to polling for ECUs state
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_POLLING){                                                            // stateX [polling for ECUs present on the network]
    Serial.println();                                                                           // format/print blank line
    for(j_u8=0;j_u8<8;j_u8++){                                                                  // start polling thru all possible ECU IDs
      Serial.print("Checking ECU ID = 0x"); Serial.print(ecuId[j_u8],HEX);                      // prompt which ECU checking for 
      txData[0] = 2;                                                                            // #of additional data bytes
      txData[1] = 0x01;                                                                         // service code
      txData[2] = 0x00;                                                                         // PID
      mcp251xfd_msg_write(ptrChn[chnIdx],ecuId[j_u8],!IDE,!FDF,!BRS,!RTR,8,txData);             // populate channel struct with necessary transmit info
      mcp251xfd_write_memory(TXQ,ptrChn[chnIdx]);                                               // write the Tx message info to MCP2517 TXQ
      mcp251xfd_start_transmit(TXQ,ptrChn[chnIdx]);                                             // initiate a transmit onto the CANbus network
      delay(100);                                                                               // delay to allow currently polled ECU to respond
      if(mcp251xfd_check_message(ptrChn[chnIdx])){                                              // check if current chnCAN has recieved a msg
        Serial.print(" - Online");                                                              // format/print ECU is online
        mcp251xfd_read_memory(FIFO1,ptrChn[chnIdx]);                                            // read current chnCAN message stored in FIFO1
        if(ptrChn[chnIdx]->msg.rxData[1] == (txData[1] + 0x40)){                                // check for +response from ECU OBD2 service 0x01
          Serial.println(" [supports CAN OBD2]");                                               // format/print ECU supports OBD2 over CAN
          ecuOnline[j_u8] = 1;                                                                  // update ECU online status flag
          ecuCnt_u8++;                                                                          // increment #of online ECUs
        }
        else{                                                                                   // -response, or unknown response from ECU OBD2 service 0x01
          Serial.println(" [does not support CAN OBD2]");                                       // format/print ECU does not support OBD2 over CAN
          ecuOnline[j_u8] = 0;                                                                  // update ECU online status flag
        }
      }
      else{                                                                                     // no reply from ECU
        Serial.println(" - Offline");                                                           // format/print ECU is offline
      }
    }
    if(ecuCnt_u8){                                                                              // 1 or more ECUs online & CAN OBD2 compliant
      s1PgIdx_u8 = 0;                                                                           // reset service 0x01 page index
      modePrev_u8 = mode_u8;                                                                    // update previous to state to current
      mode_u8 = STATE_ECU;                                                                      // go to ECUs prompt state
    }
    else{                                                                                       // no ECUs or zero ECUs online that are CAN OBD2 compliant
      Serial.println(F("** No CAN OBD2 compliant ECUs found **"));                              // format/print user prompt
      modePrev_u8 = mode_u8;                                                                    // update previous to state to current
      mode_u8 = STATE_POLL;                                                                     // go to user ECU poll prompt state
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_ECU){                                                                // stateX [prompting user of online ECUs to select]
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[0]);                                                               // format/print seperator
    Serial.println(promptBuf[2]);                                                               // hotkeys set 1
    Serial.println(promptBuf[3]);                                                               // hotkeys set 2
    Serial.println(F("[0-n] = Selects online ECU"));                                            // hotkeys set 3
    Serial.println();                                                                           // format/print new line
    Serial.println(F("Online ECUs for diagnostics:"));                                          // format/print user prompt
    i_u8 = 0;                                                                                   // reset for algorithm usage
    for(j_u8=0;j_u8<8;j_u8++){                                                                  // start polling thru all possible ECUs online
      if(ecuOnline[j_u8]){                                                                      // check if current ECU is online
        Serial.write('0' + j_u8); Serial.print(") 0x"); Serial.println(ecuId[j_u8],HEX);        // format/print online ECU ID
        ecuIdx[i_u8++] = j_u8;                                                                  // save online ECU selection numerical index
      }
      if(i_u8 >= ecuCnt_u8){                                                                    // check if reached max #of online ECUs
        j_u8 = 8;                                                                               // set to max to exit for loop
      }
    }
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[1]);                                                               // format/print label
    modePrev_u8 = mode_u8;                                                                      // update previous state to current
    mode_u8 = STATE_INPUT;                                                                      // go to user input state
    }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERVICE){                                                            // stateX [OBD2 supported services prompt]
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[0]);                                                               // format/print seperator
    Serial.println(promptBuf[2]);                                                               // format/print seperator
    Serial.println(promptBuf[3]);                                                               // hotkeys set 2
    Serial.println(F("[0-1] = Selects OBD2 service"));                                          // hotkeys set 3
    Serial.println();                                                                           // format/print new line
    Serial.println(F("Supported OBD2 services:"));                                              // format/print user prompt
    for(j_u8=0;j_u8<2;j_u8++){                                                                  // start polling thru all supported OBD2 services
        Serial.write('0' + j_u8); Serial.print(") 0x");                                         // format/print option index
        if(odb2serv[j_u8] < 0x10){Serial.print("0");}                                           // format/print hex leading zero
        Serial.println(odb2serv[j_u8],HEX);                                                     // format/print online OBD2 service    
    }
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[1]);                                                               // format/print label
    s1PgIdx_u8 = 0;                                                                             // reset the page index
    modePrev_u8 = mode_u8;                                                                      // update previous state to current
    mode_u8 = STATE_INPUT;                                                                      // go to user input state
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERVING_S4){                                                         // stateX [clear diagnostics]
    for(j_u8=0;j_u8<8;j_u8++){txData[j_u8]=0;}                                                  // reset Tx buffer
    txData[0] = 2;                                                                              // #of additional data bytes
    txData[1] = 0x04;                                                                           // service code
    txData[2] = 0x00;                                                                           // PID
    mcp251xfd_msg_write(ptrChn[chnIdx],ecuId[j_u8],!IDE,!FDF,!BRS,!RTR,8,txData);               // populate channel struct with necessary transmit info
    mcp251xfd_write_memory(TXQ,ptrChn[chnIdx]);                                                 // write the Tx message info to MCP2517 TXQ
    mcp251xfd_start_transmit(TXQ,ptrChn[chnIdx]);                                               // initiate a transmit onto the CANbus network
    Serial.print(F("OBD2 Service 0x04 - Complete on ECU ID 0x"));                               // format/print OBD2 service complete
    Serial.println(ecuIdVal, HEX);                                                              // format/print OBD2 ECU ID
    modePrev_u8 = mode_u8;                                                                      // update previous state to current
    mode_u8 = STATE_SERVICE;                                                                    // go to user OBD2 service state
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERV_S1){                                                            // stateX [header prompt user for PID to monitor]
    for(j_u8=0;j_u8<6;j_u8++){                                                                  // start looping to find max #of service 0x01 PIDs supported
      txData[0] = 2;                                                                            // #of additional data bytes
      txData[1] = 0x01;                                                                         // service code
      txData[2] = 0x20 * j_u8;                                                                  // PID
      mcp251xfd_msg_write(ptrChn[chnIdx],ecuIdVal,!IDE,!FDF,!BRS,!RTR,8,txData);                // populate channel struct with necessary transmit info
      mcp251xfd_write_memory(TXQ,ptrChn[chnIdx]);                                               // write the Tx message info to MCP2517 TXQ
      mcp251xfd_start_transmit(TXQ,ptrChn[chnIdx]);                                             // initiate a transmit onto the CANbus network
      delay(100);                                                                               // delay to allow currently polled ECU to respond
      if(mcp251xfd_check_message(ptrChn[chnIdx])){                                              // check if current chnCAN has recieved a msg
        mcp251xfd_read_memory(FIFO1,ptrChn[chnIdx]);                                            // read current chnCAN message stored in FIFO1
        if(ptrChn[chnIdx]->msg.rxData[1] != (txData[1] + 0x40))                                 // check for +response from ECU OBD2 service 0x01
          ecuError_u8 = ERR_RSPNEG;                                                             // set ECU error flag         
      }
      else                                                                                      // no reply from ECU
        ecuError_u8 = ERR_RSPNO;                                                                // set ECU error flag

      s1PgMax_u8 = s1PgMaxCalc(txData[2],ptrChn[chnIdx]->msg.rxData);                           // calculate maximum OBD2 service 0x01 pages
      
      if(!(ptrChn[chnIdx]->msg.rxData[6] & 1))                                                  // next set of service 0x01 PIDs not supported by ECU
        j_u8 = 6;                                                                               // set to max to exit for loop  
    }
    if(!ecuError_u8){
      Serial.println();                                                                         // format/print new line
      Serial.println(promptBuf[0]);                                                             // format/print seperator
      Serial.println(promptBuf[2]);                                                             // hotkeys set 1
      Serial.println(promptBuf[3]);                                                             // hotkeys set 2
      Serial.println(F("[<]   = Prev set of PIDs"));                                            // hotkeys set 4
      Serial.println(F("[>]   = Next set of PIDs"));                                            // hotkeys set 4
      Serial.println(F("[a-p] = Selects a PID"));                                               // hotkeys set 5
      Serial.println(F("N/S   = Not supported  (Not a single frame PID)"));                     // Option key code 1
      Serial.println(F("N/A   = Not applicable (ECU does not support PID)"));                   // Option key code 2
      Serial.println();                                                                         // format/print new line
      Serial.println(F("Select PID to monitor (500ms):"));                                      // format/print user prompt
      modePrev_u8 = mode_u8;                                                                    // update previous state to current
      mode_u8 = STATE_SERV_S1a;                                                                 // go to user OBD2 service state
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERV_S1a){                                                           // stateX [prompt user for PID to monitor]
    Serial.print("[Line ");                                                                     // format/print line# callout
    if(s1PgIdx_u8 < 10) {Serial.print(" ");}                                                    // format/print leading space for alignment
    Serial.print(s1PgIdx_u8+1); Serial.print("/");                                              // format/print line# callout (cont.)
    
    if(s1PgMax_u8 < 10) {Serial.print(" ");}                                                    // format/print leading space for alignment
    Serial.print(s1PgMax_u8); Serial.print("] ");
    txData[0] = 2;                                                                              // #of additional data bytes
    txData[1] = 0x01;                                                                           // service code
    txData[2] = s1PidLookup(s1PgIdx_u8);                                                        // PID
    mcp251xfd_msg_write(ptrChn[chnIdx],ecuIdVal,!IDE,!FDF,!BRS,!RTR,8,txData);                  // populate channel struct with necessary transmit info
    mcp251xfd_write_memory(TXQ,ptrChn[chnIdx]);                                                 // write the Tx message info to MCP2517 TXQ
    mcp251xfd_start_transmit(TXQ,ptrChn[chnIdx]);                                               // initiate a transmit onto the CANbus network
    delay(100);                                                                                 // delay to allow currently polled ECU to respond
    if(mcp251xfd_check_message(ptrChn[chnIdx])){                                                // check if current chnCAN has recieved a msg
      mcp251xfd_read_memory(FIFO1,ptrChn[chnIdx]);                                              // read current chnCAN message stored in FIFO1
      if(ptrChn[chnIdx]->msg.rxData[1] != (txData[1] + 0x40))                                   // check for +response from ECU OBD2 service 0x01
        ecuError_u8 = ERR_RSPNEG;                                                               // set ECU error flag         
    }
    else                                                                                        // no reply from ECU
      ecuError_u8 = ERR_RSPNO;                                                                  // set ECU error flag

    if(!ecuError_u8){                                                                           // no ECU errors
      s1PidSpt_u16 = s1PidSpt(s1PgIdx_u8,ptrChn[chnIdx]->msg.rxData);                           // combine 2 of the 4 bytes recieved to single 16 bit value based on s1 page index
      for(j_u8=0;j_u8<16;j_u8++){                                                               // start looping thru current OBD2 serivce 0x01 PID enable bits
        rVal[1] = (s1PidSpt_u16 >> (15-j_u8)) & 1;                                              // get PID enable bit starting at MSB
        rVal[2] = s1PidPlen[s1PgIdx_u8][j_u8];                                                  // get PID payload length
        rVal[3] = (s1PgIdx_u8 * 0x10) + j_u8;                                                   // calculate serivice 0x01 PID
        Serial.write('A' + j_u8); Serial.print(")");                                            // format/print option char 
        s1Idx[j_u8][0] = 'A' + j_u8;                                                            // save the ASCII option char
        
        if(rVal[1]){                                                                            // check that serivice 0x01 PID is supported
          if(rVal[2] <= 5){                                                                     // check that serivice 0x01 PID payload is 8 bytes or less
            Serial.print("0x");                                                                 // format/print PID 
            if(rVal[3] < 0x10)  {Serial.print("0");}                                            // format/print leading zero
            Serial.print(rVal[3], HEX);                                                         // format/print PID (cont.)
            s1Idx[j_u8][1] = rVal[3];                                                           // save s1 PID
          }
          else{                                                                                 // PID not single frame
            Serial.print("N/S ");                                                               // format/print PID not supported by program
            s1Idx[j_u8][1] = 0xFF;                                                              // set to max to indicate invalid option
          }
        }
        else{                                                                                   // PID not supported by ECU
          Serial.print("N/A ");                                                                 // format/print PID not supported by ECU
          s1Idx[j_u8][1] = 0xFF;                                                                // set to max to indicate invalid option
        }
        Serial.print(" ");                                                                      // format/print space between options
      }
      Serial.println();                                                                         // format/print label
      Serial.println();                                                                         // format/print label
      Serial.println(promptBuf[1]);                                                             // format/print label
      modePrev_u8 = mode_u8;                                                                    // update previous state to current
      mode_u8 = STATE_INPUT;                                                                    // go to user OBD2 service state
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERVING_S1){                                                         // stateX [PID continous monitoring]
    Serial.println();                                                                           // format/print new line
    Serial.println(promptBuf[0]);                                                               // format/print seperator
    Serial.println(promptBuf[2]);                                                               // hotkeys set 1
    Serial.println(promptBuf[3]);                                                               // hotkeys set 2
    Serial.println(F("[s]   = Start/Stop continous monitoring"));                               // hotkeys set 3
    Serial.println();                                                                           // format/print new line
    Serial.println();                                                                           // format/print label
    Serial.println(promptBuf[1]);                                                               // format/print label
    modePrev_u8 = mode_u8;                                                                      // update previous state to current
    mode_u8 = STATE_INPUT;                                                                      // go to user OBD2 service state
  }
  // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if(mode_u8 == STATE_SERVING_S1a){                                                        // stateX [PID continous monitoring]
    txData[0] = 2;                                                                              // #of additional data bytes
    txData[1] = 0x01;                                                                           // service code
    txData[2] = s1PidIdx_u8;                                                                    // PID
    mcp251xfd_msg_write(ptrChn[chnIdx],ecuIdVal,!IDE,!FDF,!BRS,!RTR,8,txData);                  // populate channel struct with necessary transmit info
    mcp251xfd_write_memory(TXQ,ptrChn[chnIdx]);                                                 // write the Tx message info to MCP2517 TXQ
    mcp251xfd_start_transmit(TXQ,ptrChn[chnIdx]);                                               // initiate a transmit onto the CANbus network
    delay(500);                                                                                 // delay to allow currently polled ECU to respond & serial writes
    if(mcp251xfd_check_message(ptrChn[chnIdx])){                                                // check if current chnCAN has recieved a msg
      mcp251xfd_read_memory(FIFO1,ptrChn[chnIdx]);                                              // read current chnCAN message stored in FIFO1
      if(ptrChn[chnIdx]->msg.rxData[1] != (txData[1] + 0x40))                                   // check for +response from ECU OBD2 service 0x01
        ecuError_u8 = ERR_RSPNEG;                                                               // set ECU error flag         
    }
    else                                                                                        // no reply from ECU
      ecuError_u8 = ERR_RSPNO;                                                                  // set ECU error flag

    if(!ecuError_u8){                                                                           // no ECU errors
      obd2s1PidDecrypt(ptrChn[chnIdx]->msg.rxData);
    }
    if(Serial.available()){                                                                     // check if user input char(s)
      modePrev_u8 = mode_u8;                                                                    // update previous state to current
      mode_u8 = STATE_INPUT;                                                                    // go to input state
    }
  }
  if(ecuError_u8){                                                                              // ECU error flag set
    Serial.println();                                                                           // format/print new line
    Serial.print("** ECU 0x");Serial.print(ecuIdVal,HEX);                                       // format/print that ECU no longer online
    if(ERR_RSPNO)                                                                               // ECU no response
      Serial.println(F(" - Error: ECU offline **"));                                            // format/print that ECU no longer online (cont.)
    else if(ERR_RSPNEG)                                                                         // ECU no response
      Serial.println(F(" - Error: ECU negative response **"));                                  // format/print that ECU no longer online (cont.)

    Serial.println(F("Going back to homepage"));                                                // format/print prompt going back to homepage
    modePrev_u8 = mode_u8;                                                                      // update previous state to current
    mode_u8 = STATE_POLL;                                                                       // go to user OBD2 service state
  }
}

word s1PidSpt(byte pgIdx_u8,byte *ptrBuf_u8) {
  word rVal_u16;
  if(pgIdx_u8 % 2 == 0){
    rVal_u16 = (rVal_u16 << 0)  | ptrBuf_u8[3];
    rVal_u16 = (rVal_u16 << 8)  | ptrBuf_u8[4];
  }
  else{
    rVal_u16 = (rVal_u16 << 0) | ptrBuf_u8[5];
    rVal_u16 = (rVal_u16 << 8) | ptrBuf_u8[6];
  }
  return rVal_u16;
}

byte s1PidLookup(byte pgIdx_u8){
  if(pgIdx_u8 < 2)        return 0x00;
  else if(pgIdx_u8 < 4)   return 0x20;
  else if(pgIdx_u8 < 6)   return 0x40;
  else if(pgIdx_u8 < 8)   return 0x60;
  else if(pgIdx_u8 < 10)  return 0x80;
  else                    return 0xA0;
}

byte s1PgMaxCalc(byte s1Pid,byte *ptrBuf_u8) {
  if(s1Pid == 0x00)       return 2 + (ptrBuf_u8[6] & 1);
  else if(s1Pid == 0x20)  return 4 + (ptrBuf_u8[6] & 1);
  else if(s1Pid == 0x40)  return 6 + (ptrBuf_u8[6] & 1);
  else if(s1Pid == 0x60)  return 8 + (ptrBuf_u8[6] & 1);
  else if(s1Pid == 0x80)  return 10 + (ptrBuf_u8[6] & 1);
  else                    return 11;
}
