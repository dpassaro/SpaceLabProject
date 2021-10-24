 /*
// interfaccia con kayboard
  pinMode(KEYB_DOUT_PIN, INPUT);
// outputs
  pinMode(KEYB_SH1_LD0_PIN, OUTPUT);init
  pinMode(KEYB_SH_CLK_PIN, OUTPUT);
 */
// define per keyboard
//
// mask for AND, if (readKeyb() & <mask>) == <mask>) {<maskNameIsTrue}
//
#define keybConnectedMask 0x5500
#define IN3isOFFmask 0x20
#define IN2isOFFmask 0x04
#define IN1isOFFmask 0x02
#define IN0isOFFmask 0x80
#define TrigIsORmask 0x10
// da capire cosa farci
#define keybIsEnabledMask 0x40
#define keybResetMask 0x08
#define keybDispONmask 0x01

//
#define keybIsConnected (dataIN & 0xFF00) == keybConnectedMask
#define IN3isON (dataIN & IN3isOFFmask) != IN3isOFFmask
#define IN2isON (dataIN & IN2isOFFmask) != IN2isOFFmask
#define IN1isON (dataIN & IN1isOFFmask) != IN1isOFFmask
#define IN0isON (dataIN & IN0isOFFmask) != IN0isOFFmask
#define TrigIsOR (dataIN & TrigIsORmask) == TrigIsORmask
#define TrigIsAND (dataIN & TrigIsORmask) != TrigIsORmask
// da capire cosa farci
#define keybIsEnabled (dataIN & keybIsEnabledMask) == keybIsEnabledMask
#define keybResetON (dataIN & keybResetMask) != keybResetMask
#define keybDispON (dataIN & keybDispONmask) != keybDispONmask

boolean keybConnected=false; // TRUE: keyboard is connected
boolean keybConnected_prev=false; // TRUE: keyboard was connected

byte inEnable=0;      // i 4 LSbits sono gli enable degli input: =1 enabled
byte keybTriggerAndOr; // =0 OR
boolean keybEnable; // =false disabled anche se connected
boolean dispKeyPressed; // =true pulsante DISP premuto
boolean resetKeyPressed; // =true pulsante RESET premuto
//
// da definire l'uso degli switch/pulsanti DISP, RESET, KEYB ON/OFF
//

/*
uint16_t scanKeyb(void) {
  uint16_t dataIN;
  byte i;
  // load keyb sreg
  digitalWrite(KEYB_SH1_LD0_PIN, 0);
  digitalWrite(KEYB_SH1_LD0_PIN, 1);
  // clock to 0
  digitalWrite(KEYB_SH_CLK_PIN, 0);
  // read data
  for(i=0; i<16; i++) {
    dataIN = (dataIN<<1) + digitalRead(KEYB_DOUT_PIN);
    digitalWrite(KEYB_SH_CLK_PIN, 1);
    digitalWrite(KEYB_SH_CLK_PIN, 0);
  }

// setta le variabili globali in base a quanto letto
  if(keybIsConnected) {
   // Serial.println("KEYB");
    keybConnected=true; 
    if(IN0isON) bitSet(inEnable,0); else bitClear(inEnable,0);
    if(IN1isON) bitSet(inEnable,1); else bitClear(inEnable,1);
    if(IN2isON) bitSet(inEnable,2); else bitClear(inEnable,2);
    if(IN3isON) bitSet(inEnable,3); else bitClear(inEnable,3);
    if(TrigIsOR) keybTriggerAndOr=0; else keybTriggerAndOr=0xFF;
    if(keybIsEnabled) keybEnable=true; else keybEnable=false;
    if(keybDispON) dispKeyPressed=true; else dispKeyPressed=false;
    if(keybResetON) resetKeyPressed=true; else resetKeyPressed=false;
  }
  else keybConnected=false;
  
  return(dataIN);
}
*/

/*
uint16_t LED_DATA;

void WR595_LED (uint16_t DATA16) {
  int8_t bitCnt;

  for(bitCnt=15; bitCnt>=0; bitCnt--) {
    digitalWrite( SERIN, 0 ); 
    if(bitRead(DATA16, bitCnt)) digitalWrite( SERIN, 1 );
 
    digitalWrite( SRCLK, 1 );
    digitalWrite( SRCLK, 0 );
  }
  digitalWrite( RCLK_LED, 1 );
  digitalWrite( RCLK_LED, 0 );  
}
*/
void copyStatusToFPGA(void) {
   bitClear(XLR8_TrigConfReg1, GLOBAL_ENABLE); // disa counters
   bitClear(XLR8_TrigConfReg, TRIGGER_ENA); // disa TRIGGER

   // T2TcountLimit
   XLR8_T2TcountLimitL = read_FRAM_byte(EEPROM_T2TcountLimit_ADDR);
   XLR8_T2TcountLimitH = read_FRAM_byte(EEPROM_T2TcountLimit_ADDR+1);

   // counters
   XLR8_TrigCnt_rdout = read_FRAM_long24(EEPROM_XLR8_TrigCnt_ADDR);
   XLR8_TimeCnt_ms_rdout = read_FRAM_long32(EEPROM_XLR8_TimeCnt_ms_ADDR);
   XLR8_EvCnt0_rdout = read_FRAM_long24(EEPROM_XLR8_IN0Cnt_ADDR);
   XLR8_EvCnt1_rdout = read_FRAM_long24(EEPROM_XLR8_IN1Cnt_ADDR);
   XLR8_EvCnt2_rdout = read_FRAM_long24(EEPROM_XLR8_IN2Cnt_ADDR);
   XLR8_EvCnt3_rdout = read_FRAM_long24(EEPROM_XLR8_IN3Cnt_ADDR);
   XLR8_And0_rdout = read_FRAM_long24(EEPROM_XLR8_And0_ADDR);
   XLR8_And1_rdout = read_FRAM_long24(EEPROM_XLR8_And1_ADDR);
   XLR8_And2_rdout = read_FRAM_long24(EEPROM_XLR8_And2_ADDR);
//   XLR8_And3_rdout = read_FRAM_long32(EEPROM_XLR8_And3_ADDR);
   XLR8_And0_mask = read_FRAM_byte(EEPROM_XLR8_And0_mask_ADDR);
   XLR8_And1_mask = read_FRAM_byte(EEPROM_XLR8_And1_mask_ADDR);
   XLR8_And2_mask = read_FRAM_byte(EEPROM_XLR8_And2_mask_ADDR);
   XLR8_Trig_mask = read_FRAM_byte(EEPROM_XLR8_Trig_mask_ADDR);
   // conf regs
   XLR8_TrigConfReg = read_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR);
   XLR8_TrigConfReg1 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);
}

void scanKeybStable(void) { // ci vorrebbe un loop in cui almeno tre letture consecutive spaziate di X ms rivelano la keyb
#ifdef pusantiOnly
     keybConnected = false;
#else
  keybConnected_prev = keybConnected;
  
  if(keybConnected_prev==false) {
     scanKeyb();
     if(keybConnected==true) {delay(500); scanKeyb(); }  // nel caso di cambiamento dell'ID 0-1 vuol dire che stiamo inserendo la key, aspetta mezzo secondo e rifai il check
  }
  else {
     scanKeyb();
     if(keybConnected==false) {delay(500); scanKeyb();} // nel caso di cambiamento dell'ID 1-0 vuol dire che stiamo estraendo la key, aspetta mezzo secondo e rifai il check
     if( (keybConnected_prev==true) & (keybConnected==false) ) { copyStatusToFPGA();} // keyb estratta, restora lo stato dalla FRAM
  }
#endif
}

void printMenu(void) {
  Serial.print(F("\n\r========= COSMOCUBE Arduino  XLR8 =========\n\r"));
  Serial.print(F("<h> or <space> print this list\n\r"));
  Serial.print(F("-------------------------------------------\n\r"));
  Serial.print(F("<s> Circuit Status\n\r"));
  Serial.print(F("-------------------------------------------\n\r"));
  Serial.print(F("<d> Scroll up pagina display\n\r"));
  Serial.print(F("<D> Scroll dn pagina display\n\r"));
  Serial.print(F("<o> Display on/off\n\r"));
  Serial.print(F("<1> Enable/disable TOC\n\r"));
  Serial.print(F("<2> SELECT TOC SOURCE TRG/AND\n\r"));
  Serial.flush();
#if defined(FULL)
  Serial.print(F("<3> globalEnable, ENA/DISA\n\r"));
#endif
  Serial.print(F("<k> Reset TT data buffer\n\r"));
  Serial.print(F("<9> TT data buffer dump\n\r"));
  Serial.print(F("-------------------------------------------\n\r"));
  Serial.print(F("<t> Trigger MENU\n\r"));
  Serial.print(F("<r> RESET FPGA COUNTER + REBOOT PROCESSOR\n\r"));
  Serial.print(F("===========================================\n\r"));
  Serial.flush();
#if defined(FULL)
  Serial.print(F("<p> test write/read trigger counter\n\r"));
  Serial.print(F("<w> copy FRAM config regs to FPGA\n\r"));
  Serial.print(F("<4> Reset TRIGGER\n\r"));
  Serial.print(F("<<> test pulse IN3\n\r"));
  Serial.print(F("<x> test FRAM\n\r"));
  Serial.print(F("<0> data buffer filling level\n\r"));
  Serial.flush();
#endif
}

void printTriggerMenu(void) {
  Serial.println();
  Serial.println(F("===== Trigger STATUS ====="));
  Serial.println(F("<s> print TRIGGER status"));
  Serial.println(F("===== Trigger MODE ====="));
  Serial.println(F("<a> AND"));
  Serial.println(F("<o> OR"));
  Serial.println(F("<t> T2T"));
  Serial.println(F("<y> T2T_OR"));
  Serial.println(F("<q> And0-3 menu")); 
  Serial.flush();
  // versione a 2 scintillatori, tolti i contatori di altre coincidenze
  /*
  #if defined(FULL) 
   //   Serial.println(F("<m> mu decay"));
   //   Serial.println(F("<f> floor for mu decay"));
      Serial.println(F("<q> And0-3 menu")); 
  #endif
  */
  Serial.println(F("==== Trigger ENABLE ===="));
  Serial.println(F("<e> inputs 0..7 enable"));
  Serial.println(F("<g> global TRIGGER enable"));
  Serial.println(F("========================"));
  Serial.println(F("<c> Set T2T count limit"));
  Serial.println(F("========================"));
  Serial.println(F("<d> Display Flash mode ena/disa"));
  Serial.println(F("========================"));
  Serial.println(F("<h> <sp> print this menu"));  
  Serial.println(F("<x> exit"));  
  Serial.flush();
}


/*
void FIFO2FRAM(void) {
  uint16_t dataRead;
 // uint32_t temp32;
  uint8_t n;
  
//   write_pointer = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);
   write_pointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

   n=0;

   dataRead = XLR8_T2Treg;
   while( (write_pointer <= (FRAM_SIZE-1)) & (!bitRead(dataRead, 15)) ) {  // metterci il parametro
       //  temp32 = ( ( (uint32_t)dataRead ) * timeUnit_ps + 499)/1000;
       //  Serial.println(temp32);
       
       //  write_FRAM_long16(write_pointer, dataRead);
         write_FRAM_long32(write_pointer, dataRead);
         write_pointer += 2; // incrementa puntatore
      //   write_FRAM_long16(DATA_BUFFER_WRITE_POINTER, write_pointer); 
         write_FRAM_long32(DATA_BUFFER_WRITE_POINTER, write_pointer); 
         if(write_pointer > (FRAM_SIZE-1)) break;  // FRAM BUFFER FULL!

         // FRAM not full, read next from FIFO if available
         XLR8_T2Treg_L = 0; 
         dataRead = XLR8_T2Treg;
         n++;
   }
}
*/

void LOG2FRAMx(uint32_t data24) {
   write_pointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

         write_FRAM_long24(write_pointer, data24);
         write_FRAM_long32(DATA_BUFFER_WRITE_POINTER, write_pointer); 
}

void LOG2FRAM(uint32_t data24) {

 //  noInterrupts();
   write_pointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

   if( (write_pointer <= (FRAM_SIZE-2)) && (write_pointer >= DATA_BUFFER_START_ADDR) ) {  // se il buffer non è full, c'è spazio per almeno altri 3 byte
         write_FRAM_long24(write_pointer, data24);
         write_pointer += NbytePerSingleLogData; // incrementa puntatore
         if(write_pointer >= DATA_BUFFER_START_ADDR) write_FRAM_long32(DATA_BUFFER_WRITE_POINTER, write_pointer); 
     //    if(write_pointer > (FRAM_SIZE-1)) break;  // FRAM BUFFER FULL! 
   }

 //  interrupts();
}


void testPulseIN3(void) {
  uint8_t n, n_max;//, temp8;
  uint8_t doPrint;
  uint16_t dataRead;

  Serial.print(F("--> enter num of couple of pulses: "));
  n_max = readByte_EndByAnyChar();
  Serial.println(n_max);

  Serial.print(F("--> read and print FIFO? 1/0: "));
  doPrint = readByte_EndByAnyChar();
  Serial.println(doPrint);

 //  XLR8_T2Treg_L = 0;  // un colpo di lettura, forse da mettere al boot dopo il reset generale
 //  XLR8_T2Treg_H = 0;  // un colpo di scrittura, forse da mettere al boot dopo il reset generale
 //  Serial.println(XLR8_T2Treg);  XLR8_T2Treg_L = 0;
 //  Serial.println(XLR8_T2Treg);  XLR8_T2Treg_L = 0;

   // svuota la FIFO
   dataRead = XLR8_T2Treg;
   while(!bitRead(dataRead, 15)) {XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg;}
  
 for(n=0; n<n_max; n++) {
    XLR8_CPU_Pulse;
  __builtin_avr_delay_cycles(60);
    XLR8_CPU_Pulse;
  __builtin_avr_delay_cycles(60);
  }
 // __builtin_avr_delay_cycles(6000);

if(doPrint==1) {
   n = 0;
   dataRead = XLR8_T2Treg;
   while(!bitRead(dataRead, 15)) {
      Serial.println(dataRead);
      XLR8_T2Treg_L = 0; 
      dataRead = XLR8_T2Treg;
      n++;
      }
   Serial.print("Num of data: "); Serial.println(n);
}
   /*
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r0");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r1");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r2");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r3");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r4");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r5");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r6");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r7");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r8");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r9");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   dataRead = XLR8_T2Treg; if(bitRead(dataRead,15)) {Serial.println("r10");  XLR8_T2Treg_L = 0; dataRead = XLR8_T2Treg; } Serial.println(dataRead);  XLR8_T2Treg_L = 0;
   */
}


void trigCntRWtest(void) {
  XLR8_TrigCnt_rdout = 87654321;
  XLR8_TimeCnt_ms_rdout = 12345678;
}

uint32_t readTimeCnt_ms(byte doLatch) {
 //uint32_t t32;
 if(doLatch) {
   bitSet(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
   bitClear(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
 }   
// return( (uint32_t)XLR8_TimeCnt_ms_rdout0 + (uint32_t)XLR8_TimeCnt_ms_rdout1 * 256 + (uint32_t)XLR8_TimeCnt_ms_rdout2 * 65536 + (uint32_t)XLR8_TimeCnt_ms_rdout3 * 16777216 );
 return( XLR8_TimeCnt_ms_rdout );
}

uint32_t readTriggerCnt(byte doLatch) {
 //uint32_t t32;
 if(doLatch) {
   bitSet(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
   bitClear(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
 }   
// return( (uint32_t)XLR8_TrigCnt_rdout0 + (uint32_t)XLR8_TrigCnt_rdout1 * 256 + (uint32_t)XLR8_TrigCnt_rdout2 * 65536 + (uint32_t)XLR8_TrigCnt_rdout3 * 16777216 );
 return( XLR8_TrigCnt_rdout & 0xFFFFFF );
}

/*
uint32_t readTriggerIntrCnt(byte doLatch) {
 uint32_t t32;
 if(doLatch) {
   bitSet(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
   bitClear(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
 }   
 return( XLR8_TrigIntrCnt_rdout );
}
*/

uint32_t readInputCnt(byte doLatch, byte whichInput) {
// uint32_t t32;
 if(doLatch) {
   bitSet(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
   bitClear(XLR8_TrigConfReg1, LATCH_READOUT_DATA);
 } 
 //Serial.println(whichInput);  
 switch  (whichInput) {
    case 0: return( XLR8_EvCnt0_rdout & 0xFFFFFF );
    case 1: return( XLR8_EvCnt1_rdout & 0xFFFFFF );
    case 2: return( XLR8_EvCnt2_rdout & 0xFFFFFF );
    case 3: return( XLR8_EvCnt3_rdout & 0xFFFFFF );
    case 4: return( XLR8_EvCnt4_rdout & 0xFFFFFF);
    case 5: return( XLR8_EvCnt5_rdout & 0xFFFFFF);
    case 6: return( XLR8_EvCnt6_rdout & 0xFFFFFF);
    case 7: return( XLR8_EvCnt7_rdout & 0xFFFFFF);
    case 8: return( XLR8_And0_rdout & 0xFFFFFF);
    case 9: return( XLR8_And1_rdout & 0xFFFFFF);
    case 10: return( XLR8_And2_rdout & 0xFFFFFF);
    default: return(0);
 }
}



void dispFlashEnaDisa(void) {
  uint8_t temp8, input8;

  temp8 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);

  Serial.print(F("\n\r dispFlashEna: FRAM = "));
  Serial.print(bitRead(temp8,ENA_FLASH));
  Serial.print(F("  XLR8 ="));
  Serial.print(bitRead(XLR8_TrigConfReg1,ENA_FLASH));
  Serial.print(F("\n\r"));

  Serial.print(F("--> Enable/Disable (=1/0): "));
  input8 = readByte_EndByAnyChar();
  if(input8) {
    bitSet(temp8, ENA_FLASH);
    bitSet(XLR8_TrigConfReg1, ENA_FLASH);
  }
  else {
    bitClear(temp8, ENA_FLASH);
    bitClear(XLR8_TrigConfReg1, ENA_FLASH);
  }
  Serial.println();

  write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, temp8);  // update FRAM
  
}

void TOCsource(void) {
  uint8_t temp8, input8;

  temp8 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);

  Serial.print(F("\n\r TOCsource: FRAM = "));
  Serial.print(bitRead(temp8, TOCsourceSel));
  Serial.print(F("  XLR8 ="));
  Serial.print(bitRead(XLR8_TrigConfReg1, TOCsourceSel));
  Serial.print(F("\n\r"));

  Serial.print(F("--> Which source? TRIGGER=1, AND=0: "));
  input8 = readByte_EndByAnyChar();
  if(input8) {
    bitSet(temp8, TOCsourceSel);
    bitSet(XLR8_TrigConfReg1, TOCsourceSel);
  }
  else {
    bitClear(temp8, TOCsourceSel);
    bitClear(XLR8_TrigConfReg1, TOCsourceSel);
  }
  Serial.println();

  write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, temp8);  // update FRAM
  
}


void writeGlobalEnable(void) {
  uint8_t temp8, input8;

//  temp8 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);
  temp8 = XLR8_TrigConfReg1;

  Serial.print(F("\n\r globalEnable: XLR8 = "));
  Serial.print(bitRead(temp8, GLOBAL_ENABLE));
  Serial.print(F("  FRAM ="));
  Serial.print(bitRead(read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR), GLOBAL_ENABLE));
  Serial.print(F("\n\r"));

  Serial.print(F("--> set Enable/Disable (=1/0) for globalEnable: "));
  input8 = readByte_EndByAnyChar();
  if(input8) bitSet(temp8, GLOBAL_ENABLE);
  else bitClear(temp8, GLOBAL_ENABLE);
  Serial.println();

  write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, temp8);  // update FRAM
  XLR8_TrigConfReg1 = temp8; // update XLR8 register
  

  if( read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR)!= XLR8_TrigConfReg1 ) {
    Serial.print(F("Memory/XLR8 mismatch: FRAM hex "));
    Serial.print(read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR), HEX);
    Serial.print(F("  XLR8 hex "));
    Serial.print(XLR8_TrigConfReg1, HEX);
    Serial.println();
  }
  else {
    Serial.print(F("Written in memory & XLR8 register.\n\r"));
  }
}


void dataBufferFillingLevel(void) {
 // uint16_t currentWritePointer; //, bufferPointer;
  uint32_t currentWritePointer; 

//  currentWritePointer = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);
  currentWritePointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

  Serial.println();
  Serial.print((currentWritePointer - DATA_BUFFER_START_ADDR)/2);
  Serial.print(F(" data written of "));
  Serial.print((FRAM_SIZE-1 - DATA_BUFFER_START_ADDR + 1)/2);

  Serial.println(F(" available"));
}


void printRate(uint32_t nevents, uint32_t timeSec) {
    Serial.println((float)nevents*60000 /(float)timeSec,4); 
}

void printAndInfo(byte wichAnd, byte mask, uint32_t tempEvent32, uint32_t tempTime32) {
  Serial.print(F("And")); Serial.print(wichAnd);  Serial.print(F(" cnt: ")); 
  Serial.print(tempEvent32); Serial.print(F(" mask: ")); print_binary(mask, 8); Serial.print(F(" Rm: ")); printRate(tempEvent32, tempTime32);
}

void printVoltages(void) {
    char str_temp[6];
    unsigned long temp32;
// S (THR)
      Vadc1 = analogRead(Vthr1);
      Vadc2 = analogRead(Vthr2);
#if defined(FULL)
      Serial.print(F("\n\rADC Thr: "));
      Serial.println(Vadc1);
#endif
      Vadc1 = (int16_t) ((((int32_t)Vadc1 - 520)*489)/100);  // 4.89 mV/step, 2.50 V Vref
      Serial.print("Thr: ");
      if(Vadc1>0) Serial.print('+');
      if(Vadc1==0) Serial.print(' ');     
      Serial.print(Vadc1); Serial.println(F("mV"));

      Vadc2 = (int16_t) ((((int32_t)Vadc2 - 520)*489)/100);  // 4.89 mV/step, 2.50 V Vref
      Serial.print("Thr: ");
      if(Vadc2>0) Serial.print('+');
      if(Vadc2==0) Serial.print(' ');     
      Serial.print(Vadc1); Serial.println(F("mV"));
// Vb     
      temp32 = (unsigned long)analogRead(Vbias);
#if defined(FULL)
      Serial.print(F("ADC Bias: "));
      Serial.println(temp32);
#endif
     //   temp32 = (temp32*496)/1000;  // legge Vbias/10, step 5 mV, quindi *5 e trovo i mV, *10 trovo la vera Vbias in mV  METTO 496/10 per compensare errori partitore, /100 per risoluzione 0.1V
      temp32 = (temp32*480)/1000;  // legge Vbias/10, step 5 mV, quindi *5 e trovo i mV, *10 trovo la vera Vbias in mV  METTO 496/10 per compensare errori partitore, /100 per risoluzione 0.1V

     // 1st param is mininum width, 2nd param is precision; float value is copied onto str_temp
      dtostrf((float)temp32/10, 2, 1, str_temp);
      Serial.print(F("Bias: "));
      Serial.print(str_temp); Serial.println(F("V")); 
}

void printStatus(void) {
  uint8_t temp8;//, temp8mode;
  uint32_t temp32, nevents32, tempEvent32;
 // float float32;
  
  Serial.println();
  Serial.println(F("CIRCUIT STATUS:"));
  Serial.println();

  printVoltages();

  temp8 = XLR8_TrigConfReg1;
 // temp8mode = (temp8&0b11100000)>>5;
  
  Serial.print(F("TRIGGER is "));
  if(bitRead(temp8, GLOBAL_ENABLE)) Serial.println(F("enabled"));
  else Serial.println(F("disabled"));
  
  temp8 = XLR8_Trig_mask;
  Serial.print(F("INPUTS ENABLED: "));
  
  if(bitRead(temp8, 7)) Serial.print(F("IN7 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 6)) Serial.print(F("IN6 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 5)) Serial.print(F("IN5 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 4)) Serial.print(F("IN4 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 3)) Serial.print(F("IN3 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 2)) Serial.print(F("IN2 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 1)) Serial.print(F("IN1 "));
  else Serial.print(F("xxx "));
  if(bitRead(temp8, 0)) Serial.print(F("IN0 "));
  else Serial.print(F("xxx "));
  Serial.println();

  temp8 = XLR8_TrigConfReg;
  Serial.print(F("TRIGGER MODE: "));
  if(bitRead(temp8, 7)) Serial.print(F("AND"));
  else Serial.print(F("OR"));
  Serial.print(F(" --- "));
  if(bitRead(temp8, 5)) Serial.print(F("TtoT (on AND/OR of ena inputs) <= T2T limit"));
  else Serial.print(F("TtoT (enabled inputs in parallel) <= T2T limit"));
  //else if(temp8mode == TRIGGER_MODE_MU_DECAY) Serial.print(F("4-layers mu decay with Trigger-to-Trigger time <= T2T limit"));
  //else if(temp8mode == TRIGGER_MODE_FLOOR) Serial.print(F("4-layers FLOOR with Trigger-to-Trigger time <= T2T limit"));
//  else Serial.print(F("*** UNKNOWN ***"));
  Serial.println();

  Serial.print(F("Trigger-to-Trigger time (T2T) limit: "));
//  temp32 = ((uint32_t)((uint16_t)XLR8_T2TcountLimitL + (uint16_t)XLR8_T2TcountLimitH*256) * timeUnit_ps + 499)/1000;
  temp32 = (XLR8_T2TcountLimit * timeUnit_ps + 499)/1000;
  Serial.print(temp32);
  Serial.println(" ns");

  Serial.print(F("TT buffer: "));
//  Serial.print((read_FRAM_long16(DATA_BUFFER_WRITE_POINTER) - DATA_BUFFER_START_ADDR)/2);
  Serial.print((read_FRAM_long32(DATA_BUFFER_WRITE_POINTER) - DATA_BUFFER_START_ADDR)/NbytePerLog);
  Serial.print(F(" data written of "));
  Serial.print((FRAM_SIZE-1 - DATA_BUFFER_START_ADDR + 1)/NbytePerLog);
  Serial.println(F(" available"));

  Serial.print(F("TOC:   "));
  if(bitRead(XLR8_TrigConfReg1, TOCenable)) Serial.print(F("ENABLED"));
  else Serial.print(F("DISABLED"));
  Serial.print(F(", on "));
  if(bitRead(XLR8_TrigConfReg1, TOCsourceSel)) Serial.println(F("TRIGGER"));
  else Serial.println(F("AND"));

  Serial.print(F("Display FLASH: "));
  if(bitRead(XLR8_TrigConfReg1, ENA_FLASH)) Serial.println(F("ENABLED"));
  else Serial.println(F("DISABLED"));

  Serial.print(F("Time counter (ms): ")); temp32 = readTimeCnt_ms(1); Serial.println(temp32);  // qui avvine il latch di tutti i dati del readout
  Serial.print(F("TRIGGER counter    : ")); nevents32 = readTriggerCnt(0); Serial.println(nevents32);
  Serial.print(F("Rate medio (ev/min): ")); printRate(nevents32, temp32);

  for(temp8=0; temp8<8; temp8++) {
      Serial.print(F("IN")); Serial.print(temp8); Serial.print(F(" cnt: ")); Serial.println(readInputCnt(0,temp8));
  }

// lettura conteggi e mask degli AndExtra
  tempEvent32 = readInputCnt(0,8); printAndInfo(0, XLR8_And0_mask, tempEvent32, temp32);
  tempEvent32 = readInputCnt(0,9); printAndInfo(1, XLR8_And1_mask, tempEvent32, temp32);
  tempEvent32 = readInputCnt(0,10); printAndInfo(2, XLR8_And2_mask, tempEvent32, temp32);

  // contenuto registri TrigConfReg1
 Serial.println(F("==========================="));
 Serial.println(F("== ALTRO in TrigConfReg1 =="));
 Serial.println(F("==========================="));

 temp8 = XLR8_TrigConfReg1;

 Serial.print(F("globalEnable (counters) "));
 if(bitRead(temp8, GLOBAL_ENABLE)) Serial.println(F("ENABLED"));
 else Serial.println(F("DISABLED"));

 Serial.print(F("latch data for read ="));
 if(bitRead(temp8, 3)) Serial.println(F("1"));
 else Serial.println(F("0"));

 Serial.print(F("reset time ="));
 if(bitRead(temp8, TIME_AND_COUNTERS_RESET)) Serial.println(F("1"));
 else Serial.println(F("0"));

 Serial.print(F("reset trigger ="));
 if(bitRead(temp8, TRIGGER_RESET)) Serial.println(F("1"));
 else Serial.println(F("0"));

}


// ================================================================
//   LETTURA DATI SUGLI EVENTI, DA USARE AD OGNI TIMER INTERRUPT
// ================================================================
void readEventsData(void) {
  //Time counter (ms), serve per rate60sec()
  time_in_ms = readTimeCnt_ms(1); // qui avviene il latch di tutti i dati del readout
  tempoSecondi = (time_in_ms + 499)/1000;

  //TRIGGER counter, serve per rate60sec() 
  numTrigger = readTriggerCnt(0);
  
  //TRIGGER INT counter
  // = readTriggerIntrCnt(0); 

  /*
  //IN0 counter
  numIN0 = readInputCnt(0,0); 
  //IN1 counter
  numIN1 = readInputCnt(0,1);
  //IN2 counter
  numIN2 = readInputCnt(0,2);
  //IN3 counter
  numIN3 = readInputCnt(0,3);
  */
}

void printVbias(void) {
  //---------------------------------------- STAMPA V_BIAS: VERIFICARE SE FUNZIONA
  char str_temp[6];
  unsigned long temp;
  
  temp = (unsigned long)analogRead(Vbias);
  temp = (temp*480)/1000;  // legge Vbias/10, step 5 mV, quindi *5 e trovo i mV, *10 trovo la vera Vbias in mV  METTO 496/10 per compensare errori partitore, /100 per risoluzione 0.1V
  // 1st param is mininum width, 2nd param is precision; float value is copied onto str_temp
  dtostrf((float)temp/10, 2, 1, str_temp);
  
  Serial.print(str_temp); Serial.flush(); 
  //----------------------------------------
}

void dataBufferDumpPallone(void) {
//  uint16_t currentWritePointer, bufferPointer;
  uint32_t currentWritePointer, bufferPointer;
  uint32_t temp32;
  uint8_t i;
  //noInterrupts();
  
  Serial.println();
  Serial.println(F("Time_sec trig #0 #1 #2 #3 #4 #5 #6 #7 And0 And1 And2 V_bias:"));
  Serial.println();
  currentWritePointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

  if(currentWritePointer < DATA_BUFFER_START_ADDR) {
    Serial.println(F("Write pointer less than DATA_BUFFER_START_ADDR"));
    interrupts();
    return;
  }
  if(currentWritePointer > FRAM_SIZE) {  // ultimo dato in (FRAM_SIZE-2)-(FRAM_SIZE-1), massimo valore del write pointer FRAM_SIZE dopo la scrittura dell'ultimo dato
    Serial.println(F("Write pointer greater than FRAM size"));
    interrupts();
    return;
  }

// qui print del blocco dati fino a fine buffer
  for(bufferPointer = DATA_BUFFER_START_ADDR; bufferPointer < currentWritePointer; ) {
   // Serial.print(bufferPointer); Serial.flush(); Serial.print(" "); Serial.flush();
    for(i=0; i<NparamLog; i++) {
       temp32 = read_FRAM_long24(bufferPointer) ;
       Serial.print(temp32);Serial.flush(); Serial.print(" "); Serial.flush();
       bufferPointer+=NbytePerSingleLogData;
    }
    //-------------------------------------
    printVbias();
    //-------------------------------------
    Serial.println(); Serial.flush();
  }
  
  Serial.print(">"); Serial.flush();
}

void dataBufferDump(void) {
//  uint16_t currentWritePointer, bufferPointer;
  uint32_t currentWritePointer, bufferPointer;
  uint32_t temp32;

  Serial.println();
  Serial.println(F("DATA BUFFER DUMP ns:"));

//  currentWritePointer = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);
  currentWritePointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);

  if(currentWritePointer < DATA_BUFFER_START_ADDR) {
    Serial.println(F("Write pointer less than DATA_BUFFER_START_ADDR"));
    return;
  }
  if(currentWritePointer > FRAM_SIZE) {  // ultimo dato in (FRAM_SIZE-2)-(FRAM_SIZE-1), massimo valore del write pointer FRAM_SIZE dopo la scrittura dell'ultimo dato
    Serial.println(F("Write pointer greater than FRAM size"));
    return;
  }

  for(bufferPointer = DATA_BUFFER_START_ADDR; bufferPointer < currentWritePointer; bufferPointer += 2) {
    temp32 = ( ( (uint32_t)read_FRAM_u16(bufferPointer) ) * timeUnit_ps + 499)/1000;
//    temp32 = (uint32_t)( (unsigned)read_FRAM_long16(bufferPointer));
    Serial.println(temp32);
  }
}


void resetWritePointer(void) {
 // uint16_t temp16;
  uint32_t temp32;
  
  Serial.println();
  Serial.print(F("password (856221):"));
  temp32 = readULong32(0x0D);
  Serial.println();
  Serial.print(F("TT buffer "));
  if(temp32 != 856221) { Serial.print(F("non svuotato!")); return; }
  
//  write_FRAM_long16(DATA_BUFFER_WRITE_POINTER, DATA_BUFFER_START_ADDR);  
//  temp16 = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);
  write_FRAM_long32(DATA_BUFFER_WRITE_POINTER, DATA_BUFFER_START_ADDR);  
  temp32 = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);

//  if(temp16 != DATA_BUFFER_START_ADDR) {
  if(temp32 != DATA_BUFFER_START_ADDR) {
    Serial.print(F("mismatch, expected ")); Serial.print(DATA_BUFFER_START_ADDR); 
 //   Serial.print(F(", read ")); Serial.print(temp16);
    Serial.print(F(", read ")); Serial.print(temp32);
  }
  else  Serial.print("svuotato!");
  Serial.println();
}


void readT2Treg (void) {
 // byte temp8;
  
  Serial.println();
//  T2T = (uint16_t)XLR8_T2Treg_L + ((uint16_t)XLR8_T2Treg_H)*256;
  T2T = XLR8_T2Treg;
  Serial.print(T2T, DEC); Serial.print("  "); Serial.println(T2T, HEX);  
}

void setT2TcountLimit(void) {
  uint16_t ULongVal, ULongValR;
  uint32_t temp32ns;
 // int i;

  Serial.println();
// read EEPROM value
  ULongVal = read_FRAM_long16(EEPROM_T2TcountLimit_ADDR);
  temp32ns = (uint32_t)((uint32_t)ULongVal * timeUnit_ps + 499)/1000;
  Serial.print(F("\n\r T2TcountLimit in memory = "));
  Serial.print(temp32ns);
  Serial.println(" ns");

//  ULongVal = (uint16_t)XLR8_T2TcountLimitL + (uint16_t)XLR8_T2TcountLimitH*256;
  ULongVal = XLR8_T2TcountLimit;
  temp32ns = (uint32_t)((uint32_t)ULongVal * timeUnit_ps + 499)/1000;
  Serial.print(F("\n\r T2TcountLimit in XLR8_Trigger = "));
  Serial.print(temp32ns);
  Serial.println(" ns");

  Serial.print(F("--> Set T2TcountLimit (max 500000 ns): "));
  temp32ns = readULong32_EndByAnyChar();
  ULongVal = (uint16_t)((temp32ns * 1000)/timeUnit_ps);

 // scrivi il valore nella FRAM
  write_FRAM_long16(EEPROM_T2TcountLimit_ADDR, ULongVal);
// scrivilo in XLR8_Trigger
//  XLR8_T2TcountLimitL = (uint8_t)(ULongVal & 0xFF);
//  XLR8_T2TcountLimitH = (uint8_t)((ULongVal/256) & 0xFF);
  XLR8_T2TcountLimit = ULongVal;

// rileggilo dalla FRAM
  ULongValR = read_FRAM_long16(EEPROM_T2TcountLimit_ADDR);
  if( ULongValR != ULongVal ) {
    Serial.print(F("T2TcountLimit on memory mismatch: read "));
    Serial.print(ULongValR, DEC);
    Serial.print(F("  instead of "));
    Serial.print(ULongVal, DEC);
  }
  else {
    Serial.println();
    Serial.print(F(" ... Written in memory"));
  }
    Serial.println();

// rileggi da XLR8_Trigger
//  ULongValR = (uint16_t)XLR8_T2TcountLimitL + (uint16_t)XLR8_T2TcountLimitH*256;
  ULongVal = XLR8_T2TcountLimit;
  if( ULongValR != ULongVal ) {
    Serial.print(F("T2TcountLimit on XLR8_Trigger mismatch: read "));
    Serial.print(ULongValR, DEC);
    Serial.print(F("  instead of "));
    Serial.print(ULongVal, DEC);
  }
  else {
    Serial.print(F(" ... Written in XLR8_Trigger"));
  }
   
  Serial.println();
}


void setTriggerEna(byte mask) {
  /*
// bit# in XLR8_TrigConfReg register
#define IN0_ENA 0
#define IN1_ENA 1
#define IN2_ENA 2
#define IN3_ENA 3
#define TRIGGER_ENA 4
*/

 bitClear(XLR8_TrigConfReg, IN0_ENA);
 if(bitRead(mask, 0)) bitSet(XLR8_TrigConfReg, IN0_ENA);
 
 bitClear(XLR8_TrigConfReg, IN1_ENA);
 if(bitRead(mask, 1)) bitSet(XLR8_TrigConfReg, IN1_ENA);
 
 bitClear(XLR8_TrigConfReg, IN2_ENA);
 if(bitRead(mask, 2)) bitSet(XLR8_TrigConfReg, IN2_ENA);
 
 bitClear(XLR8_TrigConfReg, IN3_ENA);
 if(bitRead(mask, 3)) bitSet(XLR8_TrigConfReg, IN3_ENA);

 if( (XLR8_TrigConfReg & 0xF) == mask ) Serial.println("OK!");
 else {
  Serial.print("** MISMATCH: written "); Serial.print(mask&0xF, BIN); Serial.print(",  read "); Serial.println(XLR8_TrigConfReg & 0xF, BIN);
 }
 
}


void resetTrigger(void) {
  Serial.print("\n\r-- non implementata\n\r");
//  Serial.print("\n\r-- Reset TRIGGER\n\r");
  //eventReset
}

void EnaDisaToc(void) {
  uint8_t tocFRAM, tocFRAMbit, tocFPGA, tocR, toc;

// read TOC ena status
  tocFRAM = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);
  tocFRAMbit = bitRead(tocFRAM, TOCenable);
  tocFPGA = bitRead(XLR8_TrigConfReg1, TOCenable);

  Serial.print(F("\n\r TOC = FRAM "));
  Serial.print(tocFRAMbit);
  Serial.print(F(", FPGA "));
  Serial.println(tocFPGA);

  Serial.print(F("--> set Enable/Disable (=1/0) for TOC: "));
//  toc = readByte('\r');
  toc = readByte_EndByAnyChar();
  toc = toc & 0x1;
  Serial.print(F("\n\r toc = "));
  Serial.print(toc);
  Serial.print(F("\n\r"));

 // scrivi nella FRAM e nella FPGA e rileggila per check
  if(toc==1) { 
    bitSet(tocFRAM, TOCenable); 
    bitSet(XLR8_TrigConfReg1, TOCenable);
  }
  else { 
    bitClear(tocFRAM, TOCenable); 
    bitClear(XLR8_TrigConfReg1, TOCenable);
  }
 write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, tocFRAM);

 tocR = bitRead(read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR), TOCenable);
 
  if( tocR!= toc ) {
    Serial.print(F("TOC enable data on memory mismatch: read "));
    Serial.print(tocR);
    Serial.print(F("  instead of "));
    Serial.print(toc);
    Serial.print(F("\n\r"));
  }
  else {
    Serial.print(F("Written in memory.\n\r"));
  }
}

void setResetGlobalEnable(void) {
  uint8_t enadisaFRAM, enadisaFRAMbit, enadisaFPGA, enadisaR, enadisa;

// read enadisa status
  enadisaFRAM = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);
  enadisaFRAMbit = bitRead(enadisaFRAM, GLOBAL_ENABLE);
  enadisaFPGA = bitRead(XLR8_TrigConfReg1, GLOBAL_ENABLE);

  Serial.print(F("\n\r globalEnable = FRAM "));
  Serial.print(enadisaFRAMbit);
  Serial.print(F(", FPGA "));
  Serial.println(enadisaFPGA);

  Serial.print(F("--> set globalEnable (=1 ena, =0 disa): "));
  enadisa = readByte_EndByAnyChar();
  enadisa = enadisa & 0x1;
  Serial.print(F("\n\r globalEnable = "));
  Serial.print(enadisa);
  Serial.print(F("\n\r"));

 // scrivi nella FRAM e nella FPGA e rileggila per check
  if(enadisa==1) { 
    bitSet(enadisaFRAM, GLOBAL_ENABLE); 
    bitSet(XLR8_TrigConfReg1, GLOBAL_ENABLE);
  }
  else { 
    bitClear(enadisaFRAM, GLOBAL_ENABLE); 
    bitClear(XLR8_TrigConfReg1, GLOBAL_ENABLE);
  }
 write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, enadisaFRAM);

 enadisaR = bitRead(read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR), GLOBAL_ENABLE);
 
  if( enadisaR!= enadisa ) {
    Serial.print(F("globalEnable data on memory mismatch: read "));
    Serial.print(enadisaR);
    Serial.print(F("  instead of "));
    Serial.print(enadisa);
    Serial.print(F("\n\r"));
  }
  else {
    Serial.print(F("Written in memory.\n\r"));
  }
}


void writeEnableMask(void) {
  uint8_t mask, maskR;

  Serial.print(F("--> INPUT[7:0] enable, 8-bit binary pattern as 1010...: "));
 // mask = readByteBinary('\r');
  mask = readByteBinary_EndByAnyChar();
 // mask = mask & 0xF;
  Serial.print(F("\n\r <ena[7:0]> = "));
  print_binary(mask, 8);
  Serial.print(F("\n\r"));

//  maskR = (XLR8_TrigConfReg & 0xF0) | mask;  // rimpiazza i bit di mask appena immessi
//  XLR8_TrigConfReg = maskR;
    XLR8_Trig_mask = mask;
  
 // scrivi mask nella EEPROM e rileggila per check
//  write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, maskR);
  write_FRAM_byte(EEPROM_XLR8_Trig_mask_ADDR, mask);

  maskR = read_FRAM_byte(EEPROM_XLR8_Trig_mask_ADDR);

  if( maskR!=XLR8_Trig_mask ) {
    Serial.print(F("XLR8_TrigConfReg and FRAM mismatch: XLR8_TrigConfReg "));
    print_binary(XLR8_Trig_mask, 8);
    Serial.print(F("  FRAM "));
    print_binary(maskR, 8);
    Serial.print(F("\n\r"));
  }
  else {
    Serial.print(F("Written in memory.\n\r"));
  }
}



void readEnableMask(void) {
    Serial.println(F("Enables <trigger><ena[3:0]> (binary): "));
    Serial.print(F("FRAM = ")); print_binary(read_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR), 5); Serial.println();
    Serial.print(F("XLR8_TrigConfReg = ")); print_binary(XLR8_TrigConfReg & 0x1F, 5); Serial.println();
//    Serial.print(F("XLR8_TrigConfReg = ")); Serial.println(XLR8_TrigConfReg & 0x1F, BIN);
    Serial.println();
}


void proveFramReg(void) {
  byte toc;

Serial.println();
Serial.println("XLR8_FRAM:");
  Serial.print("byte letto: "); Serial.println(XLR8_FRAM, DEC);
  Serial.print("byte da scrivere: ");
  toc = readByte_EndByAnyChar();
  Serial.println();
  XLR8_FRAM = toc;
  Serial.print("byte letto: "); Serial.println(XLR8_FRAM, DEC);

  Serial.print("bit set # 4 = 16 "); bitSet(XLR8_FRAM, 4);
  Serial.print("byte letto: "); Serial.println(XLR8_FRAM, DEC);
  Serial.print("bit clear # 4 = 16 "); bitClear(XLR8_FRAM, 4);
  Serial.print("byte letto: "); Serial.println(XLR8_FRAM, DEC);
}


#if defined(FRAM_64k)
void testFRAM(void) {
  byte temp8;
  uint16_t temp16;
  uint32_t temp32;

  write_FRAM_byte(0x1FFF, 0xAA);
  if((temp8=read_FRAM_byte(0x1FFF)) != 0xAA) Serial.println("FRAM byte mismatch");
  else Serial.println("FRAM byte R/W OK!");
  Serial.print("expected; 0xAA"); Serial.print("  read: "); Serial.println(temp8, HEX);
  
  write_FRAM_long16(0x0, 0xAABB);
  if((temp16=read_FRAM_long16(0x0)) != 0xAABB) Serial.println("FRAM long mismatch");
  else Serial.println("FRAM long R/W OK!");
  Serial.print("expected; 0xAABB"); Serial.print("  read: "); Serial.println(temp16, HEX);
  
  write_FRAM_long32(0xFFF, 0xDEADFACE);
  if((temp32=read_FRAM_long32(0xFFF)) != 0xDEADFACE) Serial.println("FRAM long32 mismatch");
  else Serial.println("FRAM long32 R/W OK!");
  Serial.print("expected; 0xDEADFACE"); Serial.print("  read: "); Serial.println(temp32, HEX);

  Serial.println("==== int16 all addresses FRAM R/W TEST ====\n\r");
  Serial.println("write ... \n\r");

  for(temp16=0; temp16<(0x1FFF - 3); temp16 = temp16 + 4) {
    write_FRAM_long32(temp16, 0xDEADFACE - temp16);
  }
  
  Serial.println("read ... \n\r");

  for(temp16=0; temp16<(0x1FFF - 3); temp16+=4) {
    temp32=read_FRAM_long32(temp16);
    if(temp32 != (0xDEADFACE - temp16)) { 
      Serial.print("No match, addr="); Serial.print(temp16, HEX); Serial.print(" read="); Serial.print(temp16, HEX); 
      Serial.print(" exp="); Serial.println(0xDEADFACE - temp16, HEX);
      break;
      }
  }
}
#elif defined(FRAM_8M)
void testFRAM(void) {
  byte temp8;
  uint16_t temp16;
  uint32_t addr32;
  uint32_t temp32;

  Serial.println("==== FRAM R/W TEST ====\n\r");

 // while(1) {digitalWrite(KEYB_SH1_LD0_PIN, 1); Serial.println(read_FRAM_statusRegister(), BIN); digitalWrite(KEYB_SH1_LD0_PIN, 0); delay(800);}
  
 // Serial.print("StatusRegister: "); Serial.println(read_FRAM_statusRegister(), BIN);  
  write_FRAM_byte(0x1FFF, 0xAA);
//  Serial.print("StatusRegister: "); Serial.println(read_FRAM_statusRegister(), BIN);  
  if((temp8=read_FRAM_byte(0x1FFF)) != 0xAA) Serial.println("FRAM byte mismatch");
  else Serial.println("FRAM byte R/W OK!");
  Serial.print("expected: 0xAA"); Serial.print("  read: "); Serial.println(temp8, HEX);
  
  write_FRAM_long16(0x0, 0xAABB);
  if((temp16=read_FRAM_long16(0x0)) != 0xAABB) Serial.println("FRAM long16 mismatch");
  else Serial.println("FRAM long16 R/W OK!");
  Serial.print("expected: 0xAABB"); Serial.print("  read: "); Serial.println(temp16, HEX);
  
  write_FRAM_long32(0, 0xDEADFACE);
  if((temp32=read_FRAM_long32(0)) != 0xDEADFACE) Serial.println("FRAM long32 @ 0 mismatch");
  else Serial.println("FRAM long32 @ 0 R/W OK!");
  Serial.print("expected: 0xDEADFACE"); Serial.print("  read: "); Serial.println(temp32, HEX);

  write_FRAM_long32(0xFFF-3, 0xDEADFACE);
  if((temp32=read_FRAM_long32(0xFFFFFF-3)) != 0xDEADFACE) Serial.println("FRAM long32 @ eom mismatch");
  else Serial.println("FRAM long32 @ eom R/W OK!");
  Serial.print("expected: 0xDEADFACE"); Serial.print("  read: "); Serial.println(temp32, HEX);

  Serial.println("==== int24 all addresses FRAM R/W TEST ====\n\r");
  Serial.println("write ... \n\r");

  for(addr32=0; addr32<(FRAM_SIZE-4); addr32 = addr32 + 3) {
    write_FRAM_long24(addr32, 0xDEADFA - addr32);
  }
  
  Serial.println("read ... \n\r");

  for(addr32=0; addr32<(FRAM_SIZE-4); addr32+=3) {
    temp32=read_FRAM_long24(addr32);
    if(temp32 != (0xDEADFA - addr32)) { 
      Serial.print("No match, addr="); Serial.print(addr32, HEX); Serial.print(" read="); Serial.print(addr32, HEX); 
      Serial.print(" exp="); Serial.println(0xDEADFA - addr32, HEX);
     // break;
      }
  }
}
#endif



void saveCountersInFRAM(void) {
  uint32_t temp32;

//  temp32 = readTimeCnt_ms(1);
//Serial.print(time_in_ms); Serial.print(' ');
  write_FRAM_long32(EEPROM_XLR8_TimeCnt_ms_ADDR, time_in_ms);
  LOG2FRAM(tempoSecondi);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(tempoSecondi); Serial.print(' ');

//  temp32 = readTriggerCnt(0);
  write_FRAM_long24(EEPROM_XLR8_TrigCnt_ADDR, numTrigger);
  LOG2FRAM(numTrigger);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(numTrigger); Serial.print(' ');

  temp32 = readInputCnt(0,0);
  write_FRAM_long24(EEPROM_XLR8_IN0Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,1);
  write_FRAM_long24(EEPROM_XLR8_IN1Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,2);
  write_FRAM_long24(EEPROM_XLR8_IN2Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,3);
  write_FRAM_long24(EEPROM_XLR8_IN3Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');

  temp32 = readInputCnt(0,4);
  write_FRAM_long24(EEPROM_XLR8_IN4Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,5);
  write_FRAM_long24(EEPROM_XLR8_IN5Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,6);
  write_FRAM_long24(EEPROM_XLR8_IN6Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,7);
  write_FRAM_long24(EEPROM_XLR8_IN7Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,8);
  write_FRAM_long24(EEPROM_XLR8_And0_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');

  temp32 = readInputCnt(0,9);
  write_FRAM_long24(EEPROM_XLR8_And1_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.print(read_FRAM_long24(write_pointer-3)); Serial.print(' ');
  Serial.print(temp32); Serial.print(' ');
  
  temp32 = readInputCnt(0,10);
  write_FRAM_long24(EEPROM_XLR8_And2_ADDR, temp32);
  LOG2FRAM(temp32);
//  Serial.println(read_FRAM_long24(write_pointer-3));
  Serial.print(temp32); Serial.print(' ');

  Serial.println();
}

/*
void saveCountersInFRAM(void) {
  uint32_t temp32;

  temp32 = readTimeCnt_ms(1);
  write_FRAM_long32(EEPROM_XLR8_TimeCnt_ms_ADDR, temp32);  // qui avviene il latch di tutti i dati del readout
  LOG2FRAM(temp32);

  temp32 = readTriggerCnt(0);
  write_FRAM_long32(EEPROM_XLR8_TrigCnt_ADDR, temp32);
  LOG2FRAM(temp32);

  temp32 = readInputCnt(0,0);
  write_FRAM_long32(EEPROM_XLR8_IN0Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,1);
  write_FRAM_long32(EEPROM_XLR8_IN1Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,2);
  write_FRAM_long32(EEPROM_XLR8_IN2Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,3);
  write_FRAM_long32(EEPROM_XLR8_IN3Cnt_ADDR, temp32);
  LOG2FRAM(temp32);

  temp32 = readInputCnt(0,4);
  write_FRAM_long32(EEPROM_XLR8_IN4Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,5);
  write_FRAM_long32(EEPROM_XLR8_IN5Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,6);
  write_FRAM_long32(EEPROM_XLR8_IN6Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,7);
  write_FRAM_long32(EEPROM_XLR8_IN7Cnt_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,8);
  write_FRAM_long32(EEPROM_XLR8_And0_ADDR, temp32);
  LOG2FRAM(temp32);

  temp32 = readInputCnt(0,9);
  write_FRAM_long32(EEPROM_XLR8_And1_ADDR, temp32);
  LOG2FRAM(temp32);
  
  temp32 = readInputCnt(0,10);
  write_FRAM_long32(EEPROM_XLR8_And2_ADDR, temp32);
  LOG2FRAM(temp32);
}
*/
#define TRIGGER_RESET 5
#define TIME_AND_COUNTERS_RESET 4
#define LATCH_READOUT_DATA 3
#define ENA_FLASH 2
#define GLOBAL_ENABLE 1

void resetFPGA(void) {
  uint8_t temp8, temp8_1;

  noInterrupts();
  
   bitClear(XLR8_TrigConfReg, TRIGGER_ENA); // disa TRIGGER
   bitClear(XLR8_TrigConfReg1, GLOBAL_ENABLE); 
   bitSet(XLR8_TrigConfReg1, TIME_AND_COUNTERS_RESET); 
   bitSet(XLR8_TrigConfReg1, TRIGGER_RESET); 
   bitClear(XLR8_TrigConfReg1, LATCH_READOUT_DATA); 

   // T2TcountLimit
 //  XLR8_T2TcountLimitL = read_FRAM_byte(EEPROM_T2TcountLimit_ADDR);
 //  XLR8_T2TcountLimitH = read_FRAM_byte(EEPROM_T2TcountLimit_ADDR+1);

   // counters
   XLR8_TrigCnt_rdout = 0;
   XLR8_TimeCnt_ms_rdout = 0;
   XLR8_EvCnt0_rdout = 0;
   XLR8_EvCnt1_rdout = 0;
   XLR8_EvCnt2_rdout = 0;
   XLR8_EvCnt3_rdout = 0;
   XLR8_EvCnt4_rdout = 0;
   XLR8_EvCnt5_rdout = 0;
   XLR8_EvCnt6_rdout = 0;
   XLR8_EvCnt7_rdout = 0;

   XLR8_And0_rdout = 0;
   XLR8_And1_rdout = 0;
   XLR8_And2_rdout = 0;

   saveCountersInFRAM();

// non servirebbe, ma non fa male
   XLR8_And0_mask = read_FRAM_byte(EEPROM_XLR8_And0_mask_ADDR);
   XLR8_And1_mask = read_FRAM_byte(EEPROM_XLR8_And1_mask_ADDR);
   XLR8_And2_mask = read_FRAM_byte(EEPROM_XLR8_And2_mask_ADDR);
   XLR8_Trig_mask = read_FRAM_byte(EEPROM_XLR8_Trig_mask_ADDR);
// ---------------------------

   // conf regs
   temp8 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR);
   bitSet(temp8, TRIGGER_ENA);
   write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, temp8);
//   XLR8_TrigConfReg = temp8;

//   bitSet(XLR8_TrigConfReg, TRIGGER_ENA); // ena TRIGGER

   temp8_1 = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);
   bitSet(temp8_1, GLOBAL_ENABLE); 
   bitClear(temp8_1, TIME_AND_COUNTERS_RESET); 
   bitClear(temp8_1, TRIGGER_RESET); 
   bitClear(temp8_1, LATCH_READOUT_DATA); 
   // per essere sicuri di resettare il bit di reset time
   write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, temp8_1); 

   XLR8_TrigConfReg1 = temp8_1;
   XLR8_TrigConfReg = temp8;
 // Serial.println(temp8, BIN);

// reset puntatore buffer
   write_FRAM_long32(DATA_BUFFER_WRITE_POINTER, DATA_BUFFER_START_ADDR);  

   interrupts();
}




/*
  write_FRAM_long32(EEPROM_XLR8_TrigCnt_ADDR, readTriggerCnt(0));
  write_FRAM_long32(EEPROM_XLR8_IN0Cnt_ADDR, readInputCnt(0,0));
  write_FRAM_long32(EEPROM_XLR8_IN1Cnt_ADDR, readInputCnt(0,1));
  write_FRAM_long32(EEPROM_XLR8_IN2Cnt_ADDR, readInputCnt(0,2));
  write_FRAM_long32(EEPROM_XLR8_IN3Cnt_ADDR, readInputCnt(0,3));
*/

// da aggiornare la parte dell'OLED

void dispInCntPage(uint8_t offset) {
    unsigned long temp32;
    byte temp8mode;
    char str[10];

      if(INIT_DISP == true) lcd.clear();          // Clear the display

      if(INIT_DISP == true) { 
        lcd.setCursor(0,0); 
        lcd.print(offset); lcd.print(F(":")); 
        sprintf(str, "%7lu", readInputCnt(1,offset));
        lcd.print(str); 
        }
      else if(read_FRAM_long32(EEPROM_XLR8_IN0Cnt_ADDR + 4*offset)!= readInputCnt(1,offset)) {
        lcd.setCursor(2,0); 
        sprintf(str, "%7lu", readInputCnt(1,offset));
        lcd.print(str); 
        }
      
      if(INIT_DISP == true) { 
        lcd.setCursor(0,1); 
        lcd.print(offset+1); lcd.print(F(":")); 
       sprintf(str, "%7lu", readInputCnt(0,1 + offset));
        lcd.print(str); 
        }
      else if(read_FRAM_long32(EEPROM_XLR8_IN1Cnt_ADDR + 4*offset)!= readInputCnt(0,1 + offset)){ 
        lcd.setCursor(2,1); 
        sprintf(str, "%7lu", readInputCnt(0,1 + offset));
        lcd.print(str); 
        }    
      
      if(INIT_DISP == true) { 
        lcd.setCursor(11,0); 
        lcd.print(offset+2); lcd.print(F(":")); 
        sprintf(str, "%7lu", readInputCnt(0,2 + offset));
        lcd.print(str); 
        }
      else if(read_FRAM_long32(EEPROM_XLR8_IN2Cnt_ADDR + 4*offset)!= readInputCnt(0,2 + offset)){ 
        lcd.setCursor(13,0); 
        sprintf(str, "%7lu", readInputCnt(0,2 + offset));
        lcd.print(str); 
        }    
      
      if(INIT_DISP == true) { 
        lcd.setCursor(11,1); 
        lcd.print(offset+3); lcd.print(F(":")); 
        sprintf(str, "%7lu", readInputCnt(0,3 + offset));
        lcd.print(str); 
        }
      else if(read_FRAM_long32(EEPROM_XLR8_IN3Cnt_ADDR + 4*offset)!= readInputCnt(0,3 + offset)){ 
        lcd.setCursor(13,1); 
        sprintf(str, "%7lu", readInputCnt(0,3 + offset));
        lcd.print(str); 
        }    

      if(INIT_DISP == true) { lcd.setCursor(0,3); lcd.print(F("Rm:")); }
      else { 
        lcd.setCursor(3,3); 
        lcd.print(F("   "));
        lcd.setCursor(3,3); 
        }    
      if(tempoSecondi!=0) {
         temp32 = (numTrigger*60 + tempoSecondi/2)/tempoSecondi;
         if(temp32 <= 999) lcd.print( temp32 );  
         else if(temp32<=10000) {
           lcd.print(temp32/1000); lcd.print('k'); lcd.print((temp32%1000)/100);
         }
         else if(temp32<=100000) {
           lcd.print(temp32/10000); lcd.print((temp32%10000)/1000);  lcd.print('k');
         }
         else lcd.print(F("***")); 
      }


      if(INIT_DISP == true) { lcd.setCursor(7,3); lcd.print(F("R1:")); }
      else { 
        lcd.setCursor(10,3); 
        lcd.print(F("   "));
        lcd.setCursor(10,3); 
        }    
      if(eventiPerMinuto<=999) lcd.print(eventiPerMinuto);  
      else if(eventiPerMinuto<=10000) {
        lcd.print(eventiPerMinuto/1000); lcd.print('k'); lcd.print((eventiPerMinuto%1000)/100);
      }
      else if(eventiPerMinuto<=100000) {
        lcd.print(eventiPerMinuto/10000); lcd.print((eventiPerMinuto%10000)/1000); lcd.print('k');
      }
      else lcd.print(F("***")); 


      if(INIT_DISP == true) { 
        lcd.setCursor(0,2); 
        lcd.print(F("T:")); 
        sprintf(str, "%7lu", tempoSecondi);
        lcd.print(str); 
        }
      else { 
        lcd.setCursor(2,2); 
     //   lcd.print(F("        "));
     //   lcd.setCursor(2,2); 
        sprintf(str, "%7lu", tempoSecondi);
        lcd.print(str); 
        }    
      
      if(INIT_DISP == true) { 
        lcd.setCursor(11,2); 
        lcd.print(F("E:")); 
        sprintf(str, "%7lu", numTrigger);
        lcd.print(str); 
        }
      else { 
        lcd.setCursor(13,2); 
       // lcd.print(F("        "));
       // lcd.setCursor(12,2); 
        sprintf(str, "%7lu", numTrigger);
        lcd.print(str); 
        }    


//      temp8mode = (XLR8_TrigConfReg & 0b11100000)>>5;
      temp8mode = XLR8_TrigConfReg;

      if(INIT_DISP == true) { lcd.setCursor(14,3); lcd.print(F("Tr:")); }
      else { 
 //       lcd.setCursor(17,3); 
 //       lcd.print(F("   "));
        lcd.setCursor(17,3); 
        }    
      if(bitRead(temp8mode,7)) lcd.print(F("AND"));
      else lcd.print(F("OR "));
     // if(bitRead(temp8mode,5)) lcd.print(F("TT"));
    //  else lcd.print(F("TTo"));
    //  else if(temp8mode == TRIGGER_MODE_MU_DECAY) lcd.print(F("4Ld"));
    //  else if(temp8mode == TRIGGER_MODE_FLOOR)    lcd.print(F("4Lf"));
    //  else lcd.print(F("UNK"));

      INIT_DISP = false;
}




void dispInfo0(void) {

  #ifdef LCD_I2C
    dispInCntPage(0);

  #endif

  #ifdef OLED128x64

  #endif
}

void dispInfo1(void) {

  #ifdef LCD_I2C
    dispInCntPage(4);
  #endif

  #ifdef OLED128x64

  #endif
}

// portarli da 4 a 32 con i caratteri speciali
#define linesPerColumn 32
#define linesPerChar 8


void plotColumn(uint8_t column, uint32_t entry){
  uint8_t row;

  if(entry > linesPerColumn) entry = linesPerColumn;
  row = 3;
  lcd.setCursor(column, row);
  if(entry==0) lcd.print((char)0xFE);
  else {
    while(entry >= linesPerChar) {
      lcd.print((char)0xFF);
      entry -= linesPerChar;
      lcd.setCursor(column, --row);
    }
    if(entry!=0) lcd.write(byte(entry));
  }
}

//uint16_t prevWritePointer = 0;
uint32_t prevWritePointer = 0;
uint32_t zoomHistoFactor = 0;
boolean doRedraw = false;
#define lasciaFloor true
#define togliFloor false
#define binWidth 1000
#define startTime 890

// per la sottrazione del fondo si assume un T2T_limit = 100 us, 20-200 per il calcolo del fondo
void plotMu(boolean FloorOrNotFloor, uint32_t forceEntryPerLine, boolean doRedraw) {
//  uint16_t currentWritePointer, bufferPointer;
  uint32_t currentWritePointer, bufferPointer;
  uint32_t temp32;
  uint32_t entries[20], floorData, maxNentry, entryPerLine;
  uint8_t n;
 // int8_t rowIndex;
 // const char binPlot=0xFF;

//  currentWritePointer = read_FRAM_long16(DATA_BUFFER_WRITE_POINTER);
  currentWritePointer = read_FRAM_long32(DATA_BUFFER_WRITE_POINTER);
  
  if( (currentWritePointer == prevWritePointer) & (doRedraw == false) ) return; // esce se non ci sono nuovi dati letti dalla FIFO
  else prevWritePointer = currentWritePointer;

  if(currentWritePointer < DATA_BUFFER_START_ADDR) {
    return;
  }
  if(currentWritePointer > FRAM_SIZE) {  // ultimo dato in (FRAM_SIZE-2)-(FRAM_SIZE-1), massimo valore del write pointer FRAM_SIZE dopo la scrittura dell'ultimo dato
    return;
  }

//Serial.println();
//     Serial.print(  forceEntryPerLine);
 
  floorData=0; 
  for(n=0; n<20; n++) entries[n]=0; 
  
  for(bufferPointer = DATA_BUFFER_START_ADDR; bufferPointer < currentWritePointer; bufferPointer += 2) {
    temp32 = ( ( (uint32_t)read_FRAM_u16(bufferPointer) ) * timeUnit_ps + 499)/1000;
    if(temp32<(startTime+binWidth) ) entries[0]++;
    else if(temp32<(startTime+binWidth+binWidth)) entries[1]++;
    else if(temp32<(startTime+binWidth+2*binWidth)) entries[2]++;
    else if(temp32<(startTime+binWidth+3*binWidth)) entries[3]++;
    else if(temp32<(startTime+binWidth+4*binWidth)) entries[4]++;
    else if(temp32<(startTime+binWidth+5*binWidth)) entries[5]++;
    else if(temp32<(startTime+binWidth+6*binWidth)) entries[6]++;
    else if(temp32<(startTime+binWidth+7*binWidth)) entries[7]++;
    else if(temp32<(startTime+binWidth+8*binWidth)) entries[8]++;
    else if(temp32<(startTime+binWidth+9*binWidth)) entries[9]++;
    else if(temp32<(startTime+binWidth+10*binWidth)) entries[10]++;
    else if(temp32<(startTime+binWidth+11*binWidth)) entries[11]++;
    else if(temp32<(startTime+binWidth+12*binWidth)) entries[12]++;
    else if(temp32<(startTime+binWidth+13*binWidth)) entries[13]++;
    else if(temp32<(startTime+binWidth+14*binWidth)) entries[14]++;
    else if(temp32<(startTime+binWidth+15*binWidth)) entries[15]++;
    else if(temp32<(startTime+binWidth+16*binWidth)) entries[16]++;
    else if(temp32<(startTime+binWidth+17*binWidth)) entries[17]++;
    else if(temp32<(startTime+binWidth+18*binWidth)) entries[18]++;
    else if(temp32<(startTime+binWidth+19*binWidth)) entries[19]++;
    else floorData++;    
  }
  floorData = floorData/80;

// tolgo il fondo se voluto dal chiamante
if(FloorOrNotFloor == togliFloor) {
  for(n=0;n<20;n++) {
//    Serial.print(entries[n]); Serial.print(" ");
    if(entries[n]>=floorData) entries[n] -= floorData;
    else entries[n]=0;
//    Serial.println(entries[n]);
  }
}
  
// massimo delle entries in un bin per normalizzare il display
  maxNentry=0;
  for(n=0;n<20;n++) {
    if(entries[n]>maxNentry) maxNentry = entries[n];
  }

if(forceEntryPerLine==0) {
  if(maxNentry <= linesPerColumn) entryPerLine = 1;
  else {
    entryPerLine = maxNentry/linesPerColumn;
    if( (maxNentry%linesPerColumn) != 0 ) entryPerLine++;
  }
}
else entryPerLine = forceEntryPerLine;

// display
  lcd.clear();          // Clear the display

  for(n=0; n<20; n++) {
      temp32 = entries[n];
// bin normalizzati
      if((temp32 < entryPerLine) & (temp32 != 0)) temp32=1;  // se non è zero ma rischia di essere visual. come 0 --> = 1
      else temp32 /= entryPerLine;
 //   Serial.println(temp32);   
      plotColumn(n, temp32);
  }
  INIT_DISP = false;

  /*
  if(disp_mode==3) {
      lcd.setCursor(17, 0);
      lcd.print("raw");
  }
  if(disp_mode==4) {
      lcd.setCursor(13, 0);
      lcd.print("noFloor");
  }
  lcd.setCursor(19, 1);
  */
  //if(zoomHistoFactor!=0) lcd.print("Z");
 // else lcd.print(" ");
}


void dispInfo3(void) {
  plotMu(lasciaFloor, 0, false);
  lcd.setCursor(17, 0);
  lcd.print("raw");
}

void dispInfo4(void) {
  plotMu(togliFloor, 0, false);
  lcd.setCursor(13, 0);
  lcd.print("noFloor");
}


void dispInfo2(void) {
    unsigned long temp32;
    char str_temp[6];
   // float tempf;
    byte temp8, i;
    char str[10];

    #ifdef LCD_I2C
      if(INIT_DISP == true) lcd.clear();  // Clear the display
// TOC
      lcd.setCursor(0,0);   // Set the cursor to col 0, row 0
      lcd.print(F("TOC:"));
      if(bitRead(XLR8_TrigConfReg1, TOCenable)) {
        lcd.print(F("on"));
        if(bitRead(XLR8_TrigConfReg1, TOCsourceSel)) lcd.print(F("TRG"));
        else lcd.print(F("AND"));
      }
      else lcd.print(F("DISA "));
// buffer
      lcd.setCursor(11,0);   // Set the cursor to col 8, row 0
      lcd.print(F("Buf:"));
//      sprintf(str, "%5d", (read_FRAM_long16(DATA_BUFFER_WRITE_POINTER) - DATA_BUFFER_START_ADDR)/2);
      sprintf(str, "%5ld", (read_FRAM_long32(DATA_BUFFER_WRITE_POINTER) - DATA_BUFFER_START_ADDR)/NbytePerLog);
      lcd.print(str); 
    //  lcd.print((read_FRAM_long16(DATA_BUFFER_WRITE_POINTER) - DATA_BUFFER_START_ADDR)/2);
      lcd.setCursor(11,1);   // Set the cursor to col 8, row 0
      lcd.print(F("of"));
      lcd.setCursor(14,1);   // Set the cursor to col 8, row 0
      lcd.print( (FRAM_SIZE-1 - DATA_BUFFER_START_ADDR + 1)/NbytePerLog);
/*
// T-T
      lcd.setCursor(10,2);   
      lcd.print(F("T2T:"));
      temp32 = ((uint32_t)(XLR8_T2TcountLimit) * timeUnit_ps + 499)/1000;
      sprintf(str, "%6lu", temp32);
      lcd.print(str); 
    //  lcd.print(temp32);
    //  lcd.print(F("n"));
*/
// S1 (THR)
      lcd.setCursor(0,2);  
      lcd.print(F("S1:"));
      Vadc1 = analogRead(Vthr1);
      Vadc1 = (int16_t) ((((int32_t)Vadc1 - 520)*489)/100);  // 4.89 mV/step, 2.50 V Vref
      sprintf(str, "%4d", Vadc1);
      lcd.print(str); 
      lcd.print(F("mV"));
// S2 (THR)
      lcd.setCursor(11,2);   
      lcd.print(F("S2:"));
      Vadc2 = analogRead(Vthr2);
      Vadc2 = (int16_t) ((((int32_t)Vadc2 - 520)*489)/100);  // 4.89 mV/step, 2.50 V Vref
      sprintf(str, "%4d", Vadc2);
      lcd.print(str); 
      lcd.print(F("mV"));
 // Vb     
      lcd.setCursor(0,1);   // Set the cursor to col 0, row 1
      lcd.print(F("Vb:"));
      temp32 = (unsigned long)analogRead(Vbias);
      temp32 = (temp32*480)/1000;  // legge Vbias/10, step 5 mV, quindi *5 e trovo i mV, *10 trovo la vera Vbias in mV  METTO 496/10 per compensare errori partitore, /100 per risoluzione 0.1V

     // 1st param is the var, 2nd param is num of digits, 3rd param is digits after comma; float value is copied onto str_temp
      dtostrf((float)temp32/10, 2, 1, str_temp);
      lcd.print(str_temp); lcd.print(F("V"));
// INP ena
      temp8 = XLR8_Trig_mask;
      lcd.setCursor(0,3);   // Set the cursor to col 0, row 1
      lcd.print(F("InpEnabled:"));

    for(i=0; i<8; i++)  {
      if(bitRead(temp8, i)) lcd.print(i);
      else lcd.print(F("x"));
    }
    /*
      if(bitRead(XLR8_Trig_mask, 7)) lcd.print(F("7"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 6)) lcd.print(F("6"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 5)) lcd.print(F("5"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 4)) lcd.print(F("4"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 3)) lcd.print(F("3"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 2)) lcd.print(F("2"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 1)) lcd.print(F("1"));
      else lcd.print(F("x"));
      if(bitRead(XLR8_Trig_mask, 0)) lcd.print(F("0"));
      else lcd.print(F("x"));
      */
/*
// trigger ena/disa
      lcd.setCursor(12,3);   // Set the cursor to col 0, row 1
      lcd.print(F("Trg:"));
      if(bitRead(XLR8_TrigConfReg, TRIGGER_ENA)) lcd.print(F("ENA "));
      else lcd.print(F("DISA"));
*/
      INIT_DISP = false;
#endif
}


void dispInfo(void) {
   if(disp_mode == 0) {
      dispInfo0();
    //  prevWritePointer = 0; // per abilitare il refresh di disp_mode2
   }
   else if(disp_mode == 1) {
      dispInfo1();
   //   prevWritePointer = 0; // per abilitare il refresh di disp_mode2
   }
   else if(disp_mode == 2) {
      dispInfo2();
   }
   else if(disp_mode == 3) {
      dispInfo3();
   }
   else if(disp_mode == 4) {
      dispInfo4();
   }
}


void readDispMode(void) {
  disp_mode = read_FRAM_byte(EEPROM_disp_mode_ADDR);
  if( (disp_mode<0) || (disp_mode >= nModes) ) {
    disp_mode = 0;
    write_FRAM_byte(EEPROM_disp_mode_ADDR, 0);
  }
}

/*
void histoZoomIn(void) {
  if(zoomHistoFactor != 0) {
    zoomHistoFactor--;
    while(digitalRead(TRIG_SEL_PIN)==0) delay(100);
    delay(100);
    while(digitalRead(TRIG_SEL_PIN)==0) delay(100);
    delay(100);
    plotMu(lasciaFloor, zoomHistoFactor, true);
  //  Serial.print(  zoomHistoFactor);
  }
}

void histoZoomOut(void) {
    zoomHistoFactor++;
    while(digitalRead(TOC_PIN)==0) delay(100);
    delay(100);
    while(digitalRead(TOC_PIN)==0) delay(100);
    delay(100);
    plotMu(lasciaFloor, zoomHistoFactor, true);  
   // Serial.print(  zoomHistoFactor);
}
*/

//#define SingleKey 0
//#define KEYBOARD 1

// con display a 4x20
//    il tasto DISP farà rolling + luce spenta/accesa con pressione lunga
boolean lcdOff=false;

void rollDispMode(void) {
  uint8_t tempoPressione;
  boolean loopCond;
  
  #define stepPressione_ms 20
  #define unSecondo 1000/stepPressione_ms
  #define quattroSecondi 4000/stepPressione_ms

  tempoPressione = unSecondo;

  scanKeybStable(); // aggiorna keybConnected flag
  do {
    delay(20);
    if(tempoPressione != 0) tempoPressione--;
    else break;
    if(keybConnected==true) { scanKeybStable(); loopCond = (dispKeyPressed==true); }
//    else loopCond = (digitalRead(ROLL_LIGHT_RESET_PIN)==0);
  }
  while(loopCond==true); // attendi la fine della pressione del tasto di roll

  if(tempoPressione != 0)  {
    if(lcdOff==true) { lcd.backlight(); lcdOff=false; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);} // turn on backlight
    else {
       INIT_DISP = true;
       disp_mode++;  
       
     //  Serial.println("INC");
       
       if(disp_mode >= nModes) disp_mode = 0;
       prevWritePointer = 0; // per abilitare il refresh di disp_mode2/3
       lcd.clear();          // Clear the display
       lcd.setCursor(0,0);   // Set the cursor to col 0, row 0
       lcd.print(F("#")); lcd.print(disp_mode); delay(200);
  
       write_FRAM_byte(EEPROM_disp_mode_ADDR, disp_mode);
    }
  }
  else { 
    lcd.noBacklight(); lcdOff=true; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0); // turn off backlight
  }  // else
  
  tempoPressione = 10;
  do { // attendi la fine della pressione del tasto di roll
    delay(20);
    
    if(keybConnected==true) { scanKeybStable(); loopCond = (dispKeyPressed==true); }
//    else loopCond = (digitalRead(ROLL_LIGHT_RESET_PIN)==0);

    if(loopCond == false) { // conta solo quando il tasto non è premuto
      if(tempoPressione != 0) tempoPressione--;
    }
    else tempoPressione = 10; // altrimenti reinizializza il contatore
  //  Serial.print(tempoPressione); Serial.println(loopCond);
}
  while( !( (loopCond==false) & (tempoPressione == 0) ) ); 
/*  
  do { // attendi la fine della pressione del tasto di roll
    delay(200);
    if(keybConnected==true) { scanKeybStable(); loopCond = (dispKeyPressed==true); }
    else loopCond = (digitalRead(ROLL_LIGHT_RESET_PIN)==0);
  }
  while(loopCond==true); 
*/
//  delay(200); // assorbe eventuali rimbalzi, spero
}


/*
void rollTRIGGER(void) {
  uint8_t tempoPressione, tocFRAM;
  boolean loopCond;
  
  #define stepPressione_ms 20
  #define treSecondi 3000/stepPressione_ms
  #define mezzoSec 500/stepPressione_ms

  if(keybConnected==true) return;

  if(bitRead(XLR8_TrigConfReg, TRIGGER_MODE2)) {XLR8_TRIGGER_MODE_OR write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg);} // setta OR
  else {XLR8_TRIGGER_MODE_AND write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg);} // setta AND 

  tempoPressione = 10;
  do { // attendi la fine della pressione del tasto di rollTrig
    delay(20);
    
    loopCond = (digitalRead(TRIG_SEL_PIN)==0);

    if(loopCond == false) { // conta solo quando il tasto non è premuto
      if(tempoPressione != 0) tempoPressione--;
    }
    else tempoPressione = 10; // altrimenti reinizializza il contatore
  //  Serial.print(tempoPressione); Serial.println(loopCond);
  }
  while( !( (loopCond==false) & (tempoPressione == 0) ) ); 

}
*/
// con disp 4x20 mettere flag per stato TOC

void doReset(void)  {
    lcd.backlight();      // Backlight ON
    lcdOff = false;
    write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);
    lcd.clear();          // Clear the display
    lcd.print("0"); delay(1000); //"RESET ..."
    resetFPGA();  // reset processore
    wdt_enable(WDTO_30MS); while(1);
}

/*
byte resetKeyIndex=0;
void handleResetKey(void) {
  uint8_t tempoPressione, tocFRAM;
  boolean loopCond;
  
  #define stepPressione_ms 20
  #define treSecondi 3000/stepPressione_ms
  
  tempoPressione = treSecondi;

  do {
    delay(20);
    if(tempoPressione != 0) tempoPressione--;
    if(keybConnected==true) { scanKeybStable(); loopCond = (resetKeyPressed==true) & (tempoPressione != 0); }
    else loopCond = (tempoPressione != 0) & (digitalRead(TOC_PIN)==0);
  }
  while(loopCond==true); // attendi la fine della pressione del tasto di reset

  if(tempoPressione == 0)  {
    doReset();
  }
  else { // toggle ena/disa/onOR/onAND TOC
    // read TOC ena/type status
    tocFRAM = read_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR);

    if(resetKeyIndex == 0) {  // ena TOC
       bitSet(tocFRAM, TOCenable); 
       bitSet(tocFRAM, TOCsourceSel); 
       bitSet(XLR8_TrigConfReg1, TOCenable);
       bitSet(XLR8_TrigConfReg1, TOCsourceSel); // TOC on TRIGGER
    }
    else if(resetKeyIndex == 1) { // TOC on AND
         bitClear(tocFRAM, TOCsourceSel); 
         bitClear(XLR8_TrigConfReg1, TOCsourceSel);
    }
    else if(resetKeyIndex == 2) { // disa TOC
       bitClear(tocFRAM, TOCenable); 
       bitClear(XLR8_TrigConfReg1, TOCenable);
   }
   write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, tocFRAM);
   resetKeyIndex++; if(resetKeyIndex==3) resetKeyIndex=0;

  tempoPressione = 10;
  do { // attendi la fine della pressione del tasto di rollTrig
    delay(20);
    
    loopCond = (digitalRead(TOC_PIN)==0);

    if(loopCond == false) { // conta solo quando il tasto non è premuto
      if(tempoPressione != 0) tempoPressione--;
    }
    else tempoPressione = 10; // altrimenti reinizializza il contatore
  //  Serial.print(tempoPressione); Serial.println(loopCond);
  }
  while( !( (loopCond==false) & (tempoPressione == 0) ) ); 
  }
}
*/






/*
 // TrigConfReg bits
  wire TRG_MODE2 = TrigConfRreg[7]; // scelta tipo di trigger
  wire TRG_MODE1 = TrigConfRreg[6]; // scelta tipo di trigger
  wire TRG_MODE0 = TrigConfRreg[5]; // scelta tipo di trigger
  wire TRIGGER_ENA = TrigConfRreg[4];
  wire Ena3 = TrigConfRreg[3];
  wire Ena2 = TrigConfRreg[2];
  wire Ena1 = TrigConfRreg[1];
  wire Ena0 = TrigConfRreg[0];
  
  // TrigConfReg1 bits
  wire TOCenable        = TrigConfRreg1[7];  // =1 TOC enabled
  wire TOCsourceSel     = TrigConfRreg1[6];  // =1 TOC su trigger, =0 TOC su AND
  wire resetTrigger     = TrigConfRreg1[5];
  wire resetTime        = TrigConfRreg1[4];
  wire latchDataForRead = TrigConfRreg1[3]; // copia i contatori di temp ed eventi nei registri per la lettura da CPU
  wire enaFlash         = TrigConfRreg1[2]; // =1 display flash ad ogni trigger 100ms, =0 display aggiornato ad ogni trigger
  wire globalEnable     = TrigConfRreg1[1]; // =1 abilita tutti i contatori, =0 li congela

 */

void initFirstTimeFRAM(void) {
   // T2TcountLimit
   write_FRAM_long16(EEPROM_T2TcountLimit_ADDR, 64000);
   XLR8_T2TcountLimit = read_FRAM_long16(EEPROM_T2TcountLimit_ADDR);

   // counters
   write_FRAM_long32(EEPROM_XLR8_TrigCnt_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_TimeCnt_ms_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_IN0Cnt_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_IN1Cnt_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_IN2Cnt_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_IN3Cnt_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_And0_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_And1_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_And2_ADDR,0);
//   write_FRAM_long32(EEPROM_XLR8_And3_ADDR,0);
//   write_FRAM_long32(EEPROM_XLR8_And4_ADDR,0);
//   write_FRAM_long32(EEPROM_XLR8_And5_ADDR,0);
   write_FRAM_long32(EEPROM_XLR8_Trig_mask_ADDR,0xFF);

   // conf regs
   write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, 0x1F);
   write_FRAM_byte(EEPROM_XLR8_TrigConfReg1_ADDR, 0xC6);

   write_FRAM_byte(EEPROM_XLR8_And0_mask_ADDR, 0xFF);
   write_FRAM_byte(EEPROM_XLR8_And1_mask_ADDR, 0xFF);
   write_FRAM_byte(EEPROM_XLR8_And2_mask_ADDR, 0xFF);
//   write_FRAM_byte(EEPROM_XLR8_And54_mask_ADDR, 0xFF);

// display OnOff = ON
   write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);
   
   write_FRAM_long32(EEPROM_hasBeenInit_ADDR, EEPROM_hasBeenInit_ID);
}

void printMsg(byte data8) {
  Serial.print(F("mask binary is: ")); print_binary(data8, 8); Serial.println();
  Serial.print(F("--- change? y/n - <x> exit"));
}

byte printEnterNewEnterMask(void) {
  Serial.print(F("--- enter new: "));
  return(readByteBinary_EndByAnyChar());
}

void printEntered(byte data8) {
  Serial.print(F("\n\r entered = ")); print_binary(data8, 4);   
}

void printMaskfull(byte data8) {
  Serial.println();
  print_binary(data8, 8);
  Serial.println();
}

byte whileMask(void) {
  byte mask;
  
  mask = printEnterNewEnterMask();
//  maskFull = (maskFull & 0xF0) + mask;
  printEntered(mask);
//  return(maskFull);
  return(mask);
}

/*
byte whileMask1(byte maskFull) {
  byte mask;
  
  mask = printEnterNewEnterMask();
  maskFull = (maskFull & 0x0F) + mask*16;
  printEntered(mask);
  return(maskFull);
}
*/

void writeAnd0_5EnableMask(void) {
 // uint8_t mask1, mask0, mask, maskFull;
  uint8_t maskFull;
  char chin;

  Serial.println();

// ========= And0_mask ===========
  Serial.println(F("And0: "));
  maskFull = XLR8_And0_mask;
 // mask0 = maskFull & 0xFF; // mask And0
 // mask1 = maskFull >> 4;   // mask And1 
  printMsg(maskFull); 
  while(1){
    chin = Serial.read();
    if( chin=='y') {
 //      maskFull = whileMask(maskFull);
       maskFull = whileMask();
       break;
    }
    else if( chin=='n') break;
    else if( chin=='x') { write_FRAM_byte(EEPROM_XLR8_And0_mask_ADDR, maskFull); return; }
  }
  XLR8_And0_mask = maskFull;
  write_FRAM_byte(EEPROM_XLR8_And0_mask_ADDR, maskFull);
// ===============================

// ========= And1_mask ===========
  maskFull = XLR8_And1_mask;
//  mask0 = maskFull & 0xFF; // mask And0
//  mask1 = maskFull >> 4;   // mask And1 
  Serial.println(F("And1: "));
  printMsg(maskFull); 
  while(1){
    chin = Serial.read();
    if( chin=='y') {
  //     maskFull = whileMask(maskFull);
       maskFull = whileMask();
       break;
    }
    else if( chin=='n') break;
    else if( chin=='x') { write_FRAM_byte(EEPROM_XLR8_And1_mask_ADDR, maskFull); return; }
  }
  Serial.println();
  XLR8_And1_mask = maskFull;
  write_FRAM_byte(EEPROM_XLR8_And1_mask_ADDR, maskFull);
// ===============================

// ========= And2_mask ===========
  maskFull = XLR8_And2_mask;
//  mask0 = maskFull & 0xFF; // mask And0
//  mask1 = maskFull >> 4;   // mask And1 
  Serial.println(F("And2: "));
  printMsg(maskFull); 
  while(1){
    chin = Serial.read();
    if( chin=='y') {
//       maskFull = whileMask(maskFull);
       maskFull = whileMask();
       break;
    }
    else if( chin=='n') break;
    else if( chin=='x') { write_FRAM_byte(EEPROM_XLR8_And2_mask_ADDR, maskFull); return; }
  }
  Serial.println();
  XLR8_And2_mask = maskFull;
  write_FRAM_byte(EEPROM_XLR8_And2_mask_ADDR, maskFull);
// ===============================


  
}


void readAnd05 (void) {
  uint32_t temp32;
  Serial.println();
  Serial.print(F("Time counter (ms): ")); temp32 = readTimeCnt_ms(1); Serial.println(temp32);  // qui avvine il latch di tutti i dati del readout
  Serial.print(F("And0 counter: ")); Serial.println(readInputCnt(0,8));
  Serial.print(F("And1 counter: ")); Serial.println(readInputCnt(0,9));
  Serial.print(F("And2 counter: ")); Serial.println(readInputCnt(0,10));
//  Serial.print(F("And3 counter: ")); Serial.println(readInputCnt(0,7));  
//  Serial.print(F("And4 counter: ")); Serial.println(readInputCnt(0,8));  
//  Serial.print(F("And5 counter: ")); Serial.println(readInputCnt(0,9));  
  Serial.print(F("And0 mask: ")); Serial.println(XLR8_And0_mask, BIN);  
  Serial.print(F("And1 mask: ")); Serial.println(XLR8_And1_mask, BIN);  
  Serial.print(F("And2 mask: ")); Serial.println(XLR8_And2_mask, BIN);  
//  Serial.print(F("And54 mask: ")); Serial.println(XLR8_And54_mask, BIN);  
}

void printAnd0_5_menu(void) {
  Serial.print(F("\n\r===== Contatori coincidenze And0..3 =====\n\r"));
  Serial.print(F("<h> or <space> print this list\n\r"));
  Serial.print(F("-------------------------------------------\n\r"));
  Serial.print(F("<s> Circuit Status\n\r"));
  Serial.print(F("-------------------------------------------\n\r"));
  Serial.print(F("<d> Coincidence AndExtra counters\n\r"));
  Serial.print(F("<e> Coincidence AndExtra enable mask\n\r"));
  Serial.print(F("<x> Exit\n\r"));
  Serial.println();
}


void And0_5_menu(void) {
   char ch;
   printAnd0_5_menu();
   ch = 0;
   Serial.print(F("\n\rAnd0-3>")); 
   while(ch!='x') {
    if (Serial.available()) {
      ch = Serial.read();  // preleva un carattere
      Serial.print(ch);
           if( (ch=='h') | (ch==' ') ) printAnd0_5_menu(); 
      else if(ch=='s') printStatus();
      else if(ch=='d') readAnd05();
      else if(ch=='e') writeAnd0_5EnableMask();
      else Serial.print(ch);
      Serial.print(F("\n\rAnd03>")); 
    }
   }
}




void triggerMenu(void) {
   char ch;
   printTriggerMenu();
   ch = 0;
   Serial.print(F("\n\rTRIG>")); 
   while(ch!='x') {
    if (Serial.available()) {
      ch = Serial.read();  // preleva un carattere
      Serial.print(ch);
           if( (ch=='h') | (ch==' ') ) printTriggerMenu(); 
      else if(ch=='s') printStatus();
      else if(ch=='d') dispFlashEnaDisa();
      else if(ch=='e') writeEnableMask();
      else if(ch=='g') writeGlobalEnable();
      else if(ch=='a') {XLR8_TRIGGER_MODE_AND write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\rTRIGGER MODE AND"));} 
      else if(ch=='t') {XLR8_TRIGGER_MODE_T2T write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\rTRIGGER MODE T2T"));} 
      else if(ch=='y') {XLR8_TRIGGER_MODE_T2T_OR write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\rTRIGGER MODE T2T"));} 
      else if(ch=='o') {XLR8_TRIGGER_MODE_OR write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\rTRIGGER MODE OR"));} 
      else if(ch=='q') And0_5_menu();
/*       
         else if(ch=='m') {XLR8_TRIGGER_MODE_MU_DECAY write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\rTRIGGER MODE MU DECAY"));} 
         else if(ch=='f') {XLR8_TRIGGER_MODE_FLOOR write_FRAM_byte(EEPROM_XLR8_TrigConfReg_ADDR, XLR8_TrigConfReg); Serial.println(F("\n\nTRIGGER MODE FLOOR"));} 
*/
      else if(ch=='c') setT2TcountLimit();
      else Serial.print(ch);
      Serial.print(F("\n\rTRIG>")); 
    }
   }
}
