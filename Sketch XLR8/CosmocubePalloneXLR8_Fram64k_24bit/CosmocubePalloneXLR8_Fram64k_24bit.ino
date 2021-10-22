/**************************************************************************
okkio: le librerie Adafruit per OLED 128x64 allocano al boot 1k di ram che non
viene conteggiata ---> aggiungere 1k alla ram usata nel report del compilatore
//
//
// 2021-04-09 implementazione tastierino
// 2021-04-29 aggiunta 6 contatori di coincidenze con mask programmabile
// 2021-05-05 read_FRAM_long-->read_FRAM_long16, write_FRAM_long-->write_FRAM_long16
// fino al 2021-06-11  diverse modifiche 
//     distribuzioni T2T, con e senza il floor
//     roll display, on/off da PC
//     
// 2021-07-10 contatori da 32 a 24 bit per recuperare spazio in FPGA
// 2021-07-22 corretti diversi bachetti dopo aver messo l'opzione "mostra tutti i warning"
//    DA FARE: READ/WRITE FRAM 24 bit
//             scrittura in BUFFER storia conteggi e trigger ogni N secondi (parametro programmabile da PC su FRAM)
//
 **************************************************************************/
//#define FULL
//#define FRAM_8M
//#define FRAM_4M
#define FRAM_64k

// per uso dei soli pulsanti al posto della keyboard
#define pusantiOnly

#define timerInterval 500000
// quantiPerSecondo: quanti interrupt timer in un secondo
#define quantiPerSecondo (1000000/timerInterval) 

// 13 parametri a 24 bit da loggare nella FRAM: time, Ntri, 8 inputs, 3 And
#define NbytePerSingleLogData 3
#define NparamLog 13
#define NbytePerLog (NparamLog*NbytePerSingleLogData)

#include <avr/io.h>
#include <avr/wdt.h>
#include <TimerOne.h>
#include <XLR8Core.h>

#define XLR8_FRAM      _SFR_MEM8(0xD7)

//mappa indirizzi dei registri del XBS TRIGGER

#define XLR8_TrigConfReg   _SFR_MEM8(0xD8)

#define XLR8_T2Treg  _SFR_MEM16(0xD9)
#define XLR8_T2Treg_L  _SFR_MEM8(0xD9)
#define XLR8_T2Treg_H  _SFR_MEM8(0xDA)

#define XLR8_TrigConfReg1  _SFR_MEM8(0xDB)

#define XLR8_T2TcountLimit  _SFR_MEM16(0xDC)
#define XLR8_T2TcountLimitL  _SFR_MEM8(0xDC)
#define XLR8_T2TcountLimitH  _SFR_MEM8(0xDD)

#define XLR8_TimeCnt_ms_rdout  _SFR_MEM32(0x8C)

// tutti indirizzi consecutivi
#define XLR8_EvCnt0_rdout  _SFR_MEM32(0x90)
#define XLR8_EvCnt1_rdout  _SFR_MEM32(0x94)
#define XLR8_EvCnt2_rdout  _SFR_MEM32(0x98)
#define XLR8_EvCnt3_rdout  _SFR_MEM32(0x9C)
#define XLR8_EvCnt4_rdout  _SFR_MEM32(0xA0)
#define XLR8_EvCnt5_rdout  _SFR_MEM32(0xA4)
#define XLR8_EvCnt6_rdout  _SFR_MEM32(0xA8)
#define XLR8_EvCnt7_rdout  _SFR_MEM32(0xAC)
//

#define XLR8_TrigCnt_rdout  _SFR_MEM32(0xC7)

#define XLR8_CPU_Pulse _SFR_MEM8(0xFF)
/*
#define XLR8_TrigIntrCnt_rdout  _SFR_MEM32(0xA4)
#define XLR8_TrigIntrCnt_rdout0  _SFR_MEM8(0xA4)
#define XLR8_TrigIntrCnt_rdout1  _SFR_MEM8(0xA5)
#define XLR8_TrigIntrCnt_rdout2  _SFR_MEM8(0xA6)
#define XLR8_TrigIntrCnt_rdout3  _SFR_MEM8(0xA7)
*/

#define XLR8_And0_rdout _SFR_MEM32(0xDE)
#define XLR8_And1_rdout _SFR_MEM32(0xE2)
#define XLR8_And2_rdout _SFR_MEM32(0xE6)

#define XLR8_And0_mask _SFR_MEM8(0xF6)
#define XLR8_And1_mask _SFR_MEM8(0xF7)
#define XLR8_And2_mask _SFR_MEM8(0xB5)

#define XLR8_Trig_mask _SFR_MEM8(0xBE)

// il numero di eventi dall'ultimo reset è nel registro XLR8_TrigCnt_rdout
//   occorre prima copiarci il contatore dei trigger e poi leggerlo
//   fa tutto readTriggerCnt(1)  in cui l'1 determina lesecuzione della copia
//
//#define numEventi readTriggerCnt(1)
// in  nev  viene messo il suddetto trigger count, aggiornato ad ogni timer interrupt
//  di 1 sec
uint32_t time_in_ms, numTrigger, numIN0, numIN1, numIN2, numIN3;  // 15-3-2021

// definizioni per rollDispMode()
#define SingleKey 0
#define KEYBOARD 1

// ---> USA QUESTI DEFINE PER ABILITARE IL TIPO DI DISPLAY USATO
#define LCD_I2C
//#define OLED128x64

#define Reset_AVR wdt_enable(WDTO_30MS); while(1); ﻿

// byte
// mode   descr      / separa riga0 da riga1
//  0      time, @ev / vth or vbias, rate medio or 1 min
//  1      cnt#0, cnt#2 / cnt#1, cnt#3
//
int8_t disp_mode;  // copia RAM della loc FRAM qui sotto
//#define nModes 5  // con le due pagine sul decadimento del mu
#define nModes 3


// INDIRIZZI IN FRAM
#define EEPROM_disp_mode_ADDR 0
//
// numEventi 32 bit
//#define EEPROM_numEventi_ADDR 1
// timeSecondi 32 bit
//#define EEPROM_tempoSecondi_ADDR 5
// TOC enable, 8 bit
//#define EEPROM_TOC_ADDR 9  non piu' usato
// XLR8_TrigConfReg, 8 bit
#define EEPROM_XLR8_TrigConfReg_ADDR 10
// T2T count limit, 16 bit
#define EEPROM_T2TcountLimit_ADDR 11
// data buffer write pointer, 32 bit
#define DATA_BUFFER_WRITE_POINTER 13
// 8 bit
#define EEPROM_XLR8_TrigConfReg1_ADDR 17
//32 bit
#define EEPROM_XLR8_TimeCnt_ms_ADDR 18
//32 bit
#define EEPROM_XLR8_TrigCnt_ADDR 22

// contatori IN 32 bitdevono essere in locazioni contigue
#define EEPROM_XLR8_IN0Cnt_ADDR 26
#define EEPROM_XLR8_IN1Cnt_ADDR 30
#define EEPROM_XLR8_IN2Cnt_ADDR 34
#define EEPROM_XLR8_IN3Cnt_ADDR 38
#define EEPROM_XLR8_IN4Cnt_ADDR 42
#define EEPROM_XLR8_IN5Cnt_ADDR 46
#define EEPROM_XLR8_IN6Cnt_ADDR 50
#define EEPROM_XLR8_IN7Cnt_ADDR 54

//32 bit
#define EEPROM_XLR8_TrigIntCnt_ADDR 58

// 32 bit ID
#define EEPROM_hasBeenInit_ADDR 62
#define EEPROM_hasBeenInit_ID 0xDEADFACE

// contatori And0-5 32 bit
#define EEPROM_XLR8_And0_ADDR 66
#define EEPROM_XLR8_And1_ADDR 70
#define EEPROM_XLR8_And2_ADDR 74

// AndExtra mask 8 bit
#define EEPROM_XLR8_And0_mask_ADDR 78
#define EEPROM_XLR8_And1_mask_ADDR 79
#define EEPROM_XLR8_And2_mask_ADDR 80

// stato on/off della BACKLIGHT del display
#define EEPROM_OnOffDisplay_ADDR 81

/*
// 32 bit
#define EEPROM_XLR8_IN4Cnt_ADDR 80
#define EEPROM_XLR8_IN5Cnt_ADDR 84
#define EEPROM_XLR8_IN6Cnt_ADDR 88
#define EEPROM_XLR8_IN7Cnt_ADDR 92
*/

// Trig mask 8 bit
#define EEPROM_XLR8_Trig_mask_ADDR 82

// ==== data buffer da indirizzo 100 a indirizzo 4191 ====
// data buffer start address, numero 16 bit
#define DATA_BUFFER_START_ADDR 100
#if defined(FRAM_64k)
   #define FRAM_SIZE 4096
#endif
#if defined(FRAM_8M)
   #define FRAM_SIZE 1048576
#endif
#if defined(FRAM_4M)
   #define FRAM_SIZE 524288
#endif

// pin scheda -------------------------
/*
// '595 display LED SMD
#define SERIN 13
#define SRCLK 12
#define RCLK_LED 11
*/
// precedenti
//#define TRIGGER 2
//#define IN0 3
//#define IN1 4
//#define IN2 5
//#define RESETb 7
//#define INJECT 13
//#define TOC_TOC LED_BUILTIN

// attuali
//#define TRIGGER_OUT 7 ... pilotato da FOGA
// usati in input da FOGA
#define IN0 5
#define IN1 6
#define IN2 3
#define IN3 4
#define IN4 7
#define IN5 8
#define IN6 10
#define IN7 9
// su ogni toggle del livello in uscita fa un TOC, USATO DA FPGA
#define TOC 2
// switch verso GND per fare il reset dei contatori al boot e per cambiare disp mode quando in run
//  gli input digitali su pin analogici sono A0..A5 --> 14..19 
//#define ROLL_LIGHT_RESET_PIN 17
//#define TOC_PIN 18
//#define TRIG_SEL_PIN 19
//
// external keyboard ELIMINATA -----
// input
//#define KEYB_DOUT_PIN 17
// outputs
#define KEYB_SH1_LD0_PIN 18
//#define KEYB_SH_CLK_PIN 16
// -----------------------
//
// usati in out dall'FPGA
#define D8 8
#define D9 9
#define D10 10
/*
#define D11 11
#define D12 12 
#define D13 13
*/
// canali ADC usati
#define Vthr1  A0
#define Vthr2  A2
#define Vbias A1

// ------------------------------------


// bit# in XLR8 TrigConfRreg register
#define IN0_ENA 0
#define IN1_ENA 1
#define IN2_ENA 2
#define IN3_ENA 3
#define TRIGGER_ENA 4
#define TRIGGER_MODE0 5
#define TRIGGER_MODE1 6
#define TRIGGER_MODE2 7

#define TRIGGER_MODE_AND 0
#define TRIGGER_MODE_T2T 1
#define TRIGGER_MODE_T2T_OR 5
#define TRIGGER_MODE_MU_DECAY 2
#define TRIGGER_MODE_FLOOR 3
#define TRIGGER_MODE_OR 4

// ------------------------------------
// bit# in XLR8 TrigConfRreg1 register
#define TOCenable 7
#define TOCsourceSel 6
#define TRIGGER_RESET 5
#define TIME_AND_COUNTERS_RESET 4
#define LATCH_READOUT_DATA 3
#define ENA_FLASH 2
#define GLOBAL_ENABLE 1


#define timeUnit_ps (uint32_t)15625

//#define READ_FORCE_CHAR 'r'

//#define eventReset   digitalWrite(RESETb, LOW); digitalWrite(RESETb, HIGH);
#define XLR8_TRIGGER_reset          bitSet(XLR8_TrigConfReg1, TRIGGER_RESET); bitClear(XLR8_TrigConfReg1, TRIGGER_RESET);

#define XLR8_TRIGGER_MODE_AND       bitSet(XLR8_TrigConfReg, TRIGGER_MODE2);
#define XLR8_TRIGGER_MODE_OR        bitClear(XLR8_TrigConfReg, TRIGGER_MODE2);
#define XLR8_TRIGGER_MODE_T2T       bitSet(XLR8_TrigConfReg, TRIGGER_MODE0);
#define XLR8_TRIGGER_MODE_T2T_OR    bitClear(XLR8_TrigConfReg, TRIGGER_MODE0);
//#define XLR8_TRIGGER_MODE_MU_DECAY  bitClear(XLR8_TrigConfReg, TRIGGER_MODE0); bitSet(XLR8_TrigConfReg, TRIGGER_MODE1); bitClear(XLR8_TrigConfReg, TRIGGER_MODE2);
//#define XLR8_TRIGGER_MODE_FLOOR     bitSet(XLR8_TrigConfReg, TRIGGER_MODE0); bitSet(XLR8_TrigConfReg, TRIGGER_MODE1); bitClear(XLR8_TrigConfReg, TRIGGER_MODE2);

#ifdef LCD_I2C
  #include <Wire.h> 
  #include <PCF8574_HD44780_I2C.h>
  // Address 0x27, 16 chars, 2 line display
  PCF8574_HD44780_I2C lcd(0x27,20,4);  // con la schedina MH sul front,chip PCF8574T
 // PCF8574_HD44780_I2C lcd(0x3F,16,2);  // con la schedina senza MH sul front, chip PCF8574A (slave address differente, vedi datasheet)

byte tacca1[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111
};
byte tacca2[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111
};

byte tacca3[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};
byte tacca4[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111
};
byte tacca5[8] = {
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte tacca6[8] = {
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte tacca7[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};


#endif

byte timerFlag, MASK, XLR8_TrigConfReg_copy;
//unsigned long tempoSecondi, numEventi, conta60secondi, numEventiPrec, eventiPerMinuto;
uint32_t tempoSecondi, eventiPerMinuto;
uint16_t T2T;

// da distingure in base alla dimensione della FRAM?

uint32_t write_pointer;

int16_t Vadc1, Vadc2;

uint8_t saveInterval;

byte keybWasEnabled; // stato di abilitazione precedente della keyb: se ora la abilito ed era disab setto i valori degli swich
                         //   se ora la disabilito ed era abilitata setto i valori dalla FRAM


// ena/disa toc  1/0
uint8_t TocEna;

// se FALSE display Rate calcolato negli ultimi 60 secondi, se TRUE display Rate medio
boolean RATE_DISP;
// per inizializzare il display ogni 2 secondi, nel caso ci siano problemi di funzionamento (connettore staccato, ...)
boolean INIT_DISP;

uint8_t dumpFlag;

//#include <EEPROM.h>
#include "FRAM.h"

#include "myCode.h"


#ifdef OLED128x64
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels

  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//  #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  #define NUMFLAKES     10 // Number of snowflakes in the animation example

// logo INFN-PI
  #define INFN_LOGO_HEIGHT   63
  #define INFN_LOGO_WIDTH    78

// logo UniSI
  #define UniSI_LOGO_HEIGHT   63
  #define UniSI_LOGO_WIDTH    61

  #include "logoUniSiena_61x63.h"
  #include "logoINFNPI_78x63.h"
#endif

// ===========================================================================

/*
// FIFO per i dati dei trigger T-T
//
#define BUFFER_SIZE 8
uint16_t buffer[BUFFER_SIZE];
uint8_t next_in = 0;
uint8_t next_out = 0;

void writeFIFO(uint16_t data) {
   uint16_t t;
   buffer[next_in] = data;
   t = next_in;
   next_in = (next_in+1) % BUFFER_SIZE;
   if(next_in == next_out)
     next_in = t;           // Buffer full !!
}

#define FIFOhasData (next_in != next_out)
#define clearFIFO next_in=next_out
// ritorna il primo carattere disponibile del buffer , se non ce ne sono
//   ritorna 0x00 (null)
uint16_t readFIFO(void) {
   uint16_t c;
   if(!FIFOhasData) return(0);
   c = buffer[next_out];
   next_out = (next_out+1) % BUFFER_SIZE;
   return(c);
}
*/
//=============================================

//#define allLEDoff WR595_LED(0xFFFFFFFF)
//#define allLEDon WR595_LED(0)

#include "commands.h"

//#include "TriggerInt\XLR8_Trigger.h"

void setup() {
 // uint8_t temp8;
  
//  tempoSecondi = 0;  // da gestire il salvataggio e richiamo in EEPROM
  timerFlag=1;  // fa fare il display all'accensione
  keybWasEnabled=0; // init flag ena keyb key

//  nev = 0;  // 15-3-2021
  
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(IN5, INPUT);
  pinMode(IN6, INPUT);
  pinMode(IN7, INPUT);
  pinMode(TOC, INPUT);

//  pinMode(CLEAR_SWITCH, INPUT);

// LED PILOTATI DALLA LOGICA DELL'FPGA, DISPLAY  --- ELIMINATO

//pinMode(SERIN, OUTPUT);
//pinMode(SRCLK, OUTPUT);
//pinMode(RCLK_LED, OUTPUT);

//digitalWrite( SERIN, 0 );
//digitalWrite( SRCLK, 0 );
//digitalWrite( RCLK_LED, 0 );

// interfaccia con kayboard ELIMINATA
//  pinMode(KEYB_DOUT_PIN, INPUT);

#if pulsantiOnly
// inputs
  pinMode(KEYB_SH1_LD0_PIN, INPUT);
#else
// outputs
//  pinMode(KEYB_SH1_LD0_PIN, OUTPUT);
#endif

// KEYBOARD ELIMINATA
//  pinMode(KEYB_SH_CLK_PIN, OUTPUT);

  init_FRAM();
  readDispMode();  // setta disp_mode
  
// init porta seriale
  Serial.begin(115200);

// azzera i contatori se al boot e' premuto il pulsante dedicato
//  scanKeyb(); // leggi tastiera e agisci di conseguenza
//  if(keybConnected==false)
//     if(digitalRead(ROLL_LIGHT_RESET_PIN)==0) resetFPGA();

// INJECT a 0
//  digitalWrite(INJECT, LOW);

delay(1000); // power-up completo

#ifdef LCD_I2C
  lcd.init();           // LCD Initialization              
  lcd.backlight();      // Backlight ON
  lcdOff=false; // backlight ON 
  lcd.clear();          // Clear the display
  lcd.setCursor(1,0);   // Set the cursor to col 1, row 0
#ifdef FRAM_4M
  lcd.print(F("CCPallone 4Mb"));   // Print the first word
#endif
#ifdef FRAM_8M
  lcd.print(F("CCPallone 8Mb"));   // Print the first word
#endif
#ifdef FRAM_64k
  lcd.print(F("CCPallone 64k"));   // Print the first word
#endif
  lcd.print(F("V0.1"));   // Print the first word
  lcd.setCursor(5,1);   // Set the cursor to col 3, row 1
  lcd.print(F("INFN-Pisa"));  // Print the second word
  lcd.setCursor(0,3);   // Set the cursor to col 0, row 3
  lcd.print(F("by Fabio Morsani"));  // Print the second word
  delay(1500);

  lcd.createChar(1, tacca1);
  lcd.createChar(2, tacca2);
  lcd.createChar(3, tacca3);
  lcd.createChar(4, tacca4);
  lcd.createChar(5, tacca5);
  lcd.createChar(6, tacca6);
  lcd.createChar(7, tacca7);
#endif


// ===============================================
//            SETUP E PRIME LETTURE ADC
// ===============================================
    analogReference(DEFAULT);
    analogRead(Vthr1);  // la prima lettura risulta schifezza
    analogRead(Vthr2);  // la prima lettura risulta schifezza
    analogRead(Vbias);  // la prima lettura risulta schifezza
    
// ===============================================


   // updateInterval = (unsigned long)read_FRAM_long16(EEPROM_updateInterval_ADDR);
    
// ======================================================================
//       RELOAD CONTATORI DI TEMPO ED EVENTI DAL'ULTIMO VALORE SALVATO
// ======================================================================
   // tempoSecondi = read_FRAM_long32(EEPROM_tempoSecondi_ADDR);  // tempo

// ======================================================================
  

  
    eventiPerMinuto = 0;
    beginPtr=0, endPtr=1;
    
//    analogReference(DEFAULT);  era qui ... l'ho spostato sopra
    RATE_DISP=false;
    INIT_DISP=true;

// ===================
//      WATCHDOG
// ===================
/*  wdt_enable parameters
    15 mS – WDTO_15MS
    32 mS – WDTO_32MS
    64 mS – WDTO_64MS
    125 mS – WDTO_125MS
    250 mS – WDTO_250MS
    500 mS – WDTO_500MS
    1 S – WDTO_1S
    2 S – WDTO_2S
    4 S – WDTO_4S
    8 S – WDTO_8S
 */
    wdt_enable(WDTO_4S);  // A 4 SECONDI
// ===================



// ===========================================================================
//    INIZIALIZZAZIONE DEI REGISTRI DI XLR8 DAI DATI CONTENUTI NELLA FRAM
// ===========================================================================
//initFirstTimeFRAM();
if(read_FRAM_long32(EEPROM_hasBeenInit_ADDR) != EEPROM_hasBeenInit_ID) initFirstTimeFRAM();
copyStatusToFPGA();
if(read_FRAM_byte(EEPROM_OnOffDisplay_ADDR)==0) { lcdOff=true;   lcd.noBacklight();  }    // Backlight OFF

// enable interrupt for special triggers
//enableTriggerInterrupt();

// ===============================================
//            INIT TIMER INTERRUPT AD 1 SEC
// ===============================================
 //   Timer1.initialize(1000000); // set a timer of length 1000000 microseconds (or 1 sec - or 1Hz)
    Timer1.initialize(timerInterval); // set a timer of length 500000 microseconds (or 0.5 sec - or 1Hz)
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here
// ===============================================

// ===========================================================================
//allLEDon;
saveInterval = 0;
dumpFlag=0;

//clearFIFO; // clear FIFO T-T data
Serial.print(F("\n\r>"));  // prompt 
}

// ===========================================================================
//               TIMER INTERRUPT SERVICE ROUTINE OGNI 1 SEC
// ===========================================================================
void timerIsr()
{
  // Toggle LED
  wdt_reset();

  if(timerFlag==0) {
     readEventsData();  // aggiunta il 15-3-2021
     timerFlag=1;  //  setta una flag ad uso del loop principale
  }
}

// ====================================================
//               PROGRAMMA PRINCIPALE
// ====================================================

void loop() {
  char ch;
 // unsigned int u16;
 // unsigned long temp32;

// LCD 20x4
//            1111111111
//  01234567890123456789
//  TOC:tocTy   Buffer:
//  Vb:12345     1234 
//  S:-12345    of 1234
//
//  0:1234567  2:1234567
//  1:1234567  3:1234567
//  T:1234567  E:1234567
//  Tr:trType  InEn:1111

/*
// test keyboard
while(1) {
  u16 = scanKeyb();
  Serial.print(u16>>8, BIN);  Serial.print(" "); Serial.println(u16 & 0xFF, BIN); 
  delay(1000);
}
*/
/*
// cambia disp_mode, con il solo pulsante su A3
// con la keyb fare in modo diverso
  if(digitalRead(ROLL_LIGHT_RESET_PIN)==0) rollDispMode();
*/
/* 
  scanKeybStable(); // leggi tastiera e agisci di conseguenza

  // implementare keybConnected_prev così quando si passa da connected a unconnected si restore il contenuto dalla FRAM
  
  if(keybConnected==true)  {
    // pulsante DISP
    if(dispKeyPressed==true) rollDispMode();
    //
    if(keybEnable==true) { // la keyb è collegata ed abilitata, setta quanto selezionato con gli switch
      keybWasEnabled = 1;
      // setta il trigger type AND/OR
      if(keybTriggerAndOr!=0) {XLR8_TRIGGER_MODE_AND} else {XLR8_TRIGGER_MODE_OR}
      // setta i bit di input enable
      ch = (XLR8_TrigConfReg & 0xF0) | inEnable;  // rimpiazza i bit di mask appena immessi
      XLR8_TrigConfReg = ch;

      if(resetKeyPressed==true) handleResetKey();
    }
    else { // collegata ma non abilitata, setta da FRAM
      if(keybWasEnabled==1) {
        keybWasEnabled=0;
        // setto a FRAM
        copyStatusToFPGA();
      }
    }
  }

  else if(digitalRead(ROLL_LIGHT_RESET_PIN)==0) {
   // if(lcdOff==true) { lcd.backlight(); lcdOff=false; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);}
    rollDispMode();
  }
  else if(digitalRead(TOC_PIN)==0) {
    if(lcdOff==true) { lcd.backlight(); lcdOff=false; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);}
    else if(disp_mode==2) histoZoomOut();
    else handleResetKey();
  }
  else if(digitalRead(TRIG_SEL_PIN)==0) {
    if(lcdOff==true) { lcd.backlight(); lcdOff=false; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);}
    else if(disp_mode==2) histoZoomIn();
    else rollTRIGGER();
  }
*/
  if(timerFlag==1) {
   //   FIFO2FRAM();  // eliminata per mancanza di spazio nell'FPGA
      // calcolo rate per minuto, una volta al secondo in base agli ultimi 15 secondi
      eventiPerMinuto = rate60sec(); 
       
    //  tempoSecondi = (time_in_ms + 499)/1000;
  
      dispInfo();
      saveInterval = (saveInterval+1)%20; // salvo ogni secondo, PRAMETERIZZARE!!! se a dividere c'è 2-> una volta al secondo
      if(saveInterval==0) saveCountersInFRAM(); // !!! se la keyb è attiva salvare solo il tempo e il contatore di trigger!!!! ???
      if(dumpFlag==1)  {
         dataBufferDumpPallone();
         dumpFlag=0;
      }
      timerFlag=0;
     // Serial.println("*");
 }  // fine timerflag==1

  if (Serial.available()) {
      ch = Serial.read();  // preleva un carattere
     // Serial.print(ch);
           if( (ch=='h') | (ch==' ')) printMenu(); 
      else if(ch=='s') printStatus();
      else if(ch=='1') EnaDisaToc();
      else if(ch=='2') TOCsource();
      else if(ch=='k') resetWritePointer();
      else if(ch=='9') dumpFlag=1;
      else if(ch=='t') triggerMenu(); 
      else if(ch=='r') { set_FRAM_write_disable(); resetFPGA(); Serial.print(F("0")); doReset(); /* wdt_enable(WDTO_30MS); while(1); */} //" Reset ..."
  #if defined(FULL) 
      else if(ch=='3') setResetGlobalEnable();
      else if(ch=='p') trigCntRWtest();
      else if(ch=='0') dataBufferFillingLevel();
      else if(ch=='4') resetTrigger();
      else if(ch=='w') copyStatusToFPGA();
      else if(ch=='x') { testFRAM(); }
      else if(ch=='<') testPulseIN3();
  #endif
      else if(ch=='v') printVoltages();
     // else if(ch=='+') plotMu();
      else if(ch=='d') {
         disp_mode++;  
         if(disp_mode >= nModes) disp_mode = 0;
         prevWritePointer = 0; // per abilitare il refresh di disp_mode2/3
         INIT_DISP = true;
         write_FRAM_byte(EEPROM_disp_mode_ADDR, disp_mode);
      //   while(timerFlag==0); // torna col prompt dopo che il display viene aggiornato  SI ALLUPPA QUI, PERCHE'?
      }
      else if(ch=='D') {
         if(disp_mode != 0) disp_mode--; else disp_mode = nModes-1;
         prevWritePointer = 0; // per abilitare il refresh di disp_mode2/3
         INIT_DISP = true;
         write_FRAM_byte(EEPROM_disp_mode_ADDR, disp_mode);
      //   while(timerFlag==0); // torna col prompt dopo che il display viene aggiornato  SI ALLUPPA QUI, PERCHE'?
      }
      else if(ch=='o') {
         if(lcdOff==true) { lcd.backlight(); lcdOff=false; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0xFF);}
         else { lcd.noBacklight(); lcdOff=true; write_FRAM_byte(EEPROM_OnOffDisplay_ADDR, 0);}
      }        
      else Serial.print(ch);
      Serial.print(F("\n\r>")); 
   }

}
