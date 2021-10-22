//
// FRAM 
//
// port D   bit       function  direction
//           5         FR_DIO    i/o
//           2         FR_SCK    out
//           4         FR_WPn    out
//           3         FR_CSn    out
//

//
/*
	buf buf_HOLDb (HOLDb, FRAMreg[7]);
	buf buf_SCK   (SCK,   FRAMreg[6]);
	buf buf_SI    (SI,    FRAMreg[5]);
	buf buf_WPn   (WPn,   FRAMreg[4]);
	buf buf_CSn   (CSn,   FRAMreg[3]);
	XLR8_FRAM
*/

//#define FRAM_WPn_1  digitalWrite(FR_WPn, HIGH)
#define FRAM_WPn_1  bitSet(XLR8_FRAM, 4)

//#define FRAM_WPn_0  digitalWrite(FR_WPn, LOW)
#define FRAM_WPn_0  bitClear(XLR8_FRAM, 4)

//#define FRAM_SCK_1  digitalWrite(FR_SCK, HIGH);
#define FRAM_SCK_1  bitSet(XLR8_FRAM, 6);

//#define FRAM_SCK_0  digitalWrite(FR_SCK, LOW);
#define FRAM_SCK_0  bitClear(XLR8_FRAM, 6);

//#define FRAM_CSn_1  digitalWrite(FR_CSn, HIGH);
#define FRAM_CSn_1  bitSet(XLR8_FRAM, 3);

//#define FRAM_CSn_0  digitalWrite(FR_CSn, LOW);
#define FRAM_CSn_0  bitClear(XLR8_FRAM, 3);

//#define FRAM_DATA_1 digitalWrite(FR_DIO, HIGH);
#define FRAM_DATA_1 bitSet(XLR8_FRAM, 5);

//#define FRAM_DATA_0 digitalWrite(FR_DIO, LOW);
#define FRAM_DATA_0 bitClear(XLR8_FRAM, 5);

//#define FRAM_DATA_OUT pinMode(FR_DIO, OUTPUT);

//#define FRAM_DATA_IN pinMode(FR_DIO, INPUT);

#define HOLDb_1 bitSet(XLR8_FRAM, 7);
#define HOLDb_0 bitClear(XLR8_FRAM, 7);

#define FRAM_SO_eq1 bitRead(XLR8_FRAM, 2)

// =1: il ciclo non viene terminato (verranno spediti i dati)
#define continue_cmd 1
#define end_cmd 0

/*
// comandi  25CL04
#define FRAM_SET_WRITE_ENABLE_LATCH 6
#define FRAM_WRITE_STATUS_REGISTER 1
#define FRAM_WRITE_MEMORY_DATA 0xA
#define FRAM_READ_MEMORY_DATA 0xB
*/

// comandi  25CL64 e CY15B108
#define FRAM_SET_WRITE_ENABLE_LATCH 6
#define FRAM_WRITE_DISABLE 4
#define FRAM_READ_STATUS_REGISTER 5
#define FRAM_WRITE_STATUS_REGISTER 1
#define FRAM_WRITE_MEMORY_DATA 0x2
#define FRAM_READ_MEMORY_DATA 0x3

/*
// comandi  CY15B104, 19 bit address
#define FRAM_SET_WRITE_ENABLE_LATCH 6
#define FRAM_WRITE_DISABLE 4
#define FRAM_READ_STATUS_REGISTER 5
#define FRAM_WRITE_STATUS_REGISTER 1
#define FRAM_WRITE_MEMORY_DATA 0x2
#define FRAM_READ_MEMORY_DATA 0x3
//
#define FRAM_FAST_READ_MEMORY_DATA 0xB
#define FRAM_SLEEP_MODE 0xB9
*/
// RESERVED GLI ALTRI


void set_FRAM_read_enable(void) {
  // FRAM_DATA_IN;
   FRAM_SCK_0; FRAM_WPn_0; FRAM_CSn_0;
}

void set_FRAM_read_disable(void) {
   FRAM_SCK_0; FRAM_CSn_1; FRAM_WPn_0;
 //  FRAM_DATA_OUT;
}


void set_FRAM_write_enable(void) {
  // FRAM_DATA_OUT;
   FRAM_SCK_0; FRAM_WPn_1; FRAM_CSn_0;
}

void set_FRAM_write_disable(void) {
   FRAM_SCK_0; FRAM_CSn_1; FRAM_WPn_0;
}


uint8_t read_FRAM(int8_t cmd_flag) {
    int8_t temp8; //, pA;

    temp8=0; // serve anche come delay dal negedge clk
   // FRAM_SCK_1;

   // primo bit gia' pronto
  //  pA = digitalRead(FR_DIO);
 //   if(bitRead(pA, 0)) bitSet(temp8, 7);
    if(FRAM_SO_eq1) bitSet(temp8, 7);
    FRAM_SCK_1; FRAM_SCK_0;  

  //  pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 6);
    FRAM_SCK_1; FRAM_SCK_0; 

 //   pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 5);
    FRAM_SCK_1; FRAM_SCK_0; 

  //  pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 4);
    FRAM_SCK_1; FRAM_SCK_0; 

   // pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 3);
    FRAM_SCK_1; FRAM_SCK_0; 

   // pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 2);
    FRAM_SCK_1; FRAM_SCK_0; 

 //   pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 1);
    FRAM_SCK_1; FRAM_SCK_0; 

  //  pA = digitalRead(FR_DIO);
    if(FRAM_SO_eq1) bitSet(temp8, 0);
    FRAM_SCK_1; FRAM_SCK_0; 

    if(cmd_flag==end_cmd) FRAM_CSn_1;
    return(temp8);
}



void write_FRAM(int8_t cmd, int8_t cmd_flag) {
    FRAM_DATA_0;
    if(bitRead(cmd, 7)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 6)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 5)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 4)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 3)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 2)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 1)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

    FRAM_DATA_0;
    if(bitRead(cmd, 0)) FRAM_DATA_1;
    FRAM_SCK_1; FRAM_SCK_0;

  //  if(cmd_flag==end_cmd) FRAM_CSn_1;
    if(cmd_flag==end_cmd) set_FRAM_write_disable();
    
}

uint8_t read_FRAM_statusRegister(void) {
   set_FRAM_write_enable();
   write_FRAM(FRAM_READ_STATUS_REGISTER, continue_cmd);
   return(read_FRAM(end_cmd));
  // FRAM_CSn_0;
 //  Serial.println(read_FRAM(end_cmd), BIN);
}

void set_FRAM_noprotect(void) {
   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_STATUS_REGISTER, continue_cmd);
   write_FRAM(0, end_cmd);  // no write protections
}

void init_FRAM(void) {
   HOLDb_1;
   FRAM_SCK_0; FRAM_WPn_0; FRAM_CSn_1;
   //FRAM_DATA_OUT;
   set_FRAM_noprotect();
}



#if defined(FRAM_64k)
void write_FRAM_byte(int16_t addr, int8_t data) {  
   uint8_t addrByte;
   
   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>8) & 0xFF);   // 15:8
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) (addr & 0xFF);        // 7:0
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   write_FRAM(data, end_cmd);  // 8-bit data
   set_FRAM_write_disable();
}
#elif defined(FRAM_8M)
void write_FRAM_byte(int32_t addr, int8_t data) {  
   uint8_t addrByte;
   
   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);   // 23:16
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);   // 15:8
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) (addr & 0xFF);        // 7:0
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   write_FRAM(data, end_cmd);  // 8-bit data
   set_FRAM_write_disable();
}
#endif



#if defined(FRAM_64k)
void write_FRAM_long16(int16_t addr, int16_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address

// i due byte sono memorizzati dall'LSByte al MSByte
   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address

   set_FRAM_write_disable();
}
#elif defined(FRAM_8M)
void write_FRAM_long16(int32_t addr, int16_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address

// i due byte sono memorizzati dall'LSByte al MSByte
   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address

   set_FRAM_write_disable();
}
#endif




#if defined(FRAM_64k)
void write_FRAM_long32(int16_t addr, int32_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>16)& 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) ((data>>24)& 0xFF);
   write_FRAM(addrByte, end_cmd); 

   //set_FRAM_write_disable();
}
void write_FRAM_long24(int16_t addr, int32_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>16)& 0xFF);
   write_FRAM(addrByte, end_cmd);  
//   addrByte = (uint8_t) ((data>>24)& 0xFF);
//   write_FRAM(addrByte, continue_cmd); 

  // set_FRAM_write_disable();
}
#elif defined(FRAM_8M)
void write_FRAM_long32(int32_t addr, int32_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>16)& 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) ((data>>24)& 0xFF);
   write_FRAM(addrByte, end_cmd); 

  // set_FRAM_write_disable();
}
void write_FRAM_long24(int32_t addr, int32_t data) {  
   uint8_t addrByte;

   set_FRAM_write_enable();
   write_FRAM(FRAM_SET_WRITE_ENABLE_LATCH, end_cmd);
   FRAM_CSn_0;
   write_FRAM(FRAM_WRITE_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

   addrByte = (uint8_t) (data & 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>8)& 0xFF);
   write_FRAM(addrByte, continue_cmd); 
   addrByte = (uint8_t) ((data>>16)& 0xFF);
   write_FRAM(addrByte, end_cmd);  
//   addrByte = (uint8_t) ((data>>24)& 0xFF);
//   write_FRAM(addrByte, continue_cmd); 

//   set_FRAM_write_disable();
}
#endif



#if defined(FRAM_64k)
uint8_t read_FRAM_byte(int16_t addr) { 
   uint8_t temp8;
   uint8_t addrByte;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd); 

  // FRAM_DATA_IN;
   temp8 = (int8_t)read_FRAM(end_cmd);  // 8 msb
   set_FRAM_read_disable();
   return(temp8);
}
#elif defined(FRAM_8M)
uint8_t read_FRAM_byte(int32_t addr) { 
   uint8_t temp8;
   uint8_t addrByte;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd); 

  // FRAM_DATA_IN;
   temp8 = (int8_t)read_FRAM(end_cmd);  // 8 msb
   set_FRAM_read_disable();
   return(temp8);
}
#endif




#if defined(FRAM_64k)
uint16_t read_FRAM_long16(int16_t addr) {  
   uint16_t temp16;
   uint8_t addrByte;
   uint8_t byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);
   
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp16 = (uint16_t)byteRead;  // 8 lsb
   byteRead = read_FRAM(end_cmd);
   temp16 = temp16 + (((uint16_t)byteRead) << 8);  // 8 msb
   set_FRAM_read_disable();
   return((uint16_t)temp16);
}
#elif defined(FRAM_8M)
uint16_t read_FRAM_long16(int32_t addr) {  
   uint16_t temp16;
   uint8_t addrByte;
   uint8_t byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);
   
   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp16 = (uint16_t)byteRead;  // 8 lsb
   byteRead = read_FRAM(end_cmd);
   temp16 = temp16 + (((uint16_t)byteRead) << 8);  // 8 msb
   set_FRAM_read_disable();
   return((int16_t)temp16);
}
#endif



#if defined(FRAM_64k)
uint16_t read_FRAM_u16(uint16_t addr) {  
   uint16_t temp16;
   uint8_t addrByte;
   uint8_t byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);
   
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp16 = (uint16_t)byteRead;  // 8 lsb
   byteRead = read_FRAM(end_cmd);
   temp16 = temp16 + (((uint16_t)byteRead) << 8);  // 8 msb
   set_FRAM_read_disable();
   return(temp16);
}
#elif defined(FRAM_8M)
uint16_t read_FRAM_u16(uint32_t addr) {  
   uint16_t temp16;
   uint8_t addrByte;
   uint8_t byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);
   
   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp16 = (uint16_t)byteRead;  // 8 lsb
   byteRead = read_FRAM(end_cmd);
   temp16 = temp16 + (((uint16_t)byteRead) << 8);  // 8 msb
   set_FRAM_read_disable();
   return(temp16);
}
#endif

#if defined(FRAM_64k)
uint32_t read_FRAM_long32(int16_t addr) {  
   uint32_t temp32;
   uint8_t addrByte, byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

 //  addrByte = (uint8_t) ((addr>>16) & 0xFF);
 //  write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp32 = (uint32_t)byteRead;  // 8 msb
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<8); 
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<16);
   byteRead = read_FRAM(end_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<24);
   set_FRAM_read_disable();
   return((int32_t)temp32);
}
uint32_t read_FRAM_long24(int16_t addr) {  
   uint32_t temp32;
   uint8_t addrByte, byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

 //  addrByte = (uint8_t) ((addr>>16) & 0xFF);
 //  write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp32 = (uint32_t)byteRead;  // 8 msb
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<8); 
   byteRead = read_FRAM(end_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<16);
 //  byteRead = read_FRAM(end_cmd);
 //  temp32 = temp32 + ((uint32_t)byteRead <<24);
   set_FRAM_read_disable();
   return((int32_t)temp32);
}
#elif defined(FRAM_8M)
uint32_t read_FRAM_long32(int32_t addr) {  
   uint32_t temp32;
   uint8_t addrByte, byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp32 = (uint32_t)byteRead;  // 8 msb
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<8); 
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<16);
   byteRead = read_FRAM(end_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<24);
   set_FRAM_read_disable();
   return((int32_t)temp32);
}
uint32_t read_FRAM_long24(int32_t addr) {  
   uint32_t temp32;
   uint8_t addrByte, byteRead;

   set_FRAM_read_enable();
  // FRAM_DATA_OUT;
   write_FRAM(FRAM_READ_MEMORY_DATA, continue_cmd);

   addrByte = (uint8_t) ((addr>>16) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  // MSB start address
   addrByte = (uint8_t) ((addr>>8) & 0xFF);
   write_FRAM(addrByte, continue_cmd);  
   addrByte = (uint8_t) (addr & 0xFF);
   write_FRAM(addrByte, continue_cmd);  

  // FRAM_DATA_IN;
   byteRead = read_FRAM(continue_cmd);
   temp32 = (uint32_t)byteRead;  // 8 msb
   byteRead = read_FRAM(continue_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<8); 
   byteRead = read_FRAM(end_cmd);
   temp32 = temp32 + ((uint32_t)byteRead <<16);
 //  byteRead = read_FRAM(end_cmd);
 //  temp32 = temp32 + ((uint32_t)byteRead <<24);
   set_FRAM_read_disable();
   return((int32_t)temp32);
}
#endif
