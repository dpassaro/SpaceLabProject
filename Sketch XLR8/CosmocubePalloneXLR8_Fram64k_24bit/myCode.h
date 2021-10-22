// buffer circolare da 61 locazioni per memorizzare 60 secondi di storia di numEventi
// in modo da calcolare ogni secondo il rate mediato nei precedenti 60 secondi
//
// esempio con buffer da 6 locazioni volendo calcolare su 5 secondi
//
//    0 1 2 3 4
//      ^   ^
//      |   |
//      eP  bP     caso "wrapping": eP < bP
//  
//  timeInterval = 5 - (bP-eP)
//
//
//    0 1 2 3 4
//      ^   ^
//      |   |
//      bP  eP     caso "normale": eP > eP
//  
//  timeInterval = eP-bP
//
// ALGORITMO:  calcolo abs(eP-bP), a seconda del caso lo devo sottrarre da BUFFLEN o no
// eP, bP devono essere signed
//
/*
   void RDA_isr() {
   int8 t;
   buffer[next_in]=getc();
   t=next_in;
   next_in=(next_in+1) % BUFFER_SIZE;
   if(next_in==next_out)
     next_in=t;           // Buffer full !!
}

#define bkbhit_UART (next_in!=next_out)
#define clear_rs232_buff next_in=next_out

 */

//===================================================================
//      RATE MEDIO AL MINUTO CALCOLATO SUGLI ULTIMI 15 SECONDI    
//===================================================================

// BUFFER circolare inc ui scrivo ogni timer interval il numero di eventi corrente
#define BUFFLEN 32

// altri buffer usati per altre prove
//
// 61
//unsigned long evHistory[BUFFLEN] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// 11
//unsigned long evHistory[BUFFLEN] = {0,1,0,0,0,0,0,0,0,0,0};
// 32
uint32_t evHistory[BUFFLEN] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// 16
//   unsigned long evHistory[BUFFLEN] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int8_t beginPtr, endPtr;

// routine chiamata ad ogni secondo
uint32_t rate60sec(void) {
 uint8_t timeInterval;
 uint32_t events;
 //int8_t t8;

//Serial.print("\n\r\n\r Entro: numEventi="); Serial.print(numEventi); Serial.print(" bP= "); Serial.print(beginPtr); Serial.print(" eP= "); Serial.print(endPtr);
//Serial.print("\n\r [eP]= "); Serial.print(evHistory[endPtr]); Serial.print(" [bP]= "); Serial.print(evHistory[beginPtr]);

// scrittura di numEventi attuale nel buffer
//
// t8 = endPtr;
     endPtr=(endPtr+1) % BUFFLEN;  // punta alla prossima locazione

 if(endPtr==beginPtr) {  // buffer full, spostare in avanti anche beginPtr
   //  endPtr=t8;  
     beginPtr=(beginPtr+1) % BUFFLEN;   
 }         
// ora può calcolare
     evHistory[endPtr] = numTrigger; // scrivi il valore corrente di nemEventi cambiato il 15-3-2021
 
     timeInterval = abs(endPtr - beginPtr); // timeInterval: quanti interrupt timer tra i due pointer
     events = abs(evHistory[endPtr] - evHistory[beginPtr]);
 
     if(endPtr < beginPtr) { // caso di "wrapping"
        timeInterval = BUFFLEN - timeInterval;
  //    timeInterval = BUFFLEN/quantiPerSecondo - timeInterval;
     }

//Serial.print("timeInterval = "); Serial.print(timeInterval); Serial.print("  beginPtr = "); Serial.print(beginPtr); Serial.print("  endPtr = "); Serial.println(endPtr);
//Serial.print("  events = "); Serial.println(events); Serial.println();
// Serial.println((events*60*quantiPerSecondo)/timeInterval);
// Serial.println((events*60*quantiPerSecondo));
// quantiPerSecondo: quanti interrupt timer in un secondo
     return( (events*60*quantiPerSecondo)/timeInterval );  // eventi al minuto
// }
// else return(0);
//Serial.print("\n\r dopo : "); Serial.print(" bP= "); Serial.print(beginPtr); Serial.print(" eP= "); Serial.print(endPtr);

}
//===================================================================



void print_binary(int v, int num_places)
{
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {
        if (v & (0x0001 << (num_places-1))) Serial.print(F("1"));
        else Serial.print(F("0"));

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0)) Serial.print(F("_"));
    }
}

byte readByteBinary_EndByAnyChar(void) {
  byte byteValue=0;  // Max value is 65535
  char incomingByte;
 
 // if (Serial.available() > 0) {   // something came across serial
    byteValue = 0;         // throw away previous integerValue
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
     // if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '1') ) {
          Serial.write(incomingByte);
          byteValue *= 2;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          byteValue = ((incomingByte - 48) + byteValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(byteValue !=0) {
          Serial.write(incomingByte);
          byteValue /= 2;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
      else if (incomingByte != -1) break;   // exit the while(1) with any char
    }
   // Serial.println(integerValue);   // Do something with the value
   return(byteValue);
 // }
}


byte readByteBinary(char returnChar) {
  byte byteValue=0;  // Max value is 65535
  char incomingByte;
 
 // if (Serial.available() > 0) {   // something came across serial
    byteValue = 0;         // throw away previous integerValue
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '1') ) {
          Serial.write(incomingByte);
          byteValue *= 2;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          byteValue = ((incomingByte - 48) + byteValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(byteValue !=0) {
          Serial.write(incomingByte);
          byteValue /= 2;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
    }
   // Serial.println(integerValue);   // Do something with the value
   return(byteValue);
 // }
}

byte readByte_EndByAnyChar(void) {
  byte byteValue=0;  // Max value is 65535
  char incomingByte;
 
 // if (Serial.available() > 0) {   // something came across serial
    byteValue = 0;         // throw away previous integerValue
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          byteValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          byteValue = ((incomingByte - 48) + byteValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(byteValue !=0) {
          Serial.write(incomingByte);
          byteValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
      else if (incomingByte != -1) break;   // exit the while(1) with any char
    }
   // Serial.println(integerValue);   // Do something with the value
   return(byteValue);
 // }
}

byte readByte(char returnChar) {
  byte byteValue=0;  // Max value is 65535
  char incomingByte;
 
 // if (Serial.available() > 0) {   // something came across serial
    byteValue = 0;         // throw away previous integerValue
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          byteValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          byteValue = ((incomingByte - 48) + byteValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(byteValue !=0) {
          Serial.write(incomingByte);
          byteValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
    }
   // Serial.println(integerValue);   // Do something with the value
   return(byteValue);
 // }
}

unsigned long readULong_EndByAnyChar(void) {
  unsigned long ulongValue;
  char incomingByte;

    ulongValue=0;
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
     // if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          ulongValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          ulongValue = (((unsigned long)incomingByte - (unsigned long)48) + ulongValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(ulongValue !=0) {
          Serial.write(incomingByte);
          ulongValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
      else if (incomingByte != -1) break;   // exit the while(1) with any char
    }
   return(ulongValue);
}

uint32_t readULong32_EndByAnyChar(void) {
  uint32_t ulongValue;
  char incomingByte;

    ulongValue=0;
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
     // if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          ulongValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          ulongValue = (((uint32_t)incomingByte - (uint32_t)48) + ulongValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(ulongValue !=0) {
          Serial.write(incomingByte);
          ulongValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
      else if (incomingByte != -1) break;   // exit the while(1) with any char
    }
   return(ulongValue);
}

uint32_t readULong32(char returnChar) {
  uint32_t ulongValue;
  char incomingByte;

    ulongValue=0;
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          ulongValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          ulongValue = (((uint32_t)incomingByte - (uint32_t)48) + ulongValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(ulongValue !=0) {
          Serial.write(incomingByte);
          ulongValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
    //  else if (incomingByte != -1) break;   // exit the while(1) with any char
    }
   return(ulongValue);
}

unsigned long readULong(char returnChar) {
  unsigned long ulongValue;
  char incomingByte;

    ulongValue=0;
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == returnChar) break;   // exit the while(1), we're done receiving
      if ( (incomingByte >= '0') & (incomingByte <= '9') ) {
          Serial.write(incomingByte);
          ulongValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          ulongValue = (((unsigned long)incomingByte - (unsigned long)48) + ulongValue);
      }
      else if ( incomingByte == 0x7F ) { // backspace
        if(ulongValue !=0) {
          Serial.write(incomingByte);
          ulongValue /= 10;  // shift right 1 decimal place --> delete last decimal digit
        }
      }
    }
   return(ulongValue);
}
