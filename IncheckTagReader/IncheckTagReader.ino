//***************************************************************************//
// Emphebion RFID and RS485 program                                          //
// Versie 1.0                                                                //
// 29-11-2017                                                                //
//***************************************************************************//
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

//***************************************************************************//
// Pin Definitions                                                           //
//***************************************************************************//
//RFID Serial connection
#define RFID_RX 8
#define RFID_TX 4
#define TB_READ_ONLY 0
#define RFID_IRQ_PIN 2

//***************************************************************************//
// Device dependent variables                                                //
//***************************************************************************//
// RFID Variables:
SoftwareSerial RFID = SoftwareSerial(RFID_RX, RFID_TX);
const int rfid_irq = 0;
uint8_t buff[28];
char c_buff[28];
byte tag_buf[4];
uint8_t* buffer_at = buff;
uint8_t* buffer_end = buff + sizeof(buff);
volatile bool rdyRFID = false;
bool rfidTagFound = false;
uint32_t rfidTag = 0;

void setup(){
  Serial.begin(9600);
  Serial.println("TEST Connection");
  
  RFID.begin(9600);
  
  pinMode(RFID_RX, INPUT);
  pinMode(RFID_TX, OUTPUT);
  
  attachInterrupt(rfid_irq, rfidRead, FALLING);
}

void loop(){  
  if(rdyRFID){
    rfidTagFound = readRFIDTag(tag_buf);
    rfidTag = parseRFIDTag(tag_buf);
    Serial.println(rfidTag);
  }
  rdyRFID = false;
}

//***************************************************************************//
// Read Presented Tag                                                        //
//***************************************************************************//
bool readRFIDTag(byte* data){
  bool result = false;
  delay(350);
  // Vul de buffer
  buffer_at = buff;
  while ( buffer_at < buffer_end )
    *buffer_at++ = RFID.read();

  // Reset de buffer pointer om het uitlezen makkelijker te maken
  buffer_at = buff;
    
  // Skip the preamble
  ++buffer_at;
  // Begin de sommatie van de Checksum
  uint8_t checksum = rfidGetNext();
  
  // De tag bestaat uit nog 4 waardes
  for(int i = 0; i <= 3; i++){
    // Pak de volgende waarde
    uint8_t value = rfidGetNext();
    data[i] = value;
        
    // Xor de waarde met de checksum
    checksum ^= value;
  }
  
  // Vraag de checksum op
  uint8_t data_checksum = rfidGetNext();
  
  // Controleer de checksum
  if ( checksum == data_checksum ){
    result = true;
  }
  else{
    Serial.println("CHECKSUM FAILED: Scan tag again");
  }

  while(RFID.available()){
    RFID.read();
  }
  
  return result;
}

//***************************************************************************//
// RFID Functies                                                             //
//***************************************************************************//
uint8_t rfidGetNext(void)
{
  // sscanf needs a 2-byte space to put the result but we
  // only need one byte.
  uint16_t result;
 
  // Working space to assemble each byte
  static char byte_chars[3];
  
  // Pull out one byte from this position in the stream
  snprintf(byte_chars,3,"%c%c",buffer_at[0],buffer_at[1]);
  sscanf(byte_chars,"%x",&result);
  buffer_at += 2;
  
  return static_cast<uint8_t>(result);
}

//***************************************************************************//
// RFID Parse Functie                                                        //
//***************************************************************************//
uint32_t parseRFIDTag(byte* data){
  uint32_t result = 0;
  for(byte i = 0; i < 3; i++){
    result ^= data[i];
    result = result << 8; //(data[i] << (3-i)*8);
  }
  result ^= data[3];
  return result;
}

//***************************************************************************//
// INTERUPT SERVICE ROUTINES                                                 //
//***************************************************************************//
void rfidRead(void){
  // Lees alleen een waarde als er niet nog een verwerkt word
  if ( !rdyRFID ){
    rdyRFID = true;                            // Zet de rdyRFID flag
  }
}
