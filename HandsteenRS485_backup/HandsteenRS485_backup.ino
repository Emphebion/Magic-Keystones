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
//RGB Strip 1 (knop): D3(rood), D6(groen), D5(blauw)
#define rood1PIN 3
#define groen1PIN 6
#define blauw1PIN 5

//RGB strip 2 (basis): D9(rood), D10(groen), D11(blauw)
#define rood2PIN 9
#define groen2PIN 10
#define blauw2PIN 11

//RS485 Serial connection
#define RS485_RX 13 //7 RO
#define RS485_TX 12 //4 DI
#define RS485TraRec 8 //12 RE-DE
#define RS485Transmit HIGH
#define RS485Receive LOW

//RFID Serial connection
#define RFID_RX 7 //13
#define RFID_TX 4 //8
#define TB_READ_ONLY 0
#define RFID_IRQ_PIN 2

//***************************************************************************//
// Device dependent values                                                   //
//***************************************************************************//
#define deviceNumber 15
#define Fadespeed 1200

//***************************************************************************//
// Device dependent variables                                                //
//***************************************************************************//
// LED variables
int rood1Val;
int groen1Val;
int blauw1Val;

int rood2Val;
int groen2Val;
int blauw2Val;

volatile bool slotOpen = false;
volatile bool slotKraken = false;

// RS485 Variables:
SoftwareSerial rs485 = SoftwareSerial(RS485_RX, RS485_TX);
const byte STX = '\2';
const byte ETX = '\3';
byte message[6];
bool mfl = false;
byte rs485Buf[6];

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
bool skipOne = true;
int rfid_count = 0;

unsigned long skillTimeout = 300000;  // Time before a skill activates (300000 ms = 300 s = 5 min)
unsigned long toegangTimeout = 2500;  // Time before access is revoked (2500 ms = 2.5 s)
unsigned long rfidReadTimeout = 1000; // Time before another Tag can be read (200 ms)
unsigned long skillLEDTimeout = 250;  // Time step before a keystone LED changes one step (250 ms = 0.25 s)
unsigned long skillTimer = 0;
unsigned long toegangTimer = 0;
unsigned long rfidReadTimer = 0;
unsigned long skillLEDTimer = 0;
unsigned long timer = 0;

void setup(){
  Serial.begin(9600);
  Serial.println("TEST");
  
  pinMode(rood1PIN, OUTPUT);
  pinMode(groen1PIN, OUTPUT);
  pinMode(blauw1PIN, OUTPUT);
  pinMode(rood2PIN, OUTPUT);
  pinMode(groen2PIN, OUTPUT);
  pinMode(blauw2PIN, OUTPUT);
  
  pinMode(RFID_RX, INPUT);
  pinMode(RFID_TX, OUTPUT);
  
  attachInterrupt(rfid_irq, rfidRead, FALLING);
  
  rs485.begin(57600);
  pinMode(RS485TraRec, OUTPUT);
  digitalWrite(RS485TraRec, RS485Receive);

  setKeystoneLEDs(0,0,150);
  setBackgroundLEDs(80,20,200);
}

void loop(){
  timer = millis();
  
  if(rs485.available()){
    byte rs485Received = receiveMessage(rs485Buf, 6, 6);
    Serial.print("[");
    Serial.print(rs485Buf[0]);
    Serial.print(",");
    Serial.print(rs485Buf[1]);
    Serial.print(",");
    Serial.print(rs485Buf[2]);
    Serial.print(",");
    Serial.print(rs485Buf[3]);
    Serial.print(",");
    Serial.print(rs485Buf[4]);
    Serial.println("]");
    if (rs485Received == 5){
      if (rs485Buf[0] == deviceNumber){
        executeCommand(rs485Buf);
      }
    }
  }
  
  if(slotKraken){
    if((timer - skillTimer) >= skillTimeout){
//      setKeystoneLEDs(200,200,200);
//      delay(3000);
//      setKeystoneLEDs(0,200,0);
      slotKraken = false;
      rfidTagFound = false;
      skillTimer = timer;
    }
  }
  
  if(rdyRFID and !slotKraken){
    rs485.end();
    RFID.begin(9600);
    delay(1);
    if (!rfidTagFound){
      rfidTagFound = readRFIDTag(tag_buf);
    }
    RFID.end();
    rs485.begin(57600);
    toegangTimer = timer;
  }
  
  if((timer - rfidReadTimer) >= rfidReadTimeout){
    rdyRFID = false;
  }
  if((timer - toegangTimer) >= toegangTimeout){
    skillLEDTimeout = 250;
    slotOpen = false;
    slotKraken = false;
    rfidTagFound = false;

    blauw1Val = 150;
    setKeystoneLEDs(0,0,blauw1Val);
    skillTimer = timer;
//    toegangTimer = timer;
//    rfidReadTimer = timer;
  }
}

//***************************************************************************//
// Read the available Message                                                //
//***************************************************************************//
byte receiveMessage(byte* data, const byte length, unsigned int timeout){
  
  unsigned long start_time = millis ();

  bool have_stx = false;

  // variables below are set when we get an STX
  bool have_etx;
  byte input_pos;
  byte crc = 0;

  while (millis () - start_time < timeout){
    byte inByte = rs485.read();

    if(inByte == STX){ // start of text
      delay(1);
      have_stx = true;
      have_etx = false;
      input_pos = 0;
      start_time = millis ();  // reset timeout period
    }
    
    else if (inByte == ETX and !have_etx){   // end of text
      delay(1);
      have_etx = true;
    }

    else{
      if (!have_stx)
        break;

      // if we have the ETX this must be the CRC
      if (have_etx){
        if (crc != inByte){
          Serial.print(crc);
          Serial.print(" != ");
          Serial.println(inByte);
          return 0;  // bad crc
        }
        return input_pos;  // return received length
      }  // end if have ETX already

      // keep adding if not full
      if (input_pos <= length){
        data [input_pos++] = inByte;
        crc ^= inByte;
      }
      else
        return 0;  // overflow
        
    }  // end of switch
  }  // end of incoming data
  return 0;  // timeout
}

//*******************************************************//
// Execute the received command                          //
//*******************************************************//
byte executeCommand(byte* command) {
  if (command[1] == 11) {
    if (rfidTagFound){
      constructMessage(message, 0, tag_buf[0], tag_buf[1], tag_buf[2], tag_buf[3]);
    }
    else{
      constructMessage(message, 0, 0, 0, 0, 0);
    }
    sendResponce(message);
    Serial.println("Command 1 executed");
  }
  else if (command[1] == 12) {
    accessGranted();
    setKeystoneLEDs(command[2],command[3],command[4]);
    constructMessage(message, 0, 1, 0, 0, 0);
    sendResponce(message);
    rfidTagFound = false;
    Serial.println("Command 2 executed");
  }
  else if (command[1] == 13) {
    accessDenied();
    setKeystoneLEDs(command[2],command[3],command[4]);
    constructMessage(message, 0, 1, 0, 0, 0);
    sendResponce(message);
    rfidTagFound = false;
    Serial.println("Command 3 executed");
  }
  else if (command[1] == 14) {
    forcingAccess();
    setKeystoneLEDs(command[2],command[3],command[4]);
    constructMessage(message, 0, 1, 0, 0, 0);
    sendResponce(message);
    Serial.println("Command 4 executed");
  }
  else if (command[1] == 15) {
    setKeystoneLEDs(command[2],command[3],command[4]);
    constructMessage(message, 0, 1, 0, 0, 0);
    sendResponce(message);
    rfidTagFound = false;
    Serial.println("Empty command executed");
  }
}

//***************************************************************************//
// Access Granted                                                            //
//***************************************************************************//
void accessGranted(void){
  Serial.println("Key confirmed");
  toegangTimer = timer;
  setKeystoneLEDs(0,255,0);
}

//***************************************************************************//
// Access Denied                                                             //
//***************************************************************************//
void accessDenied(void){
  Serial.println("Key denied");
  toegangTimer = timer;
  setKeystoneLEDs(255,0,0);
}

//***************************************************************************//
// Forcing Access                                                            //
//***************************************************************************//
void forcingAccess(void){
  Serial.println("Lockpick confirmed: TODO execute stuff");
  toegangTimer = timer;
  Serial.println(rfid_count);
  if(rdyRFID){
    rfid_count = 0;
    slotKraken = true;
  }
  else{
    if(++rfid_count >= 5){
      slotKraken = false;
      rfidTagFound = false;
    }
  }
}

//***************************************************************************//
// Set stone LEDs                                                            //
//***************************************************************************//
void setKeystoneLEDs(byte red, byte green, byte blue){
  rood1Val = red;
  groen1Val = green;
  blauw1Val = blue;
  analogWrite(rood1PIN, rood1Val);
  analogWrite(groen1PIN, groen1Val);
  analogWrite(blauw1PIN, blauw1Val);
}

//***************************************************************************//
// Set Background LEDs                                                       //
//***************************************************************************//
void setBackgroundLEDs(byte red, byte green, byte blue){
  rood2Val = red;
  groen2Val = green;
  blauw2Val = blue;
  analogWrite(rood2PIN, rood2Val);
  analogWrite(groen2PIN, groen2Val);
  analogWrite(blauw2PIN, blauw2Val);
}

//*******************************************************//
// Send a Responce to the Controller                     //
//*******************************************************//
void sendResponce(byte* sendData){
//  delay(1);  // give the master a moment to prepare to receive
  digitalWrite(RS485TraRec, RS485Transmit);  // enable sending
  sendMessage(sendData, 5);

  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
    UCSR0A |= 1 << TXC0;  // mark transmission not complete
  while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete

  digitalWrite(RS485TraRec, RS485Receive);  // disable sending
}

//*******************************************************//
// Send a message                                        //
//*******************************************************//
void sendMessage (const byte * data, const byte length)
{
  rs485.write(STX);  // STX
  for (byte i = 0; i < length; i++)
    rs485.write(data[i]);
  rs485.write(ETX);  // ETX
  rs485.write(crc8 (data, length));
}  // end of sendMsg

//***************************************************************************//
// Cyclic redundancy check                                                   //
//***************************************************************************//
static byte crc8 (const byte *addr, byte len)
{
  byte crc = 0;
  while (len--){
    byte inbyte = *addr++;
    crc ^= inbyte;
  }
  return crc;
}  // end of crc8

//***************************************************************************//
// Construct a Message for Controller                                        //
//***************************************************************************//
void constructMessage(byte* data, byte address, byte command, byte data0, byte data1, byte data2) {
  data[0] = address;
  data[1] = command;
  data[2] = data0;
  data[3] = data1;
  data[4] = data2;
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
    Serial.println(" OK");
    result = true;
  }
  else{
    Serial.println(" CHECKSUM FAILED");
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
// INTERUPT SERVICE ROUTINES                                                 //
//***************************************************************************//
void rfidRead(void){
  // Lees alleen een waarde als er niet nog een verwerkt word
  if ( !rdyRFID ){
    rfidReadTimer = millis();
    rdyRFID = true;                            // Zet de rdyRFID flag
  }
}
