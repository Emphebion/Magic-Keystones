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
#define inputPIN1 8
#define inputPIN2 9
#define inputPIN3 14
#define inputPIN4 15
#define inputPIN5 16
#define inputPIN6 17
#define inputPIN7 2
#define inputPIN8 3
#define inputPIN9 4
#define inputPIN10 5
#define inputPIN11 6
#define inputPIN12 7

//RS485 Serial connection
#define RS485_RX 10 //13 RO
#define RS485_TX 12 //4 DI
#define RS485TraRec 11 //8 RE-DE
#define RS485Transmit HIGH
#define RS485Receive LOW

//***************************************************************************//
// Device dependent values                                                   //
//***************************************************************************//
#define deviceNumber 21

//***************************************************************************//
// Device dependent variables                                                //
//***************************************************************************//
// Input variables:
byte input_array[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
byte new_input_array[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
byte input_pins[12] = {inputPIN1,inputPIN2,inputPIN3,inputPIN4,inputPIN5,inputPIN6,inputPIN7,inputPIN8,inputPIN9,inputPIN10,inputPIN11,inputPIN12};
byte msg_buf[4] = {0,0,0,0};

// RS485 Variables:
SoftwareSerial rs485 = SoftwareSerial(RS485_RX, RS485_TX);
const byte STX = '\2';
const byte ETX = '\3';
byte message[6];
byte rs485Buf[6];

unsigned long debounce_timeout = 1000;  // Time before a skill activates (3000 ms = 3 s)
unsigned long input_debounce_timer = 0;
unsigned long timer = 0;

//***************************************************************************//
// SETUP                                                                     //
//***************************************************************************//
void setup(){
  Serial.begin(9600);
  Serial.println("TEST");
  setupInputs();
  
  rs485.begin(57600);
  pinMode(RS485TraRec, OUTPUT);
  digitalWrite(RS485TraRec, RS485Receive);
}

//***************************************************************************//
// MAIN LOOP                                                                 //
//***************************************************************************//
void loop(){
  timer = millis();
  
  if(rs485.available()){
    byte rs485Received = receiveMessage(rs485Buf, 6, 6);
    if (rs485Received == 5){
      if (rs485Buf[0] == deviceNumber){
        executeCommand(rs485Buf);
      }
    }
  }

  if((timer - input_debounce_timer) > debounce_timeout){
    readInputs();
  }
//  Serial.print("inputs are: ");
//  for(byte i = 0; i < 11; i++){
//    Serial.print(digitalRead(input_pins[i]));
//    Serial.print(",");
//  }
//  Serial.println(digitalRead(input_pins[11]));
}

void readInputs(){
  bool input_update = false;
  for (byte i = 0; i < 12; i++){
    new_input_array[i] = digitalRead(input_pins[i]);
    if(input_array[i] != new_input_array[i]){
      input_array[i] = new_input_array[i];
      input_debounce_timer = millis();
      input_update = true;
    }
  }
  if(input_update){
    parseInputs();
  }
}

//***************************************************************************//
// Parse inputs from attached devices                                        //
//***************************************************************************//
void parseInputs(){
  msg_buf[3] = 0;
  Serial.print("msg_buf = ");
  for(byte i = 0; i < 3; i++){
    msg_buf[i] = 0;
    for(byte j = 0; j < 4; j++){
      msg_buf[i] ^= digitalRead(input_pins[(i*4)+j]) << 2*j;
    }
    Serial.print(msg_buf[i]);
    Serial.print(",");
  }
  Serial.println("");
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
    constructMessage(message, 0, msg_buf[3], msg_buf[2], msg_buf[1], msg_buf[0]);
    sendResponce(message);
    Serial.println("Command 1 executed");
  }
  else if (command[1] == 12) {
//    for(byte i = 0; i < 4; i++){
//      msg_buf[i] = 0;
//    }
    constructMessage(message, 0, 1, 0, 0, 0);
    sendResponce(message);
    Serial.println("Command 2 executed");
  }
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
void sendMessage (const byte * data, const byte length){
  rs485.write(STX);  // STX
  for (byte i = 0; i < length; i++)
    rs485.write(data[i]);
  rs485.write(ETX);  // ETX
  rs485.write(crc8 (data, length));
}  // end of sendMsg

//***************************************************************************//
// Cyclic redundancy check                                                   //
//***************************************************************************//
static byte crc8 (const byte *addr, byte len){
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
// Setup all input pins                                                      //
//***************************************************************************//
void setupInputs(){
  pinMode(inputPIN1,INPUT);
  pinMode(inputPIN2,INPUT);
  pinMode(inputPIN3,INPUT);
  pinMode(inputPIN4,INPUT);
  pinMode(inputPIN5,INPUT);
  pinMode(inputPIN6,INPUT);
  pinMode(inputPIN7,INPUT);
  pinMode(inputPIN8,INPUT);
  pinMode(inputPIN9,INPUT);
  pinMode(inputPIN10,INPUT);
  pinMode(inputPIN11,INPUT);
  pinMode(inputPIN12,INPUT);

  digitalWrite(inputPIN1, LOW);
  digitalWrite(inputPIN2, LOW);
  digitalWrite(inputPIN3, LOW);
  digitalWrite(inputPIN4, LOW);
  digitalWrite(inputPIN5, LOW);
  digitalWrite(inputPIN6, LOW);
  digitalWrite(inputPIN7, LOW);
  digitalWrite(inputPIN8, LOW);
  digitalWrite(inputPIN9, LOW);
  digitalWrite(inputPIN10, LOW);
  digitalWrite(inputPIN11, LOW);
  digitalWrite(inputPIN12, LOW);
}
