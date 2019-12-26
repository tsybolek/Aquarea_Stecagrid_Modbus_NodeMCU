/*This code is to:
  a) read aquarea data at CN-CNT connector over software serial
  b) read Steca data over RS-485 interface (NOT using modbus protocol)
  c) average the obtained values over the specified interval (duration can be set separately for aquarea and for Steca)
  d) dispose the average values over modbus TCP (as slave)

  Modbus implementation is based on a tutorial from http://trialcommand.com
  Thanks to Egyras for aquarea protocol decrypting, see https://github.com/Egyras/Panasonic-H-Aquarea

  For testing/debuugging (especially using CoolTerm):
  HEX array to query aquarea heat pump over CN-CNT:
  71 6c 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 12
  Example of heat pump answer:
  71 c8 01 10 56 55 62 49 00 05 00 00 00 00 00 00 00 00 00 00 19 15 11 55 16 5e 55 05 09 00 00 00 00 00 00 00 00 00 80 8f 80 8a b2 71 71 97 99 00 00 00 00 00 00 00 00 00 00 00 80 85 15 8a 85 85 d0 7b 78 1f 7e 1f 1f 79 79 8d 8d 9e 96 71 8f b7 a3 7b 8f 8e 85 80 8f 8a 94 9e 8a 8a 94 9e 82 90 8b 05 65 78 c1 0b 00 00 00 00 00 00 00 00 55 56 55 21 53 15 5a 05 12 12 19 00 00 00 00 00 00 00 00 e2 ce 0d 71 81 72 ce 0c 92 81 b0 00 aa 7c ab b0 32 32 9c b6 32 32 32 80 b7 af cd 9a ac 79 80 77 80 ff 91 01 29 59 00 00 3b 0b 1c 51 59 01 36 79 01 01 c3 02 00 dd 02 00 05 00 00 01 00 00 06 01 01 01 01 01 0a 14 00 00 00 77
  HEX array to request ACpower from StacaGrid:
  02 01 00 10 01 C9 65 40 03 00 01 29 7E 29 BE 03
  Example of a StecaGrid answer:
  02 01 00 1F C9 01 84 41 00 00 10 29 00 00 08 41 43 50 6F 77 65 72 3A 0B A2 78 85 FB 49 4C 03
*/



#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

const char* ssid = "SSID";
const char* password = "PSWD";
SoftwareSerial ss1; //for heat pump
SoftwareSerial ss2; //for inverter; you can test/debug by connecting Rx of ss2 with Rx of Serial, both with 38400 8N1
SoftwareSerial ss3; //for testing/debugging the code, use it for a loop back to ss1;

// Heat pump global variables:
float aEconsum = 0.0; //Electric comsumption averaged over the intervall
float aEproduct = 0.0; //Energy production averaged over the intervall
float aCOP = 0.0; //COP of HP averaged over the invervall
byte HPcount = 0; //counter for heat pump answers to calculate the average value
unsigned int HPwatchdog = 0; //starting time point for HP routine
unsigned int HPinterval = 10000; //interval duration vor averaging the HP values, milliseconds!
bool HPstart = true; // Flag for starting HP routine

// Inverter global variables:
float aACpower = 0.0; // AC power averaged over the specified interval
byte INVcount = 0; //counter for inverter answers to calculate the average value
unsigned int INVwatchdog = 0; //starting time point for inverter routine
unsigned int INVinterval = 10000; //interval duration for averaging the inverter values, milliseconds!
bool INVstart = true; // Flag for starting inverter routine

byte HPdata[203];  //buffer for storing the entire the aquarea answers

int ModbusTCP_port = 502;
//////// Required for Modbus TCP / IP /// Requerido para Modbus TCP/IP /////////
#define maxInputRegister 20
#define maxHoldingRegister 20

#define MB_FC_NONE 0
#define MB_FC_READ_REGISTERS 3 //implemented
#define MB_FC_WRITE_REGISTER 6 //implemented
#define MB_FC_WRITE_MULTIPLE_REGISTERS 16 //implemented
//
// MODBUS Error Codes
//
#define MB_EC_NONE 0
#define MB_EC_ILLEGAL_FUNCTION 1
#define MB_EC_ILLEGAL_DATA_ADDRESS 2
#define MB_EC_ILLEGAL_DATA_VALUE 3
#define MB_EC_SLAVE_DEVICE_FAILURE 4
//
// MODBUS MBAP offsets
//
#define MB_TCP_TID 0
#define MB_TCP_PID 2
#define MB_TCP_LEN 4
#define MB_TCP_UID 6
#define MB_TCP_FUNC 7
#define MB_TCP_REGISTER_START 8
#define MB_TCP_REGISTER_NUMBER 10

byte ByteArray[260];
unsigned int MBHoldingRegister[maxHoldingRegister];

//////////////////////////////////////////////////////////////////////////

WiFiServer MBServer(ModbusTCP_port);
WiFiClient client = MBServer.available();  //create the modbus client object in the global scope, even if it does not yet connected => in the loop we check it again and connect, if neccessary

void setup() {
  delay(5000);
  ss1.begin(9600, SWSERIAL_8E1, 4, 5, false, 256);  // (Baud rate, serial mode, RX pin, TX pin, normal logic=false;  inverse logic=true, buffsize - important, cause default is 64, which is too small for aquarea data)
  ss2.begin(38400, SWSERIAL_8N1, 2, 0);  // (Baud rate for Steca, serial mode for Steca, RX pin, TX pin, logic type: false=normal true=inverse, buffsize: default is 64)
  ss3.begin(9600, SWSERIAL_8E1, 255, 16);  // for emulating aquarea answers: loop back Tx of ss3 on Rx of ss1, then you can send examples of HP ansers over Serial
  Serial.begin(38400, SERIAL_8N1);  //for monitoring/debugging/testing
  Serial.println("All serials started, now connecting WiFi");
  WiFi.disconnect(); //Prevent connecting to wifi based on previous configuration
  IPAddress ip(10, 0, 0, 133); //Static IP address is important!!!
  IPAddress gateway(10, 0, 0, 138);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8); //DNS
  WiFi.hostname("ESP8266_primus");      // DHCP Hostname (useful for finding device for static lease)
  WiFi.config(ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  delay(100) ;
  Serial.println(".");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  MBServer.begin();
  Serial.println("Connected ");
  Serial.print("ESP8266 Slave Modbus TCP/IP ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(String(ModbusTCP_port));
  Serial.println("Modbus TCP/IP Online");
  delay(2000);
}


void loop() {
  //Start of heat pump routine:
  if (HPstart) { //record the start time point of the HP routine and set all global variables to zero:
    HPwatchdog = millis();
    aEconsum = 0.0;
    aEproduct = 0.0;
    aCOP = 0.0;
    HPcount = 0;
  }
  HPstart = false; //to prevent resetting everything in every loop
  float Econsum = 0.0;
  float Eproduct = 0.0;
  float COP = 0.0;
  if (get_ss1()) {  //only calculate if get HP data has been successfull, i.e. it returns true
    Serial.println("Get aquarea data returned TRUE...");
    Econsum = ((float)HPdata[193] - 1.0) * 200; //it is 1000 times the real value, to be rounded to an integer
    Eproduct = ((float)HPdata[194] - 1.0) * 200; //it is 1000 times the real value, to be rounded to an integer
    if (Econsum > 0.0) COP = Eproduct * 1000 / Econsum; //it is 1000 times the real value, to be rounded to an integer
    Serial.printf("Comsumption: %f ; Production: %f ; COP: %f \n", Econsum, Eproduct, COP);
    aEconsum += Econsum; //add the new value
    aEproduct += Eproduct; //add the new value
    HPcount++; //increase the devider by 1
  }
  else Serial.println("Get aquarea data returned FALSE...");
  if ((HPwatchdog + HPinterval) < millis()) {  //the interval is over
    if (HPcount > 0) {  //HP data has been obtained correctly at least once during the interval
      //first calculate the mathematically correct aCOP over the interval, i.e. SUM by SUM:
      if (aEconsum > 0.0) aCOP = aEproduct * 1000 / aEconsum; //it is 1000 times the real value, to be rounded to an integer
      aEconsum /= HPcount; //calculate the average value
      aEproduct /= HPcount; //calculate the average value
    }
    else {   //if no correct HP data has been obtained during the intervall, set all zero
      aEconsum = 0.0;
      aEproduct = 0.0;
      aCOP = 0.0;
    }
    Serial.printf("HPcount: %d ; aComsumption: %f ; aProduction: %f ; aCOP: %f \n", HPcount, aEconsum, aEproduct, aCOP);
    //Now update the modbus registers and reset the start:
    MBHoldingRegister[0] = (int)aEconsum;
    MBHoldingRegister[1] = (int)aEproduct;
    MBHoldingRegister[2] = (int)aCOP;
    HPstart = true;
  }
  //End of heat pump routine

  //Start of inverter routine
  if (INVstart) {  //record the start time point of the Steca routine and set all global variables to zero:
    INVwatchdog = millis();
    aACpower = 0.0;
    INVcount = 0;
  }
  INVstart = false; //to prevent resetting everything in every loop
  float ACpower = 0.0;
  if (get_ss2(ACpower)) {  //if got correct ACpower, add it to aACpower and increase INVcount by 1
    Serial.printf("Get data from Steca returned TRUE; ACpower = %f \n", ACpower);
    aACpower += ACpower;
    INVcount++;
  }
  else Serial.printf("Get data from Steca returned FALSE; ACpower = %f \n", ACpower);
  if ((INVwatchdog + INVinterval) < millis()) {  //the interval is over
    if (INVcount > 0) {  //Steca data has been obtained correctly at least once during the interval
      aACpower /= INVcount; //calculate the average value
    }
    else {   //if no correct Steca data has been obtained during the intervall, set zero
      aACpower = 0.0;
    }
    Serial.printf("INVcount: %d ; aACpower: %f \n", INVcount, aACpower);
    //Now update the modbus registers and reset the start flag:
    MBHoldingRegister[3] = (int)aACpower;
    INVstart = true;
  }
  // End of inverter routine


  ///////// Holding Register [0] A [9] = 10 Holding Registers Escritura
  ///////// Holding Register [0] A [9] = 10 Holding Registers Writing

  MBHoldingRegister[4] = random(0, 12);
  MBHoldingRegister[5] = random(0, 12);
  MBHoldingRegister[6] = random(0, 12);
  MBHoldingRegister[7] = random(0, 12);
  MBHoldingRegister[8] = random(0, 12);
  MBHoldingRegister[9] = random(0, 12);

  ///////// Holding Register [10] A [19] = 10 Holding Registers Lectura
  ///// Holding Register [10] A [19] = 10 Holding Registers Reading

  int Temporal[10];
  Temporal[0] = MBHoldingRegister[0];
  Temporal[1] = MBHoldingRegister[1];
  Temporal[2] = MBHoldingRegister[2];
  Temporal[3] = MBHoldingRegister[3];
  Temporal[4] = MBHoldingRegister[4];
  Temporal[5] = MBHoldingRegister[5];
  Temporal[6] = MBHoldingRegister[6];
  Temporal[7] = MBHoldingRegister[7];
  Temporal[8] = MBHoldingRegister[8];
  Temporal[9] = MBHoldingRegister[9];

  //// debug

  for (int i = 0; i < 4; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]:");
    Serial.print(Temporal[i]);
    Serial.print("; ");
  }
  Serial.println("");


  // Check if a client has connected // Modbus TCP/IP
  //WiFiClient client = MBServer.available();
  if (!client) {
    client = MBServer.available();
    //return;
  }

  boolean flagClientConnected = 0;
  byte byteFN = MB_FC_NONE;
  int Start;
  int WordDataLength;
  int ByteDataLength;
  int MessageLength;

  // Modbus TCP/IP
  //while (client.connected()) {
  if (client.connected()) {
    if (client.available())
    {
      flagClientConnected = 1;
      int i = 0;
      while (client.available())
      {
        ByteArray[i] = client.read();
        i++;
      }
      client.flush();


      ///// code here --- codigo aqui



      //// end code - fin


      //// rutine Modbus TCP
      byteFN = ByteArray[MB_TCP_FUNC];
      Start = word(ByteArray[MB_TCP_REGISTER_START], ByteArray[MB_TCP_REGISTER_START + 1]);
      WordDataLength = word(ByteArray[MB_TCP_REGISTER_NUMBER], ByteArray[MB_TCP_REGISTER_NUMBER + 1]);
    }

    // Handle request

    switch (byteFN) {
      case MB_FC_NONE:
        break;

      case MB_FC_READ_REGISTERS: // 03 Read Holding Registers
        ByteDataLength = WordDataLength * 2;
        ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
        ByteArray[8] = ByteDataLength; //Number of bytes after this one (or number of bytes of data).
        for (int i = 0; i < WordDataLength; i++)
        {
          ByteArray[ 9 + i * 2] = highByte(MBHoldingRegister[Start + i]);
          ByteArray[10 + i * 2] = lowByte(MBHoldingRegister[Start + i]);
        }
        MessageLength = ByteDataLength + 9;
        client.write((const uint8_t *)ByteArray, MessageLength);

        byteFN = MB_FC_NONE;

        break;


      case MB_FC_WRITE_REGISTER: // 06 Write Holding Register
        MBHoldingRegister[Start] = word(ByteArray[MB_TCP_REGISTER_NUMBER], ByteArray[MB_TCP_REGISTER_NUMBER + 1]);
        ByteArray[5] = 6; //Number of bytes after this one.
        MessageLength = 12;
        client.write((const uint8_t *)ByteArray, MessageLength);
        byteFN = MB_FC_NONE;
        break;

      case MB_FC_WRITE_MULTIPLE_REGISTERS: //16 Write Holding Registers
        ByteDataLength = WordDataLength * 2;
        ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
        for (int i = 0; i < WordDataLength; i++)
        {
          MBHoldingRegister[Start + i] = word(ByteArray[ 13 + i * 2], ByteArray[14 + i * 2]);
        }
        MessageLength = 12;
        client.write((const uint8_t *)ByteArray, MessageLength);
        byteFN = MB_FC_NONE;

        break;
    }
  }   //end of while client.connected
  delay(1000);
}

/*This function is to get aquarea data over software serial (ss1) and CN-CNT connector: */
bool get_ss1() {
  byte message[] = {0x71, 0x6c, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12};
  //This is the magic packet
  Serial.println("Sending magic packet...");
  ss1.write(message, sizeof(message));
  /*
    Serial.println("Send sample aquarea ansver over serial now! Waiting for 2 sec...");
    delay(2000);
    while (Serial.available()) {
    ss3.write(Serial.read());
    delay(2);
    }
  */
  int data_length = 0;
  for (byte i = 0; i < 203; i++) HPdata[i] = 0; //fill the array with zeros - so it is easier to detect erros later
  data_length = ss1.readBytes(HPdata, 203); // read Rx buffer as 203 byte array; note that the default timeout for this function is 1 second
  delay(20);  //important to wait until further data arrive
  while (ss1.available() > 0) { //empty the serial buffer from excessive data, if any
    delay(2);
    ss1.read();
  }
  Serial.println("Whole array received (203 bytes):");
  for (byte i = 0; i < 203; i++) {
    Serial.print(HPdata[i], HEX);
    Serial.print(",");
  }
  Serial.println("");
  if ((data_length != 203) or (HPdata[0] != 0x71) or (HPdata[1] != 0xc8)) {    //check the data length and header and abort if it is wrong
    Serial.println("Wrong HP data! Returning FALSE and aborting...");
    return false;
  }
  return true;
}


/*This function is to get Steca data over software serial (ss2) and RS485 interface module: */
bool get_ss2(float &fACpower) {  //this function to get Steca data over software serial (ss2)
  byte request[] = {0x02, 0x01, 0x00, 0x10, 0x01, 0xC9, 0x65, 0x40, 0x03, 0x00, 0x01, 0x29, 0x7E, 0x29, 0xBE, 0x03};   //This is the request to Steca for ACpower
  //Serial.println("Sending request to Steca...");
  ss2.write(request, sizeof(request));
  byte INVdata[31];  //buffer for storing the entire the StecaGrid answer
  for (byte i = 0; i < 31; i++) INVdata[i] = 0; //fill the array with zeros - so it is easier to detect erros later
  int data_length = 0;
  data_length = ss2.readBytes(INVdata, 31); // read Rx buffer as 31 byte array; note that the default timeout for this function is 1 second
  delay(20);  //important to wait until further data arrive
  while (ss2.available() > 0) { //empty the serial buffer from excessive data, if any
    ss2.read();
    delay(2);
  }
  if (data_length > 0) {   //if some data has been read
    if (INVdata[23] == 0x0B) {   //check the header; 0x0B means ACpower is > 0
      int iACpower = ((INVdata[26] << 8 | INVdata[24]) << 8 | INVdata[25]) << 7 ;   //"formula to float" conversion according to Steca
      fACpower = *((float*)&iACpower);  //convert HEX to float
      //Serial.println("fACpower from function: ");
      //Serial.println(fACpower);
    }
    else if (INVdata[23] == 0x0C) {  //check the header; 0x0C means ACpower = 0
      fACpower = 0.0;
    }
    else { // any other case with some data but different header
      Serial.println("Wrong data received. Setting ACpower to 0.0 and returning FALSE...");
      fACpower = 0.0;
      return false;
    }
  }
  else {  // if no data has been read for longer than the timeout for readbytes()
    Serial.println("No data received! Setting ACpower to 0.0 and returning FALSE...");
    fACpower = 0.0;
    return false;
  }
  return true;
}
