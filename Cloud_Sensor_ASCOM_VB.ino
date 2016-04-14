// include the library code:
#include <i2cmaster.h>
#include <OneWire.h>
#include <MovingAverageFilter.h>
//#include <SoftwareSerial.h>
#include <AltSoftSerial.h> //has to be pins (recieve) 8 and (transmit) 9

// DS18S20 Temperature chip i/o
OneWire ds(10);  // on pin 10

// for software serial
//const int rxpin = 2;  //pin used to receive from bluetooth
//const int txpin = 3; //pin used to transmit to bluetooth
AltSoftSerial Serial_blue; //SoftwareSerial Serial_blue(rxpin, txpin); new serial port on pines 2 and 3

//moving ave
MovingAverageFilter movingAverageFilter(60); //I think ave of 60 readings

//ASCOM setup
boolean  csSafe = false;					//flag to hold safety state
boolean skytmp = false;
bool skytmpB=false; //for bluetooth serial

//Cloud Sensor Setup
char st1[30];
long int tpl; //Cloud sensor variable
long int CS_ObjTemp; //long int
long int CS_AmbTemp; //long int
float CS_ObjTempF; 
float CS_AmbTempF; 

long int Iram;
long int totemp;
float totempF;
int photocellPin0 = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading0;     // the analog reading from the analog resistor divider
float Res0 = 10.0;
int lux;
float luxvolt;
float delta_celsius; //long int
float avg_delta_celsius; //long int



void setup() {
   Serial.begin(9600);// start the serial 19200   bluetooth module default is 9600 com8 is bluetooth
   Serial_blue.begin(9600); //bluetooth sorftware serial
   PORTC = (1 << PORTC4) | (1 << PORTC5);  //enable internal pullup resistors on i2c ports 
  }

void loop() {
  
// ASCOM setup
//Somewhere in your code you will have to decide what is safe and what is not
 CloudGetTempFun();

 
 //Call something here to set csSafe accordingly
 delta_celsius = abs(CS_AmbTempF-CS_ObjTempF);
  
avg_delta_celsius=movingAverageFilter.process(delta_celsius); //calc moving ave
 
 if (avg_delta_celsius >= 16*100) //should be <. less than 1600 or 16C is clear. try 16-21C difference
{
  csSafe=false;
  //Serial.print("not safe");
}
  else
{
  csSafe=true;
  //Serial.print("safe");
}

 String cmd;					//initialise cmd for serial receive
	
if (Serial.available()>0){		    //if there is something on the ASCOM serial port........	
  cmd = Serial.readStringUntil('#');			//read it until # (terminator)
if (cmd == "IS_SAFE"){						//check strings and do stuff.........
  getSafe();
}
else if (cmd == "CONNECT"){
       connect();       		//ASCOM checking to see if we are here
}
else if (cmd=="dc"){
   Serial.println(avg_delta_celsius);	
}
else if (cmd=="amb"){
  Serial.println(CS_AmbTempF/100);
  //Serial.println("#");
}
else if (cmd=="skyT"){
  Serial.println(CS_ObjTempF/100);
  //Serial.print("#");
   
}
else if (cmd=="skyend"){
  skytmp=false;
   //Serial.print("end");
}
else if (cmd=="sky"){
  skytmp=true;
  //Serial.print("sky");
}
else if (cmd=="TMP:00;"){
  SerialDATAFun();
    
}   
}

// for bluetooth serial port
 //String cmd;					//initialise cmd for serial receive
	
if (Serial_blue.available()>0){		    //if there is something on the ASCOM serial port........	
  cmd = Serial_blue.readStringUntil('#');			//read it until # (terminator)
if (cmd == "IS_SAFE"){						//check strings and do stuff.........
  getSafeB();
}
else if (cmd == "CONNECT"){
       connectB();       		//ASCOM checking to see if we are here
}
else if (cmd=="dc"){
   Serial_blue.println(avg_delta_celsius);	
}
else if (cmd=="amb"){
  Serial_blue.println(CS_AmbTempF/100);
  //Serial.println("#");
}
else if (cmd=="skyT"){
  Serial_blue.println(CS_ObjTempF/100);
  //Serial.print("#");
   
}
else if (cmd=="skyend"){
  skytmpB=false;
   //Serial.print("end");
}
else if (cmd=="sky"){
  skytmpB=true;
  //Serial.print("sky");
}
else if (cmd=="TMP:00;"){
  SerialDATAFunB();
    
}   
}

//graphing BT app functions

if (skytmp==true){
 graph();
}

//bluetooth serial version
if (skytmpB==true){
 graphB();
}

//DS18B20 Thermometer

  byte k;
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;

  if ( !ds.search(addr)) {
    ds.reset_search();
    return;
  }


  for ( i = 0; i < 8; i++) {

  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    return;

  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not 2000
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad


  for ( i = 0; i < 9; i++) {          // we need 9 bytes
    data[i] = ds.read();
  }


  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  totemp = (Tc_100); //Temperature * 100
  totempF = totemp;

  if (SignBit) // If its negative
  {
    totemp = (-totemp);
    totempF = (-totemp);
  } 
 

}

//***************************************************
//*****Start of User defined Functions **************
//***************************************************

//ASCOM
// Add these to your code outside the main loop

void connect(){
	Serial.println("GOTMESS#");	//response to connected check from ASCOM  
              
             }


void getSafe()				//respond to ASCOM with Safety State
{
	String retSafe = "SM_FALSE#";		//should be SM_FALSE but typo in ascom (SM_FALS)								
	if (csSafe){
	retSafe = "SM_TRUE#";
	}
	Serial.println(retSafe);
}

//bluetooth serial verison
void connectB(){
	Serial_blue.println("GOTMESS#");	//response to connected check from ASCOM  
              
             }


void getSafeB()				//respond to ASCOM with Safety State
{
	String retSafe = "SM_FALSE#";		//should be SM_FALSE but typo in ascom (SM_FALS)								
	if (csSafe){
	retSafe = "SM_TRUE#";
	}
	Serial_blue.println(retSafe);
}

//
 void graph() 
 {
   Serial.print("*E");
   Serial.print(avg_delta_celsius/100); 
   Serial.print(",");
   Serial.print(CS_AmbTempF/100);
   Serial.print(",");
   Serial.print(CS_ObjTempF/100);
   Serial.print(",");
   Serial.print(totempF/100);
  Serial.print("*");
  // Serial.print("\n");
 }

//bluetooth version
 void graphB() 
 {
   Serial_blue.print("*E");
   Serial_blue.print(avg_delta_celsius/100); 
   Serial_blue.print(",");
   Serial_blue.print(CS_AmbTempF/100);
   Serial_blue.print(",");
   Serial_blue.print(CS_ObjTempF/100);
   Serial_blue.print(",");
   Serial_blue.print(totempF/100);
  Serial_blue.print("*");
  // Serial.print("\n");
 }


//Start of IR temperature Sensor (Cloud Sensor) FUNCTIONS
// read MLX90614 i2c ambient or object temperature
long int readMLXtemperature(int TaTo) {
  long int lii;
  int dlsb, dmsb, pec;
  int dev = 0x5A << 1;

  i2c_init();
  i2c_start_wait(dev + I2C_WRITE); // set device address and write mode
  if (TaTo) i2c_write(0x06); else i2c_write(0x07);                // or command read object or ambient temperature

  i2c_rep_start(dev + I2C_READ);  // set device address and read mode
  dlsb = i2c_readAck();       // read data lsb
  dmsb = i2c_readAck();      // read data msb
  pec = i2c_readNak();
  i2c_stop();

  lii = dmsb * 0x100 + dlsb;
  return (lii);
}


void CloudGetTempFun (void) {//Set the number of Steps.
  tpl = readMLXtemperature(0); // read sensor object temperature
  tpl = tpl * 10;
  tpl = tpl / 5;
  tpl = tpl - 27315;
  CS_ObjTemp = tpl;
  CS_ObjTempF = tpl;

   tpl = readMLXtemperature(1); // read sensor ambient temperature
  tpl = tpl * 10;
  tpl = tpl / 5;
  tpl = tpl - 27315;
  CS_AmbTemp = tpl;
  CS_AmbTempF = tpl;
  
  

  // CS_AmbTemp = totemp;

  photocellReading0 = analogRead(photocellPin0);   // Read the analogue pin
  float Vout0 = photocellReading0 * 0.0048828125;  // calculate the voltage
  int lux0 = 500 / (Res0 * ((5 - Vout0) / Vout0)); // calculate the Lux
  lux = lux0;
  luxvolt = Vout0;
  // set the update flag so that the new position is displayed
 // UPDATE = true;
}

//End of IR temperature sensor (Cloud Sensor) FUNCTIONS

void SerialDATAFun (void) {//Print the number of steps required on the 1st line of the LCD display
  Serial.print("#TOB:");
  Serial.print(CS_ObjTemp);
  Serial.println(";");
  Serial.print("#TAM:");

  Serial.print(totemp);
  Serial.println(";");

  Serial.print("#IRAM:");
  Serial.print(CS_AmbTemp);
  Serial.println(";");

}

//for bluetooth serial
void SerialDATAFunB (void) {//Print the number of steps required on the 1st line of the LCD display
  Serial_blue.print("#TOB:");
  Serial_blue.print(CS_ObjTemp);
  Serial_blue.println(";");
  Serial_blue.print("#TAM:");

  Serial_blue.print(totemp);
  Serial_blue.println(";");

  Serial_blue.print("#IRAM:");
  Serial_blue.print(CS_AmbTemp);
  Serial_blue.println(";");

}


