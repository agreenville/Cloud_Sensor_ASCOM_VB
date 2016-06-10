// include the library code:
#include <i2cmaster.h>
#include <OneWire.h>
#include <MovingAverageFilter.h>
#include <AltSoftSerial.h> //has to be pins (recieve) 8 and (transmit) 9

// DS18S20 Temperature chip i/o
OneWire ds(10);  // on pin 10

// for software serial
AltSoftSerial Serial_blue; // new serial port on pins 8 and 9

//moving ave
MovingAverageFilter movingAverageFilter(60); //I think ave of 60 readings

//ASCOM setup
boolean  csSafe = false;					//flag to hold safety state
boolean skytmp = false;
bool skytmpB=false; //for bluetooth serial

//Cloud Sensor Setup
char st1[30];
long int tpl; //Cloud sensor variable
long int CS_ObjTemp; //long int sky temp
long int CS_AmbTemp; //long int sensor temp
float CS_ObjTempF; //float
float CS_AmbTempF; 

long int Iram;
long int totemp; //amb temp
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
//start temp readings
 CloudGetTempFun();
 
//calc amb and sky temp difference
 delta_celsius = abs(CS_ObjTempF-totempF);
  
avg_delta_celsius=movingAverageFilter.process(delta_celsius); //calc moving ave

//condition for is_safe
 if (avg_delta_celsius <= 16*100) //should be <. less than 1600 or 16C is clear. try 16-21C difference
{
  csSafe=false;
  //Serial.print("not safe");
}
  else
{
  csSafe=true;
  //Serial.print("safe");
}

 // command protocols for vb program, ascom driver and android bluetooth app
 String cmd;							//initialise cmd for serial receive
	
if (Serial.available()>0){				 //if there is something on the ASCOM serial port	
  cmd = Serial.readStringUntil('#');	//read it until # (terminator)
if (cmd == "IS_SAFE"){					//check strings and do function getsafe
  getSafe();
}
else if (cmd == "CONNECT"){
       connect();       				//ASCOM checking to see if connected and run connect function
}
else if (cmd=="dc"){					//return delta C to serial port. For debugging and possible ascom enviro. implementation
   Serial.println(avg_delta_celsius/100);	
}
else if (cmd=="amb"){					//return amb temp C to serial port. For debugging and possible ascom enviro. implementation
  Serial.print(totempF/100);
  Serial.println("#");
}
else if (cmd=="skyT"){					//return sky temp to serial port. For debugging and possible ascom enviro. implementation
  Serial.print(CS_ObjTempF/100);
 Serial.println("#");
   
}
else if (cmd=="skyend"){				//command sent from android bluetooth app to stop temp data being written to serial port
  skytmp=false;
   //Serial.print("end");
}
else if (cmd=="sky"){					//command sent from android bluetooth app to start temp data being written to serial port
  skytmp=true;
  //Serial.print("sky");
}
else if (cmd=="TMP:00;"){				//command sent from vb cloud sensor program for temp data being written to serial port
  SerialDATAFun();
    
}   
}

// Same commands as above but to be written to bluetooth serial port
	
if (Serial_blue.available()>0){		    
  cmd = Serial_blue.readStringUntil('#');			
if (cmd == "IS_SAFE"){						
  getSafeB();
}
else if (cmd == "CONNECT"){
       connectB();       		
}
else if (cmd=="dc"){
   Serial_blue.println(avg_delta_celsius/100);	
}
else if (cmd=="amb"){
  Serial_blue.print(totempF/100);
  Serial_blue.println("#");
}
else if (cmd=="skyT"){
  Serial_blue.print(CS_ObjTempF/100);
  Serial_blue.println("#");
   
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

//graphing Andriod BlueTooth app functions

if (skytmp==true){
 graph();
}

//bluetooth serial version
if (skytmpB==true){
 graphB();
}

//DS18B20 Thermometer readings and calc

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
  ds.write(0x44, 1);				 // start conversion, with parasite power on at the end

  delay(1000);						 // maybe 750ms is enough, maybe not 2000
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);					 // Read Scratchpad


  for ( i = 0; i < 9; i++) {         // we need 9 bytes
    data[i] = ds.read();
  }


  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;					// test most sig bit
  if (SignBit)									// negative
  {
    TReading = (TReading ^ 0xffff) + 1;			// 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;		 // multiply by (100 * 0.0625) or 6.25

  totemp = (Tc_100);							//Temperature * 100
  totempF = totemp;								// same as above but for float variable

  if (SignBit)									// If its negative
  {
    totemp = (-totemp);
    totempF = (-totemp);						// same as above but for float variable
  } 
 

}

//***************************************************
//*****Start of User defined Functions **************
//***************************************************

//ASCOM
void connect(){
	Serial.println("GOTMESS#");				//response to connected check from ASCOM    
             }


void getSafe()								//respond to ASCOM with Safety State
{
	String retSafe = "SM_FALSE#";											
	if (csSafe){
	retSafe = "SM_TRUE#";
	}
	Serial.println(retSafe);
}

//bluetooth serial port verison
void connectB(){
	Serial_blue.println("GOTMESS#");		//response to connected check from ASCOM        
             }


void getSafeB()								//respond to ASCOM with Safety State
{
	String retSafe = "SM_FALSE#";										
	if (csSafe){
	retSafe = "SM_TRUE#";
	}
	Serial_blue.println(retSafe);
}

// temp data to serial port function for graphing with Andriod app
 void graph() 
 {
   Serial.print("*E");
   Serial.print(avg_delta_celsius/100); 
   Serial.print(",");
  Serial.print(totempF/100); //amb temp
   Serial.print(",");
   Serial.print(CS_ObjTempF/100);
   Serial.print(",");
   Serial.print(CS_AmbTempF/100); //sensor temp
   Serial.print("*");
  // Serial.print("\n");
 }

//bluetooth version (hmm... prob don't need both as sensor will only be connected to Andriod device by bluetooth)
 void graphB() 
 {
   Serial_blue.print("*E");
   Serial_blue.print(avg_delta_celsius/100); 
   Serial_blue.print(",");
   Serial_blue.print(totempF/100);
   Serial_blue.print(",");
   Serial_blue.print(CS_ObjTempF/100);
   Serial_blue.print(",");
   Serial_blue.print(CS_AmbTempF/100);
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
  i2c_start_wait(dev + I2C_WRITE);								// set device address and write mode
  if (TaTo) i2c_write(0x06); else i2c_write(0x07);              // or command read object or ambient temperature

  i2c_rep_start(dev + I2C_READ);								// set device address and read mode
  dlsb = i2c_readAck();											// read data lsb
  dmsb = i2c_readAck();											// read data msb
  pec = i2c_readNak();
  i2c_stop();

  lii = dmsb * 0x100 + dlsb;
  return (lii);
}


void CloudGetTempFun (void) {								//Set the number of Steps.
  tpl = readMLXtemperature(0);								// read sensor object temperature
  tpl = tpl * 10;
  tpl = tpl / 5;
  tpl = tpl - 27315;
  CS_ObjTemp = tpl;
  CS_ObjTempF = tpl;

   tpl = readMLXtemperature(1);								// read sensor ambient temperature
  tpl = tpl * 10;
  tpl = tpl / 5;
  tpl = tpl - 27315;
  CS_AmbTemp = tpl;
  CS_AmbTempF = tpl;
  
 // I'm not sure if this is needed below
  photocellReading0 = analogRead(photocellPin0);		// Read the analogue pin
  float Vout0 = photocellReading0 * 0.0048828125;		// calculate the voltage
  int lux0 = 500 / (Res0 * ((5 - Vout0) / Vout0));		// calculate the Lux
  lux = lux0;
  luxvolt = Vout0;
  // set the update flag so that the new position is displayed
 // UPDATE = true;
}


// Functions for printing to Serial ports for vb cloud sensor program
void SerialDATAFun (void) {						//Print the number of steps required on the 1st line of the LCD display
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
void SerialDATAFunB (void) {				//Print the number of steps required on the 1st line of the LCD display
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


