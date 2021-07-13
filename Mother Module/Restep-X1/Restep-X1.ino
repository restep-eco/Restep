/*
  Restep Power System Firmware
  Version X1 2/28/20
  Copyright 2021 RESTEP.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

  For 35AHr AGM Battery
  This firmware works in conjunction with the PLX-DAQ
*/

#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(0, 1); // RX, TX

//declare variables
int Ibat;
unsigned int PinOL, PinPV, Vpv, Vin, Vsys, Vbat, Q_COUNT, BSR, DieTemp, SysStat, ChargerState, ChargeStatus, Tmoth, Tol;
unsigned int index = 2;
unsigned int LastIndex = 2;
unsigned long timestamp = 0;
unsigned int deltaTime = 0;
unsigned int timer = 0;
bool PMBALERT = false;

//declare settings for LTC4015 and INA230
unsigned int ICHARGE_TARGET = 26;
unsigned int VCHARGE_SETTING = 27;
unsigned int VABSORB_DELTA = 42;
unsigned int CONFIG_BITS = 20;
unsigned int QCOUNT_PRESCALE_FACTOR = 18;
unsigned int INA230_CONFIG = 0;
unsigned int INA230_CAL = 5;

//declare colors for RGB LEDs
unsigned int black = 257;
unsigned int blue = 12545;
unsigned int green = 49409;
unsigned int aqua = 61697;
unsigned int red = 305;
unsigned int magenta = 12593;
unsigned int purple = 12577;
unsigned int yellow = 49457;
unsigned int orange = 33073;
unsigned int white = 61745;

//declare module addresses for Shift Registers
byte OL = 1;
byte PV = 3;
byte Mother = 5;
byte Battery = 7;

//initialize RGB LEDs to white (idle)
unsigned int BatteryColor = white;
unsigned int MotherColor = white;
unsigned int PVColor = white;
unsigned int OLColor = white;
  
bool usePV = false; //SSR & MPPT control, start by disabling the MPPT and enabling the SSR
bool absorb = false; //initially assume LTC4015 is not in absorb charge phase
bool ReadBSR = true; //Run a BSR test the first time charge current > 5A.  See below.

/*------------------------------------------------------- SETUP -------------------------------------------------------------------*/

void setup() {

  // set the data rate for the SoftwareSerial port for RGB LEDs on UART
  mySerial.begin(9600);
  
  Serial.begin(9600);  
  Serial.println("CLEARDATA"); //clears up any data left from previous projects
  //always write LABEL, so excel knows the next things will be the names of the columns
  Serial.println("LABEL, deltaTime, PinOL, PinPV, Vin, Vsys, Vbat, Ibat, QCOUNT, DieTemp, SysStat, ChargerState, ChargeStatus, Tmoth, Tol");
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  //use PE6 (Digital Pin 7) as ALERT interrupt, call alert function when ALERT pin goes low
  attachInterrupt(digitalPinToInterrupt(7), alert, FALLING);      
  pinMode(5, OUTPUT);               // initialize digital pin Digital Pin 5 (pin 31, PC6) as an output.
  pinMode(0, INPUT);                // initialize digital pin Digital Pin 0 (RX) as an input.
  pinMode(1, OUTPUT);               // initialize digital pin Digital Pin 1 (TX) as an output.
  digitalWrite(5, LOW);             // HIGH turns the 48V boost converter off
  Wire.begin(); 

  //Write Icharge Target (Maximum charge current target) = (ICHARGE_TARGET + 1) * 1mV/Rsnsb
  //Maximum charge current target = 8A, Rsnsb = 2mOhm, ICHARGE_TARGET = 15
  WriteLTC4015(ICHARGE_TARGET, 15);

  //Write Vcharge Setting (Charge voltage level) = (VCHARGE_SETTING/105.0 + 2.0)V/cell
  //Charge voltage level for AGM = 13.65V (2.275V/cell), VCHARGE_SETTING = 29
  WriteLTC4015(VCHARGE_SETTING, 29);

  //Write Vabsorb Delta (Absorb voltage level) = ((VABSORB_DELTA + VCHARGE_SETTING)/105.0 + 2.0)V/cell
  //Vabsorb Delta for AGM = 1.1, total Absorb Voltage = 14.75V (2.458V/cell), VABSORB_DELTA = 19
  WriteLTC4015(VABSORB_DELTA, 19);

  //Write Config Bits bit 4 to set force_meas_sys_on.  This forces the LTC4015 measurement system on 
  //even when there is no Vin (only Vbat).  Set en_qcount high to enable the Coulomb Counter.
  WriteLTC4015(CONFIG_BITS, 20);

  //35Ahr battery * 3600 = 126,000 Coulombs.  Rsns = 2mOhms
  //qLSB > 1.92C.  QCOUNT_PRESCALE_FACTOR = 64. qLSB = 3.84C
  //QCOUNT = 16384 = 0%, QCOUNT = 49152 = 100%
  //SOC = (QCOUNT-16384)/32768
  WriteLTC4015(QCOUNT_PRESCALE_FACTOR, 64);

  //Read the BSR.  Rseries = BSR*(2mOhm)*(6cells)/750.0
  BSR = ReadLTC4015(BSR, 65);

  //Set INA230 config registers: mode = shunt&bus continuous, #of averages = 4, conversion time = 4.156ms
  //avg2=0,avg1=0,avg0=1,VbusCT2=1,VbusCT1=1,VbusCT0=0,VshCT2=1,VshCT1=1,VshCT0=0,mode3=1,mode2=1,mode1=1
  //0100001110110111
  //Total power calculation time = 33ms
  WriteINA230(70, INA230_CONFIG, 17335);
  WriteINA230(69, INA230_CONFIG, 17335);
  
  //Calculate values for INA230 calibration register
  //Current_LSB = 0.001 (1mA per LSB)
  //Scaling_Factor = 0.00512
  //Rshunt = 0.004
  //CAL = Scaling_Factor/(Curr_LSB*Rshunt)
  //CAL = 1280
  //Write to the calibration register
  WriteINA230(70, INA230_CAL, 1280);
  WriteINA230(69, INA230_CAL, 1280);

  //TCN75A will deassert ALERT once a register is read
  //TCN75A ALERTs when TA > TSET and also when TA falls below THYST
  //See page 24 of TCN75A datasheet
  
  WriteTCN75Aconfig(72,1,2); //Set Mother Module Temp Sensor config to interrupt Mode
  WriteTCN75A(72,2,12800); //Set Mother Module Temp Sensor THYST to 50deg. C
  WriteTCN75A(72,3,15360); //Set Mother Module Temp Sensor TSET to 60deg. C
  //At 50degC top of FET gets 68degC
  //At 70degC top of FET gets 114degC
  //At 60degC top of FET gets 92degC
  
  WriteTCN75Aconfig(76,1,2); //Set OL Module Temp Sensor config to interrupt Mode
  WriteTCN75A(76,2,7680); //Set OL Module Temp Sensor THYST to 30deg. C
  WriteTCN75A(76,3,10240); //Set OL Module Temp Sensor TSET to 40deg. C

  RGB_LED(Mother, aqua, false);
  delay(5000); //Wait 5 seconds for the user to launch the PLX-DAQ
  RGB_LED(Mother, blue, false);

  //Read and construct the last index from locations 0 and 1 in the EEPROM
  LastIndex = ReadEEPROM(0) << 8;
  LastIndex += ReadEEPROM(1);
    
  while(index < LastIndex){
    //Write data to the PLX-DAQ Excel plugin via USB
    Serial.print("DATA,TIME,"); //writes the time in the first column A and the time since the measurements started in column B
    deltaTime = ReadEEPROM(index) << 8;
    index++;
    deltaTime += ReadEEPROM(index);
    index++;
    Serial.print(deltaTime);
    Serial.print(",");
    PinOL = ReadEEPROM(index) << 8;
    index++;
    PinOL += ReadEEPROM(index);
    index++;
    Serial.print(PinOL);
    Serial.print(",");
    PinPV = ReadEEPROM(index) << 8;
    index++;
    PinPV += ReadEEPROM(index);
    index++;
    Serial.print(PinPV);
    Serial.print(",");
    Vin = ReadEEPROM(index) << 8;
    index++;
    Vin += ReadEEPROM(index);
    index++;
    Serial.print(Vin);
    Serial.print(",");
    Vsys = ReadEEPROM(index) << 8;
    index++;
    Vsys += ReadEEPROM(index);
    index++;
    Serial.print(Vsys);
    Serial.print(",");
    Vbat = ReadEEPROM(index) << 8;
    index++;
    Vbat += ReadEEPROM(index);
    index++;
    Serial.print(Vbat);
    Serial.print(",");
    Ibat = ReadEEPROM(index) << 8;
    index++;
    Ibat += ReadEEPROM(index);
    index++;
    Serial.print(Ibat);
    Serial.print(",");
    Q_COUNT = ReadEEPROM(index) << 8;
    index++;
    Q_COUNT += ReadEEPROM(index);
    index++;
    Serial.print(Q_COUNT);
    Serial.print(",");
    DieTemp = ReadEEPROM(index) << 8;
    index++;
    DieTemp += ReadEEPROM(index);
    index++;
    Serial.print(DieTemp);
    Serial.print(",");
    SysStat = ReadEEPROM(index) << 8;
    index++;
    SysStat += ReadEEPROM(index);
    index++;
    Serial.print(SysStat);
    Serial.print(",");
    ChargerState = ReadEEPROM(index) << 8;
    index++;
    ChargerState += ReadEEPROM(index);
    index++;
    Serial.print(ChargerState);
    Serial.print(",");
    ChargeStatus = ReadEEPROM(index) << 8;
    index++;
    ChargeStatus += ReadEEPROM(index);
    index++;
    Serial.print(ChargeStatus);
    Serial.print(",");
    Tmoth = ReadEEPROM(index) << 8;
    index++;
    Tmoth += ReadEEPROM(index);
    index++;
    Serial.print(Tmoth);
    Serial.print(",");
    Tol = ReadEEPROM(index) << 8;
    index++;
    Tol += ReadEEPROM(index);
    index++;
    if(index < 36){ //if we're in the first row of data, write the BSR value in the Tol column
      Serial.println(BSR);
    }else Serial.println(Tol);
  }
  index = 2;
}

/*------------------------------------------------------- MAIN LOOP -------------------------------------------------------------------*/

void loop() {

  //blinky winky
  digitalWrite(LED_BUILTIN, HIGH);   
  delay(100);    
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);                       
  
  //Collect Telemetry
  //I2C communication
  //LTC4015 values
  Vin = ReadLTC4015(Vin, 59);
  Vsys = ReadLTC4015(Vsys, 60);
  Vbat = ReadLTC4015(Vbat, 58);
  Ibat = ReadLTC4015Int(Ibat, 61);
  SysStat = ReadLTC4015(SysStat, 57);
  ChargerState = ReadLTC4015(ChargerState, 52);
  ChargeStatus = ReadLTC4015(ChargeStatus, 53);

  //Off-Line INA230 Values
  PinOL = ReadINA230(70, PinOL, 3);

  //PV INA230 Values
  PinPV = ReadINA230(69, PinPV, 3);
  Vpv = ReadINA230(69, Vpv, 2);
  
  //Evaluate MPPT, SSR
  if(Vpv > 5000){ 
    usePV = true; //if Vpv > 6.25V (LSB = 1.25mV) PV can operate, disable SSR & enable MPPT
    RGB_LED(OL, OLColor, usePV); //always disable SSR before enabling MPPT
  }else{
    usePV = false; //else disable MPPT & enable SSR
    RGB_LED(Battery, BatteryColor, usePV); //always disable MPPT before enabling the SSR
  }
  
//Off-Line: 
  if(PinOL <= 40){ //1W
    OLColor = white; 
  }
  if(PinOL > 40){ //1W
    OLColor = orange;  
  }
  if(PinOL > 500){ //12.5W
    OLColor = yellow;  
  }
  if(PinOL > 1000){ //25W
    OLColor = green;  
  }
  if(PinOL > 2000){ //50W
    OLColor = aqua;  
  }
  if(PinOL > 3000){ //75W
    OLColor = blue;  
  }
  
//PV: 
  if(PinPV <= 40){ //1W
    PVColor = white; 
  }
  if(PinPV > 40){ //1W
    PVColor = orange;  
  }
  if(PinPV > 500){ //12.5W
    PVColor = yellow;  
  }
  if(PinPV > 1000){ //25W
    PVColor = green;  
  }
  if(PinPV > 2000){ //50W
    PVColor = aqua;  
  }
  if(PinPV > 3000){ //75W
    PVColor = blue;  
  }
  
//Battery: 
  if(ChargerState & 512){ //if charger is in absorb (bulk charge) state
    BatteryColor = orange;  
  }
  if(Ibat > 1365){ //1A
    BatteryColor = yellow;  
  }
  if(Ibat > 4096){ //3A
    BatteryColor = green;
    if(ReadBSR == true){
       WriteLTC4015(CONFIG_BITS, 52); //Set run_bsr=1. run_bsr will automatically = 0 when it's complete.
       ReadBSR = false;
    }  
  }
  if(Ibat > 6827){ //5A
    BatteryColor = aqua; 
    
  }
  if(Ibat > 9557){ //7A
    BatteryColor = blue;  
  }
  if(ChargerState & 64){ //if charger is in cc/cv (float) state
    BatteryColor = purple;  
  }
  if(ChargerState & 256){ //if charger is suspended
    BatteryColor = white;  
  }
  if(ChargerState & 2){ //if bat_missing_fault is asserted
    BatteryColor = red;
  }

  if(Vbat < 13653 || Q_COUNT < 22938){ //if battery voltage is < 10.5V or battery charge is < 20% hibernate
    Hibernate();
  }

//Mother:
  if(MotherColor != red){
    MotherColor = white; //if there is no fault, set Mother Color to white 
    if(SysStat & 2048){ //if MPPT is enabled
    MotherColor = magenta;
    }
    //EEPROM data overrun warning
    if(index > 28440){  //13cols*2bytes*60mins=1560
      if(index > 30000){
        MotherColor = orange;  //EEPROM full!
      }else MotherColor = yellow;  //1 hour of data storage remaining
    }                 
  }
  
  //Update RGB LEDs, SSR & MPPT
  RGB_LED(Battery, BatteryColor, usePV);
  RGB_LED(PV, PVColor, false);
  RGB_LED(Mother, MotherColor, true);
  RGB_LED(OL, OLColor, usePV);

  // Take temperatures, SOC and write telemetry when timer gets to 60 seconds
  // 278*(200+16.7) = 60sec
  //EEPROM is 32kBytes 30k/(2*13*60)=19hours
  if(timer == 277 && index < 30000){
    RGB_LED(Mother, blue, false);

    //TCN75A Values
    Tmoth = ReadTCN75A(72, Tmoth, 0);
    Tol = ReadTCN75A(76, Tol, 0);

    //Evaluate SOC
    if((ChargerState & 512) == 512){  //if the LTC4015 is in absorb charge phase
     absorb = true;
    }
    if(((ChargerState & 64) == 64) && absorb){  //if the LTC4015 just transitioned from absorb to CC/CV charge phase
      absorb = false;
      WriteLTC4015(19, 49152); //set QCOUNT to 100%
    }
    
    //LTC4015 Values
    Q_COUNT = ReadLTC4015(Q_COUNT, 19);
    DieTemp = ReadLTC4015(DieTemp, 63);
  
    //Evaluate the time elapsed (in seconds) since last telemetry write
    deltaTime = (unsigned int)((millis() - timestamp) / (unsigned long)(1000));
    timestamp = millis();
    
    WriteTelemetry(deltaTime);
    WriteTelemetry(PinOL);
    WriteTelemetry(PinPV);
    WriteTelemetry(Vin);
    WriteTelemetry(Vsys);
    WriteTelemetry(Vbat);
    WriteTelemetryInt(Ibat);
    WriteTelemetry(Q_COUNT);
    WriteTelemetry(DieTemp);
    WriteTelemetry(SysStat);
    WriteTelemetry(ChargerState);
    WriteTelemetry(ChargeStatus);
    WriteTelemetry(Tmoth);
    WriteTelemetry(Tol);
    UpdateLastIndex(index);
    RGB_LED(Mother, MotherColor, false);
    timer = 0; //reset timer
  }
  timer++;
  
  //ALERT HANDLING
  if(PMBALERT == true){

    //Read TCN75A Values
    Tmoth = ReadTCN75A(72, Tmoth, 0);
    Tol = ReadTCN75A(76, Tol, 0);
    
    if(Tmoth > 14080){ //55 degC
      //Mother over temp handling
      digitalWrite(5, HIGH);  // HIGH turns the 48V boost converter off
      MotherColor = red;
    }
    if(Tmoth < 14080){ //55 degC
      //Mother temp normal handling
      digitalWrite(5, LOW);  // LOW turns the 48V boost converter on
      MotherColor = white;
    }
    if(Tol > 8960){ //35 degC
      //OL over temp handling (turn OLPS off)
      OLColor = red;
      RGB_LED(OL, OLColor, true);
    }
    if(Tol < 8960){ //35 degC
      //OL temp normal handling
      OLColor = white;
      if(Vpv <= 5000){ //if PV is not available then enable the OLPS
        RGB_LED(OL, OLColor, false);
      }
    }

    PMBALERT = false; //reset PMBALERT
  }
}

/*------------------------------------------------------- FUNCTIONS -------------------------------------------------------------------*/

void Hibernate(){

  while(Vbat < 14693 || Q_COUNT < 24576){ //while battery voltage is < 11.3V or battery charge is < 25% remain in hibernation
    //The LTC4015 will keep doing its job of charging the battery given an input can provide power
  
    //First shut off the lights and enable the SSR for off-line power
    //We assume if there was solar power the battery woudn't have been depleted
    usePV = false; //else disable MPPT & enable SSR
    //Update RGB LEDs, SSR & MPPT
    RGB_LED(Battery, black, usePV);
    RGB_LED(PV, black, false);
    RGB_LED(Mother, black, true);
    RGB_LED(OL, black, usePV);
  
    //Then shut off the 48V converter
    digitalWrite(5, HIGH);  // HIGH turns the 48V boost converter off

    //Then disable the LTC4015 measurement system
    WriteLTC4015(CONFIG_BITS, 0);

    //Then wait 1 minute and blink the mother light red
    delay(30000);
    RGB_LED(Mother, red, true);
    delay(250);
    RGB_LED(Mother, black, true);
    delay(30000);
    RGB_LED(Mother, red, true);
    delay(250);
    RGB_LED(Mother, black, true);

    //Write Config Bits bit 4 to set force_meas_sys_on.  This forces the LTC4015 measurement system on 
    //even when there is no Vin (only Vbat).  Set en_qcount high to enable the Coulomb Counter.
    WriteLTC4015(CONFIG_BITS, 20);
    Vbat = ReadLTC4015(Vbat, 58);
    Q_COUNT = ReadLTC4015(Q_COUNT, 19);
  }
  //When exiting UVLO, turn the 48V converter back on
  digitalWrite(5, LOW);  // LOW turns the 48V boost converter on
  //Everything else should come back to normal in the main loop
}

void alert(){
  PMBALERT = true;
}

void WriteTelemetry(unsigned int data)
{
  byte byte1 = data; //set LSB
  byte byte2 = data >> 8; //set MSB
  WriteEEPROM(index, byte2);
  index++;
  WriteEEPROM(index, byte1);
  index++;
}

void WriteTelemetryInt(int data)
{
  byte byte1 = data; //set LSB
  byte byte2 = data >> 8; //set MSB
  WriteEEPROMInt(index, byte2);
  index++;
  WriteEEPROMInt(index, byte1);
  index++;
}

void UpdateLastIndex(unsigned int index)
{
  byte byte1 = index; //set LSB
  byte byte2 = index >> 8; //set MSB
  WriteEEPROM(0, byte2);
  WriteEEPROM(1, byte1);
  //This function is not the best idea, but I can write locations 0 and 1 
  //for 14 hours every day for over 3 years before I wear level the EEPROM
  //assuming 1 minute unsigned intervals.
}

void WriteEEPROM(unsigned int address, byte val)
{
  Wire.beginTransmission(80);
  Wire.write((byte)(address >> 8)); //write the MSB
  Wire.write((byte) address); //write the LSB
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}

void WriteEEPROMInt(int address, byte val)
{
  Wire.beginTransmission(80);
  Wire.write((byte)(address >> 8)); //write the MSB
  Wire.write((byte) address); //write the LSB
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}

void WriteINA230(byte addr, byte reg, unsigned int val)
{
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.write((byte)(val >> 8)); //write the MSB
   Wire.write((byte) val); //write the LSB
   Wire.endTransmission();
}

void WriteTCN75A(byte addr, byte reg, int val)
{
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.write((byte)(val >> 8)); //write the MSB
   Wire.write((byte) val); //write the LSB
   Wire.endTransmission();
}

void WriteTCN75Aconfig(byte addr, byte reg, byte val)
{
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.write(val); //write the MSB
   Wire.endTransmission();
}

void WriteLTC4015(byte reg, unsigned int val)
{
   Wire.beginTransmission(104);
   Wire.write(reg);
   Wire.write((byte) val); //write the LSB
   Wire.write((byte)(val >> 8)); //write the MSB
   Wire.endTransmission();
}

byte ReadEEPROM(unsigned int index)
{
  byte rData = 0XFF;
  Wire.beginTransmission(80);
  Wire.write((byte)(index >> 8)); //write the MSB
  Wire.write((byte)(index & 0xFF)); //write the LSB
  Wire.endTransmission();
  Wire.requestFrom(80, 1);
  rData = Wire.read();
  return rData;
}

unsigned int ReadTCN75A(int DevAddr, unsigned int Val, unsigned int ValAddr){
  Wire.beginTransmission(DevAddr);
  Wire.write(ValAddr);                  //Value or status register to read
  Wire.endTransmission();
  Wire.requestFrom(DevAddr, 2); 
  while (Wire.available()) {
     if (Wire.available() == 2) {
      Val = 256 * Wire.read();          //Shift the MSB
     } else {
      Val = Val + Wire.read();          //Add the LSB
     }
  }
  return(Val);
}

unsigned int ReadINA230(int DevAddr, unsigned int Val, unsigned int ValAddr){
  Wire.beginTransmission(DevAddr);
  Wire.write(ValAddr);                  //Value or status register to read
  Wire.endTransmission();
  Wire.requestFrom(DevAddr, 2); 
  while (Wire.available()) {
     if (Wire.available() == 2) {
      Val = 256 * Wire.read();          //Shift the MSB
     } else {
      Val = Val + Wire.read();          //Add the LSB
     }
  }
  return(Val);
}

unsigned int ReadLTC4015(unsigned int Val, unsigned int ValAddr){
  Wire.beginTransmission(104);          //LTC4015 is address 104
  Wire.write(ValAddr);                  //Value or status register to read
  Wire.endTransmission();
  Wire.beginTransmission(104);
  Wire.requestFrom(104, 2); 
  while (Wire.available()) {
      if (Wire.available() == 2) {
        Val = Wire.read();              //Read the LSB
      } else {
        Val = Val + 256 * Wire.read();  //Shift and add the MSB
      }
  }
  Wire.endTransmission();
  return(Val);
}

int ReadLTC4015Int(unsigned int Val, unsigned int ValAddr){
  Wire.beginTransmission(104);          //LTC4015 is address 104
  Wire.write(ValAddr);                  //Value or status register to read
  Wire.endTransmission();
  Wire.beginTransmission(104);
  Wire.requestFrom(104, 2); 
  while (Wire.available()) {
      if (Wire.available() == 2) {
        Val = Wire.read();              //Read the LSB
      } else {
        Val = Val + 256 * Wire.read();  //Shift and add the MSB
      }
  }
  Wire.endTransmission();
  return(Val);
}

void RGB_LED(byte module, unsigned int color, bool D6){

  byte LoByte = color;  
  LoByte |= module; //add the module address onto the color 
  if(module == OL){
    if(D6){
      LoByte << 2;
      LoByte >> 2; //if usePV is true, disable the SSR. Set bit 64 to 0 to set Y6 high.
    }else{
      LoByte |= 64; //if usePV is false, enable the SSR. Set bit 64 to 1 to set Y6 low.
    }
  }
  if(module == Battery){
    if(D6){
      LoByte |= 64; //if usePV is true, enable MPPT. Set bit 64 to 1 to set Y6 low.   
    }else{
      LoByte << 2;
      LoByte >> 2; //if usePV is true, disable the SSR. Set bit 64 to zero to set Y6 high.
    }
  }
  byte HiByte = color >> 8;
  HiByte |= module; //add the module address onto the color
  mySerial.write(HiByte);
  mySerial.write(LoByte);
}
