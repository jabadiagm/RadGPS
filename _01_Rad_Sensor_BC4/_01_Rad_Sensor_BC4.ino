/*
 *  ------Geiger Tube board (Arduino Code) Example--------
 *
 *  Explanation: This example shows how to get the signal from the Geiger Tube
 *  in Arduino, we use one of the Arduino interrupt pins (PIN2).
 *  We count the time (ms) between two pulses of the Geiger tube.
 *
 *  Copyright (C) 2011 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:		0.3
 *  Design:		Marcos Yarza, David Gascon
 *  Implementation:	Marcos Yarza
 */

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3,4,5,6,7,8);


// Threshold values for the led bar
#define TH1 45
#define TH2 95
#define TH3 200
#define TH4 400
#define TH5 600

// Conversion factor - CPM to uSV/h
#define CONV_FACTOR 0.00812

#define bc4_serial Serial1
String bc4_line;

// Variables
int ledArray [] = {10,11,12,13,9};
int geiger_input = 2;
long count = 0;
long countPerMinute = 0;
long timePrevious = 0;
long timePreviousMeassure = 0;
long time = 0;
long countPrevious = 0;
float radiationValue = 0.0;

void setup(){
  pinMode(geiger_input, INPUT);
  digitalWrite(geiger_input,HIGH);
  for (int i=0;i<5;i++){
    pinMode(ledArray[i],OUTPUT);
  }

  Serial.begin(38400);
  bc4_init();

  //set up the LCD\'s number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Radiation Sensor");
  lcd.setCursor(0,1);
  lcd.print("Board - Arduino");  
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Cooking Hacks");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,1);  
  lcd.print("www.cooking-hacks.com");
  delay(500);
  for (int i=0;i<5;i++){
    delay(200);  
    lcd.scrollDisplayLeft();
  }
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  - Libelium -");
  lcd.setCursor(0,1);
  lcd.print("www.libelium.com");    
  delay(1000);

  lcd.clear();  
  lcd.setCursor(0, 0);
  lcd.print("CPM=");
  lcd.setCursor(4,0);
  lcd.print(6*count);
  lcd.setCursor(0,1);
  lcd.print(radiationValue);

  attachInterrupt(0,countPulse,FALLING);

}

void loop(){

while (bc4_serial.available()) {
	int inByte = bc4_serial.read();
    bc4_line.concat(char(inByte));
    if (inByte=='\n') {
		//lcd.clear();    
		//lcd.setCursor(0, 0);
		//lcd.print(bc4_line);
		Serial.print("---->"+bc4_line);
        //time_bc4_last_command=millis();
        bc4_line="";
    }
 } 

  if (millis()-timePreviousMeassure > 10000){
    countPerMinute = 6*count;
    radiationValue = countPerMinute * CONV_FACTOR;
    timePreviousMeassure = millis();
    Serial.print("cpm = "); 
    Serial.print(countPerMinute,DEC);
    Serial.print(" - ");
    Serial.print("uSv/h = ");
    Serial.println(radiationValue,4);      
    lcd.clear();    
    lcd.setCursor(0, 0);
    lcd.print("CPM=");
    lcd.setCursor(4,0);
    lcd.print(countPerMinute);
    lcd.setCursor(0,1);
    lcd.print(radiationValue,4);
    lcd.setCursor(6,1);
    lcd.print(" uSv/h");

    //led var setting  
    if(countPerMinute <= TH1) ledVar(0);
    if((countPerMinute <= TH2)&&(countPerMinute>TH1)) ledVar(1);
    if((countPerMinute <= TH3)&&(countPerMinute>TH2)) ledVar(2);
    if((countPerMinute <= TH4)&&(countPerMinute>TH3)) ledVar(3);
    if((countPerMinute <= TH5)&&(countPerMinute>TH4)) ledVar(4);
    if(countPerMinute>TH5) ledVar(5);

    count = 0;

  }

}

void countPulse(){
  detachInterrupt(0);
  count++;
  while(digitalRead(2)==0){
  }
  attachInterrupt(0,countPulse,FALLING);
}

void ledVar(int value){
  if (value > 0){
    for(int i=0;i<=value;i++){
      digitalWrite(ledArray[i],HIGH);
    }
    for(int i=5;i>value;i--){
      digitalWrite(ledArray[i],LOW);
    }
  }
  else {
    for(int i=5;i>=0;i--){
      digitalWrite(ledArray[i],LOW);
    }
  }
}

  void bc4_init() {
    bc4_serial.begin(38400);
    bc4_serial.print(F("AT+RESET\r\n"));  //restart BC4
    delay(1000);
    bc4serial_discard_line(1000);
    bc4_serial.print(F("AT+ROLE=1\r\n")); //Master mode
    delay(1000);
    bc4serial_discard_line(2000);
    bc4_serial.print(F("AT+INIT\r\n")); //init SPP profile
    bc4serial_discard_line(1000);
    delay(500);
    bc4_serial.print(F("AT+IAC=9E8B33\r\n")); //all devices discoverable
    bc4serial_discard_line(1000);
    delay(500);
    bc4_serial.print(F("AT+PSWD=0000\r\n")); //PASS CODE="0000"
    bc4serial_discard_line(1000);
    delay(1000);    
    bc4serial_discard_line(1000);
    bc4_serial.print(F("AT+CLASS=0\r\n")); //no CoD
	Serial.print(F("AT+CLASS=0\r\n"));
    delay(1000);
    bc4serial_discard_line(1000);
    bc4_serial.print(F("AT+INQM=1,2,10\r\n")); //RSSI inquiry, max 999 devices/48*1.28"
    delay(1000);
	bc4_serial.print(F("AT+INQ\r\n")); //no CoD
	Serial.print(F("AT+INQ\r\n")); //no CoD
	delay(10000);
    bc4serial_discard_line(1000);
	bc4_serial.print(F("at+link=1122,33,445500\r\n"));
	Serial.print(F("at+link=1122,33,445500\r\n"));
	
  }

  void bc4serial_discard_line(unsigned int bc4time_out) { //wait for a whole line to te received
  long time_start; //
  time_start=millis();
  //String line;
  while ((millis()-time_start) < bc4time_out) {
    if (bc4_serial.available()) {
      int inByte = bc4_serial.read();
      //line.concat(char(inByte));      
      if (inByte=='\n') {
        //Serial.print("-D-"+line);
        return; //end of line
      }
    }
  }
  Serial.println(F("-D-Time_Out"));
}