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
#include "EEPROM.h"
#include "TinyGPS.h"

#define gpsMac "1122,33,445500" //physical address of gps
//#define gpsMac "9C02,98,8C91BC" //physical address of gps
#define timeSampling 23000 //time between logs
#define bc4TimeOut 20000 //time to wait for GPS to send data

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
#define eeprom_init_button A10

// Variables
int ledArray [] = {10,11,12,13,9};
int geiger_input = 2;
long count = 0;
bool bc4_echo=false; //true to send bc4 traffic through Serial
int bc4_step; //bc4 state
TinyGPS gps;

void tinyGpsRestart () {
	//sends a fake NMEA sentence to  reset tinyGPS
	String nmea="$GPRMC,181544.000,V,,,,,0.03,0.00,220513,,,N*44\r\n";
	int counter;
	for (counter=0;counter<nmea.length();counter++) {
		gps.encode(nmea.charAt(counter));
	}
}


//eprom map
//0000-0050: eeprom counter
//0055-4095: logs, 10 bytes/log
#define log_record_size 10 //log format: lo lo lo lo la la la la ra ra
#define eeprom_mark_start 0 // 'R' 'A' 'D' 'S'
#define eeprom_counter_start 4 //location of start of eeprom counter
#define eeprom_counter_size 404 //404 //number of log records
#define log_start 55 //start of logs area

int eeprom_counter_byte_pointer;
byte eeprom_counter_bit_pointer;
byte eeprom_counter_read_byte (int address) {
	//returns 0-8
	byte value;
	byte pointer=0x80;
	byte counter;
	value=EEPROM.read(eeprom_counter_start+address);
	for (counter=8;counter>0;counter--) {
		if ((value & pointer)>0) return counter;
		pointer>>=1;
	}
	return 0;
}

void eeprom_counter_init () {
	byte counter;
	byte value;
	eeprom_counter_byte_pointer=0;
	eeprom_counter_bit_pointer=0;
	for (counter=0;counter<=(eeprom_counter_size-2)/8;counter++) {
		value=eeprom_counter_read_byte(counter);
		if (value==8)   {
			eeprom_counter_byte_pointer++;
		} else {
			eeprom_counter_bit_pointer=value;
			break;
		}
	}
}

unsigned int eeprom_counter_get_value() {
	return 8*eeprom_counter_byte_pointer+eeprom_counter_bit_pointer;
}

void eeprom_counter_increment () {
	unsigned int value;
	value=eeprom_counter_get_value();
	if ((value+1)>=eeprom_counter_size) { //log full
		eeprom_counter_reset();
	} else {
		eeprom_counter_bit_pointer++;
		if (eeprom_counter_bit_pointer>8) {
			eeprom_write8(eeprom_counter_start+eeprom_counter_byte_pointer,0xff);
			eeprom_counter_byte_pointer++;
			eeprom_counter_bit_pointer=1;
			eeprom_write8(eeprom_counter_start+eeprom_counter_byte_pointer,1);
		} else {
			byte new_bit=1;
			byte old_byte;
			new_bit<<=eeprom_counter_bit_pointer-1;
			old_byte=eeprom_read8(eeprom_counter_start+eeprom_counter_byte_pointer);
			eeprom_write8(eeprom_counter_start+eeprom_counter_byte_pointer,old_byte | new_bit);
		}
	}

}

void eeprom_counter_reset () {
	unsigned int counter;
	for (counter=0;counter<=(eeprom_counter_size-2)/8;counter++) {
		eeprom_write8(eeprom_counter_start+counter,0);
	}
	eeprom_counter_init(); //restart ram counter
}

void logAdd(long longitude, long latitude, unsigned int cpm) {
	unsigned int pointer;
	pointer=eeprom_counter_get_value()*log_record_size+log_start;
	eeprom_write32(pointer,longitude);
	eeprom_write32(pointer+4,latitude);
	eeprom_write16(pointer+8,cpm);
	eeprom_counter_increment();
}

bool logRead(unsigned int logNumber,long *longitude, long *latitude, unsigned int *cpm) {
	unsigned int pointer;
	pointer=logNumber*log_record_size+log_start;
	*longitude=eeprom_read32(pointer);
	*latitude=eeprom_read32(pointer+4);
	*cpm=eeprom_read16(pointer+8);
	if (*longitude==0 && *latitude==0 && *cpm==0) return false;
	return true;
}

void logDump() {
	//export log data in KML format
	unsigned int counter;
	long longitude;
	long latitude;
	unsigned int cpm;
	double dlongitude;
	double dlatitude;
	Serial.println("[BEGIN KML]");
	//header
	Serial.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"); 
	Serial.println ("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
	Serial.println ("<Placemark>");
	Serial.println ("<name>Radiation Map</name>");
	Serial.println ("<visibility>0</visibility>");
	Serial.println ("<Style id=\"redLineBluePoly\">");
	Serial.println ("<LineStyle><color>ff0000ff</color></LineStyle>");
	Serial.println ("<PolyStyle><color>ffff0000</color></PolyStyle>");
	Serial.println ("</Style>");
	Serial.println ("<styleUrl>#redLineBluePoly</styleUrl>");
	Serial.println ("<LineString>");
	Serial.println ("<extrude>1</extrude>");
	Serial.println ("<tessellate>1</tessellate>");
	Serial.println ("<altitudeMode>relativeToGround</altitudeMode>");
	Serial.println ("<coordinates> ");
	//logs
	for (counter=0;counter<eeprom_counter_size;counter++) {
		if (!logRead(counter,&longitude,&latitude,&cpm)) break;
		dlongitude=(double)longitude/100000;
		dlatitude=(double)latitude/100000;
		Serial.print(dlongitude,6);
		Serial.print(",");
		Serial.print(dlatitude,6);
		Serial.print(",");
		Serial.println(cpm);
	}
	Serial.println ("</coordinates></LineString></Placemark></kml>");
	Serial.println("[END KML]");

}

void eeprom_init() {
	//reset eeprom data & mark
	int counter;
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("EEPROM Init");
	Serial.println ("EEPROM Init");
	lcd.setCursor(0, 1);
	for (counter=0;counter<0xfff;counter++) {
		if (!(counter % 0x100)) lcd.print("."); //progress
		eeprom_write8(counter,0);
	}
	eeprom_write_String (eeprom_mark_start,"RADS");
	eeprom_counter_init(); //restart eeprom counter
}

 
  unsigned long eeprom_read32(int address) {
    //reads a 32 bits word from eeprom
    unsigned long temp=0;
    temp=EEPROM.read(address);
    temp<<=8;
    temp|=EEPROM.read(address+1);
    temp<<=8;
    temp|=EEPROM.read(address+2);
    temp<<=8;
    temp|=EEPROM.read(address+3);
    return temp;
  }
  unsigned long eeprom_read24(int address) {
    //reads a 24 bits word from eeprom
    unsigned long temp=0;
    temp=EEPROM.read(address);
    temp<<=8;
    temp|=EEPROM.read(address+1);
    temp<<=8;
    temp|=EEPROM.read(address+2);
    return temp;
  }
  unsigned int eeprom_read16(int address) {
    //reads a 16 bits word from eeprom
    unsigned int temp;
    byte high;
    byte low;
    high=EEPROM.read(address);
    low=EEPROM.read(address+1);
    temp=high<<8|low;
    return temp;
  }
  byte eeprom_read8 (int address) {
    //read a byte from eeprom
    return EEPROM.read(address);
  }
  void eeprom_read_string (int address, char *data, int n_data) {
    //reads n_data bytes from eeprom
    int counter;
    for (counter=0; counter<n_data;counter++) {
      data[counter]=EEPROM.read(address+counter);
    }
  }
  String eeprom_read_String (int address,int n_data) {
    //reads n_data bytes from eeprom and returns a String type
    String data="";
    int counter;
    for (counter=0; counter<n_data;counter++) {
      data=data+(char(EEPROM.read(address+counter)));
    }
    return data;
  } 
  String eeprom_read_hex_String(int address,int n_data) {
    //reads n_data bytes from eeprom and returns an hexadecimal String type
    String data="";
    String byte_hex;
    int counter;
    for (counter=0; counter<n_data;counter++) {
      byte_hex=String(EEPROM.read(address+counter),HEX);
      if (byte_hex.length()<2) data.concat("0");
      data.concat(byte_hex);
    }
    return data;
  }
  void eeprom_write8(int address,byte data) {
    //writes a byte in eeprom
    EEPROM.write(address,data);
  }
  void eeprom_write16(int address,unsigned int data) {
    //writes a 16bit word in eeprom
    byte high;
    byte low;
    low=(byte)(data&0x00ff);
    high=(byte)(data>>8);
    EEPROM.write(address,high);
    EEPROM.write(address+1,low);
  }
  void eeprom_write32(int address,unsigned long data) {
    //writes a 32bit word in eeprom
    unsigned long temp;
    byte high1;
    byte high2;
    byte low2;
    byte low1;
    temp=data;
    low2=(byte)(temp&0x000000ff);
    temp>>=8;
    low1=(byte)(temp&0x000000ff);
    temp>>=8;
    high2=(byte)(temp&0x000000ff);
    temp>>=8;
    high1=(byte)(temp);
    EEPROM.write(address,high1);
    EEPROM.write(address+1,high2);
    EEPROM.write(address+2,low1);
    EEPROM.write(address+3,low2);
  }
  void eeprom_write24(int address,unsigned long data) {
    //writes a 24bit word in eeprom
    unsigned long temp;
    byte high;
    byte low2;
    byte low1;
    temp=data;
    low2=(byte)(temp&0x000000ff);
    temp>>=8;
    low1=(byte)(temp&0x000000ff);
    temp>>=8;
    high=(byte)(temp&0x000000ff);
    EEPROM.write(address,high);
    EEPROM.write(address+1,low1);
    EEPROM.write(address+2,low2);
  }
  void eeprom_write_string (int address,char *data, int n_data) {
    //writes n_data bytes into eeprom
    int counter;
    for (counter=0; counter<n_data;counter++) {
      EEPROM.write(address+counter,data[counter]);
    }  
  }
  void eeprom_write_String (int address,String data) {
    //writes String into eeprom
    int counter;
    for (counter=0; counter<data.length();counter++) {
      EEPROM.write(address+counter,data.charAt(counter));
    }  
  }

  void hex_string_2_string (char *input,int n_input,char *output) {
    //takes an hex string and returns an ascii string
    int counter;
    char ascii_char[2];
    for (counter=0;counter<n_input;counter++) {
      hex_char_2_ascii(input[counter],ascii_char);
      output[2*counter]=ascii_char[0];
      output[2*counter+1]=ascii_char[1];
    }
  }

  unsigned char nibble_2_ascii(unsigned char nibble) {
  //returns ascii from hex nibble
  if (nibble<10) return '0'+nibble;
  return 'A'+nibble-10;
}
void hex_char_2_ascii(char input,char *output) {
  char nibble;
  output[0]=nibble_2_ascii((input & 0xf0)>>4);
  output[1]=nibble_2_ascii(input & 0x0F);
}

void serialEvent(String line) { //process input
	if (line.indexOf("DUMP")!=-1) {
		logDump(); //export data in KML format
	}
	if (line.indexOf("CLEAR")!=-1) {
		eeprom_init(); //reset eeprom
		
		Serial.println("EEPROM erased");
	}
	if (line.indexOf("ECHO ON")!=-1) {
		bc4_echo=true;; //
	}
	if (line.indexOf("ECHO OFF")!=-1) {
		bc4_echo=false;; //
	}

}

void setup(){
	pinMode(geiger_input, INPUT);
	digitalWrite(geiger_input,HIGH);
	pinMode(eeprom_init_button, INPUT);
	digitalWrite(eeprom_init_button,HIGH);
	for (int i=0;i<5;i++){
		pinMode(ledArray[i],OUTPUT);
	}
	//set up the LCD\'s number of columns and rows:
	lcd.begin(16, 2);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Radiation Sensor");
	lcd.setCursor(0,1);
	lcd.print("Board - Arduino");  
	delay(1000);
	if (eeprom_read_String(eeprom_mark_start,4)!="RADS") { //eeprom not initialized
		eeprom_init();
	}
	Serial.begin(38400);
	int counter;
	char eeprom_line[16];
    char str_eeprom_line[33];
	//eeprom_init();
	//eeprom_counter_reset();
	Serial.println("Leyendo Contador");
	eeprom_counter_init();
	Serial.print("Contador=");
	Serial.println(eeprom_counter_get_value());
	//log_add(0x11223344,0x11223344,eeprom_counter_get_value());
	//Serial.println("Incrementando Contador");
	//eeprom_counter_increment();
	
    Serial.println("EEPROM_Start");
    for (counter=0;counter<0x5;counter++) {
	    Serial.print(String(counter*16,HEX));
        Serial.print(" ");
        eeprom_read_string(counter*0x10,eeprom_line,16);
        hex_string_2_string(eeprom_line,16,str_eeprom_line);
        str_eeprom_line[32]=0;
        Serial.println(str_eeprom_line);
    }
	Serial.println("EEPROM_End"); 
	bc4_init();
	
	attachInterrupt(0,countPulse,FALLING);

}

void loop(){
	
	long lat, lon;
	unsigned long fix_age;

	unsigned long timeBc4LastCommand; //time of the last response from BC4
	unsigned long timeDisplayLastUpdate; //time of the last LCD refresh
	unsigned long timeEepromInitButtonPushed; //time of button push
	String bc4_line;
	String serialLine;

	long countPerMinute = 0;
	long timePreviousMeassure = 0;
	long timePreviousLog = 0;
	long time = 0;
	long countPrevious = 0;
	float radiationValue = 0.0;
	bool eepromInitButton=false;
	timeBc4LastCommand=millis();
	while (true) {
		//bc4 serial check & GPS process
		bc4_loop(); //connection sequence
		while (bc4_serial.available()) {
			char inByte = bc4_serial.read();
			gps.encode(inByte);
			bc4_line.concat(char(inByte));
			if (inByte=='\n') {
				if (bc4_echo) Serial.print("---->"+bc4_line);
				bc4_event(bc4_line);
				timeBc4LastCommand=millis();
				bc4_line="";
			}
		 } 
		while (Serial.available()) {
			char inByte2 = Serial.read();
			serialLine.concat(char(inByte2));
			if (inByte2=='\n') {
				serialEvent(serialLine);
				serialLine="";
			}
		 } 
		//bc4 time out
		if ((millis()-timeBc4LastCommand) > bc4TimeOut) { //lost sync with bc4
			unsigned long timeStartDisc;
			tinyGpsRestart();
			bc4_serial.print ("AT+DISC\r\n");
			Serial.print ("AT+DISC\r\n");
			timeStartDisc=millis();
			delay(1000);
			bc4serial_discard_line(1000);
			bc4serial_discard_line(1000);
			if ((millis()-timeStartDisc)<2900) { //bc4 is accepting commands
				bc4_init(); //reset & init bc4
			}
			timeBc4LastCommand=millis();
		}  

		//CPM
		if (millis()-timePreviousMeassure > 10000){
			timePreviousMeassure = millis();
			countPerMinute = 6*count;
			radiationValue = countPerMinute * CONV_FACTOR;
			count = 0;
		}

		//display & LED's update
		if (millis()-timeDisplayLastUpdate > 5000){
			timeDisplayLastUpdate=millis();
			lcd.clear();    
			lcd.setCursor(0, 0);
			//lcd.print("CPM=");
			//lcd.setCursor(4,0);
			lcd.print(countPerMinute);
			lcd.setCursor(6,0);
			lcd.print(radiationValue,4);
			lcd.setCursor(12,0);
			lcd.print("uSvh");
			Serial.print("cpm = "); 
			Serial.print(countPerMinute,DEC);
			Serial.print(" - ");
			Serial.print("uSv/h = ");
			Serial.println(radiationValue,4);   
			Serial.print("Satelites: ");
			Serial.println(gps.satellites());   
			Serial.print("Posicion: ");
			gps.get_position(&lat, &lon, &fix_age);
			Serial.print(lat);
			Serial.print(",");
			Serial.println(lon);
			lcd.setCursor(0, 1);
			if (bc4_step!=3) { //gps still not ready
				Serial.println ("Connecting GPS");
				lcd.print("Connecting GPS");
			} else{
				if ((millis()-timeBc4LastCommand) > bc4TimeOut/4) { //gps connection probably down
					Serial.println ("GPS OFF?");
					lcd.print("GPS OFF?");
				} else {
					if (gps.satellites() !=0xff && gps.satellites() >2) { //GPS working & location available
						lcd.print(lon);
						lcd.print(",");
						lcd.print(lat);
					} else { //location not available
						Serial.println ("No satellites");
						lcd.print("Waiting for Sats");
					}
				}
			}
			//led var setting  
			if(countPerMinute <= TH1) ledVar(0);
			if((countPerMinute <= TH2)&&(countPerMinute>TH1)) ledVar(1);
			if((countPerMinute <= TH3)&&(countPerMinute>TH2)) ledVar(2);
			if((countPerMinute <= TH4)&&(countPerMinute>TH3)) ledVar(3);
			if((countPerMinute <= TH5)&&(countPerMinute>TH4)) ledVar(4);
			if(countPerMinute>TH5) ledVar(5);
		}

		//log process
		if (millis()-timePreviousLog > timeSampling){
			timePreviousLog=millis();
			if (gps.satellites() !=0xff && gps.satellites() >2 && bc4_step==3) { //GPS working & location available
				logAdd(lon,lat,(unsigned int) countPerMinute);
				Serial.print ("LOG:");
				Serial.println (eeprom_counter_get_value());
			}
		}
		//check eeprom init button
		if (digitalRead(eeprom_init_button)==LOW) {
			if (!eepromInitButton) {
				timeEepromInitButtonPushed=millis();
				eepromInitButton=true;
			}
		} else {
			eepromInitButton=false;
		}
		if (eepromInitButton &&(millis()-timeEepromInitButtonPushed)>5000) {
			eeprom_init();
		}
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
		for(int i=5;i>0;i--){
			digitalWrite(ledArray[i],LOW);
		}
		digitalWrite(ledArray[0],HIGH);
	}
}

void bc4_init() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Bluetooth Init");
	lcd.setCursor(0, 1);
	bc4_serial.begin(38400);
	bc4_serial.print(F("AT+RESET\r\n"));  //restart BC4
	Serial.print(F("AT+RESET\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(1000);
	bc4_serial.print(F("AT+ROLE=1\r\n")); //Master mode
	Serial.print(F("AT+ROLE=1\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(2000);
	bc4_serial.print(F("AT+INIT\r\n")); //init SPP profile
	Serial.print(F("AT+INIT\r\n"));
	lcd.print(".");
	delay(500);
	bc4serial_discard_line(1000);
	bc4_serial.print(F("AT+IAC=9E8B33\r\n")); //all devices discoverable
	Serial.print(F("AT+IAC=9E8B33\r\n"));
	lcd.print(".");
	delay(500);
	bc4serial_discard_line(1000);
	bc4_serial.print(F("AT+PSWD=0000\r\n")); //PASS CODE="0000"
	Serial.print(F("AT+PSWD=0000\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(1000);
	bc4_serial.print(F("AT+CLASS=0\r\n")); //no CoD
	Serial.print(F("AT+CLASS=0\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(1000);
	bc4_serial.print(F("AT+INQM=1,1,1\r\n")); //RSSI inquiry, max 1 devices/1*1.28"
	Serial.print(F("AT+INQM=1,1,1\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(1000);
	bc4_step=0; //start connection sequence
/*	bc4_serial.print(F("AT+INQ\r\n")); //needed before at+link
	Serial.print(F("AT+INQ\r\n"));
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(4000);
	bc4_serial.print("at+link=");
	bc4_serial.println(gpsMac);
	Serial.print("at+link=");
	Serial.println(gpsMac);
	lcd.print(".");
	delay(1000);
	bc4serial_discard_line(6000); */
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

void bc4_loop() {
	if (bc4_step==0) {
		bc4_serial.print("AT+INQ\r\n"); //start discovery
		Serial.print("AT+INQ\r\n");
		bc4_step=1;
    }
    if (bc4_step==1) {
		return; //waits for devices or "OK" to finish 
    }    
    if (bc4_step==2) {
		return; //waits for link to succeed or fail
    }    
}
  void bc4_event(String line) { //process input
	if (bc4_step==1) {
		if (line.indexOf("OK")!=-1) { //end of inquiry. try linking
			bc4_serial.print("AT+LINK=");
			bc4_serial.println(gpsMac);
			Serial.print("AT+LINK=");
			Serial.println(gpsMac);
			bc4_step=2;
		}
		return;
	}  
    if (bc4_step==2) {
		if (line.indexOf("FAIL")!=-1) { //device not found
			bc4_step=1; //tries to reconnect
		} else {
			bc4_step=3; //device succesfully connected
		}
		return; //
    }    
    if (bc4_step==3) {
		if (line.indexOf("+DISC:LINK_LOSS")!=-1 || line.indexOf("+DISC:SUCCESS")!=-1) { //connection lost 1/2
			bc4_step=4; 
		} 
	}
    if (bc4_step==4) {
		if (line.indexOf("OK")!=-1) { //connection lost 2/2
			bc4_step=0; //restart connection
		} 

	}


	
  }