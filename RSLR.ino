// This code allows a Moteino to send data from a GPS to another Moteino to be displayed, via radio
// on a display
// **********************************************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code/library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/3.0/
// **********************************************************************************************************
// Version LR 0.3 RC01

#define VERSION "RS LR MEGA 0.3 RC01"
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false


#include <Adafruit_GPS.h> //Get the Adafruit GPS library here: https://github.com/adafruit/Adafruit_GPS/archive/master.zip
#include <SoftwareSerial.h> //We can't use the Hardware Serial because we need it to update the firmware
#include <SPI.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <RH_RF95.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip

//Defining some Radio stuff
#define FREQUENCY   434 // Match frequency to the hardware version of the radio on your Moteino
#define LED           9 // Moteinos have LEDs on D9
#define FLASH_SS      8 // and FLASH SS on D8
//#define ADAFRUITGPS   // uncomment if you're using Adafruit's Ultimate GPS Breakout
#define BUZZER        6 // Connect a buzzer to Digital pin 6

bool LEDstatus = false;

RH_RF95 radio; //Initialize the radio

char nmea[64];

//Define Struct for Data
struct Payload
{
	char HD[3] = "/*"; // Marker defining the start of a data Packet
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
	uint16_t miliseconds;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	float groundspeed; // In knots
	float track; // Track over ground in degrees
	float latitude; //ddmm.mmmm
	char lat; // N/S
	float longitude; // dddmm.mmmm
	char lon; // E/W
	float altitude;	//	MSL Altitude in m
	uint8_t fixquality; // Same as 3D FIX
	uint8_t satellites; // Range 0 to 14
	float HDOP; // Horizontal Dilution of Precision <2.0 is good - https://en.wikipedia.org/wiki/Dilution_of_precision_(GPS) 
	float geoidheight;
	float latitudedeg;
	float longitudedeg;
	bool fix; // FIX 1/0
};
Payload Data;

//Define some variables
uint32_t timer = millis();

// ****Initiate GPS module****
// Connect GPS module to the Following Pins:
// GPS VCC -> 3.3V Pin on Moteino
// GPS GND -> GROUND
// GPS TX  -> Digital pin 4
// GPS RX  -> Digital pin 3
SoftwareSerial mySerial(4, 3);
Adafruit_GPS GPS(&mySerial);


// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


void setup()
{
	pinMode(LED, OUTPUT);     //activate LED output pin
	LEDstatus = true;         // Turn LED ON
	digitalWrite(LED,LEDstatus); 

	pinMode(BUZZER, OUTPUT); // initiate BUZZER output pin
	
	// connect at 115200 so we can read the GPS fast enough and echo without dropping chars
	Serial.begin(115200);
	Serial.println("GPS AND TELEMETRY MODULE");
	Serial.println(VERSION);
	
	//Initialize the radio
	if (!radio.init())
		Serial.println("init failed");
	else 
	{
		Serial.print("init OK - ");
		Serial.print(FREQUENCY); Serial.print("mhz"); 
	}
	 // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
		radio.setFrequency(FREQUENCY);
		
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(9600);

#ifdef ADAFRUITGPS
		//*****uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude****
		GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
		// uncomment this line to turn on only the "minimum recommended" data
		//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
		// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
		// the parser doesn't care about other sentences at this time

		//*****Set the update rate*****
		GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
		// For the parsing code to work nicely and have time to sort through the data, and
		// print it out we don't suggest using anything higher than 1 Hz

		// Request updates on antenna status, comment out to keep quiet
		//GPS.sendCommand(PGCMD_ANTENNA);
#endif


	// the nice thing about this code is you can have a timer0 interrupt go off
	// every 1 millisecond, and read data from the GPS for you. that makes the
	// loop code a heck of a lot easier!
	useInterrupt(true);

	delay(1000);

#ifdef ADAFRUITGPS
		// Ask for firmware version
		mySerial.println(PMTK_Q_RELEASE);
#endif
}



void loop()
{
		//Serial.print("#");
	// in case you are not using the interrupt above, you'll
	// need to 'hand query' the GPS, not suggested :(
	if (!usingInterrupt)
	{
		// read data from the GPS in the 'main loop'
		char c = GPS.read();
		// if you want to debug, this is a good time to do it!
		if (GPSECHO)
			if (c) Serial.print(c);
	}

	// if a sentence is received, we can check the checksum, parse it...
		// only sent by radio if new GPS message received
	if (GPS.newNMEAreceived()) {
		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
			{
					return;                         // we can fail to parse a sentence in which case we should just wait for another
					Serial.println("NMEA not parsed");
			}
				
		if (GPS.fix)
		{
			digitalWrite(LED, HIGH);
		/*	
			Serial.print("Location: ");
			Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
			Serial.print(", ");
			Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
			Serial.print("Location (in degrees, works with Google Maps): ");
			Serial.print(GPS.latitudeDegrees, 4);
			Serial.print(", ");
			Serial.println(GPS.longitudeDegrees, 4);
			Serial.print("Speed (knots): "); Serial.println(GPS.speed);
			Serial.print("Angle: "); Serial.println(GPS.angle);
			Serial.print("Altitude: "); Serial.println(GPS.altitude);
			Serial.print("Satellites: "); Serial.println(GPS.satellites);
			Serial.println();
		*/
		}
		
		if (GPS.speed < 10)
		{
			digitalWrite(BUZZER, HIGH);
			delay(500);
			digitalWrite(BUZZER, LOW);
		}



		//String nmeas(GPS.lastNMEA());

		//Serial.println(nmeas);



		//fill in the Payload struct with new values

		Data.hour = GPS.hour;
		Data.minute = GPS.minute;
		Data.seconds = GPS.seconds;
		Data.day = GPS.day;
		Data.month = GPS.month;
		Data.year = GPS.year;
		Data.miliseconds = GPS.milliseconds;
		Data.groundspeed = GPS.speed;
		Data.track = GPS.angle;
		Data.latitude = GPS.latitude;
		Data.lat = GPS.lat;
		Data.longitude = GPS.longitude;
		Data.lon = GPS.lon;
		Data.altitude = GPS.altitude;
		Data.fixquality = GPS.fixquality;
		Data.satellites = GPS.satellites;
		Data.HDOP = GPS.HDOP;
		Data.geoidheight = GPS.geoidheight;
		Data.latitudedeg = GPS.latitudeDegrees;
		Data.longitudedeg = GPS.longitudeDegrees;
		Data.fix = GPS.fix;
/*
		Serial.print("Location: ");
		Serial.print(Data.latitude, 4); Serial.print(Data.lat);
		Serial.print(", ");
		Serial.print(Data.longitude, 4); Serial.println(Data.lon);
		Serial.print("Location (in degrees, works with Google Maps): ");
		Serial.print(Data.latitudedeg, 4);
		Serial.print(", ");
		Serial.println(Data.longitudedeg, 4);
		Serial.print("GroundSpeed (knots?): "); Serial.println(Data.groundspeed);
		Serial.print("Angle: "); Serial.println("n/A");
		Serial.print("Altitude: "); Serial.println(Data.altitude);
		Serial.print("Satellites: "); Serial.println(Data.satellites);
		Serial.println();
*/

		//Serial.print("Size of data package:"); Serial.println(sizeof(Data));

		//Now Send data to base module

		radio.send( (uint8_t* ) &Data, sizeof(Data));
		//Serial.println(sizeof(Data));
		LEDstatus = switchstate(LEDstatus);
		digitalWrite(LED,LEDstatus);    // invert LED status on each packet sent by radio to give visual feedback
		//Serial.print("pack Sent:"); Serial.println(sizeof(Data));
	}  // if nmea received
		/*
		else
		{
				char bip[5]="bip!";
				radio.send((uint8_t *)bip,sizeof(bip));
		}
		*/
}


// ********  Function definitions **************

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
		char c = GPS.read();
		// if you want to debug, this is a good time to do it!
#ifdef UDR0
		if (GPSECHO)
				if (c) UDR0 = c;
		// writing direct to UDR0 is much much faster than Serial.print 
		// but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
		if (v) {
				// Timer0 is already used for millis() - we'll just interrupt somewhere
				// in the middle and call the "Compare A" function above
				OCR0A = 0xAF;
				TIMSK0 |= _BV(OCIE0A);
				usingInterrupt = true;
		}
		else {
				// do not call the interrupt function COMPA anymore
				TIMSK0 &= ~_BV(OCIE0A);
				usingInterrupt = false;
		}
}

// *** Switch boolean var state

bool switchstate(bool st)
{
		st = !st;
		return st;
}


