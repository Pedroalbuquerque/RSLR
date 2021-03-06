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

#include <RHReliableDatagram.h> //Get the specially customized RadioHead library for the Moteino here: http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <Adafruit_GPS.h> //Get the Adafruit GPS library here: https://github.com/adafruit/Adafruit_GPS/archive/master.zip
#include <SoftwareSerial.h> //We can't use the Hardware Serial because we need it to update the firmware
#include <SPI.h> // Get it here: https://www.github.com/lowpowerlab/spiflash
#include <RH_RF95.h>  // Get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip

#define SERVER_ADDRESS 1 // Ground Station ID
#define CLIENT_ADDRESS 2 // Remote Station ID (This node)
#define FREQUENCY   434 // Match frequency to the hardware version of the radio on your Moteino
#define LED           9 // Moteinos have LEDs on D9
#define FLASH_SS      8 // And FLASH SS on D8
//#define ADAFRUITGPS   // Uncomment if you're using Adafruit's Ultimate GPS Breakout
#define BUZZER        6 // Connect a buzzer to Digital pin 6
#define BATT_MONITOR A0 // Through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_FORMULA(reading) reading * 0.00318534 * 1.47
#define BATT_CYCLES 10  // The number of complete transmit loops before another reading is done

bool LEDstatus = false;

// Singleton instance of the radio driver
RH_RF95 driver; // Initialize the generic radio driver

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Define Struct for Data
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
	float latitude; // ddmm.mmmm
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
	float bat; // Battery Voltage
};
Payload Data;

// ****Initiate GPS module****
// Connect GPS module to the Following Pins:
// GPS VCC -> 3.3V Pin on Moteino
// GPS GND -> GROUND
// GPS TX  -> Digital pin 4
// GPS RX  -> Digital pin 3
SoftwareSerial mySerial(4, 3);
Adafruit_GPS GPS(&mySerial);

byte cycleCount = BATT_CYCLES;
byte sendLoop = 0;
float batteryVolts = 0;

void setup()
{
	pinMode(LED, OUTPUT);     // Activate LED output pin
	LEDstatus = true;         // Turn LED ON
	digitalWrite(LED,LEDstatus); 

	pinMode(BUZZER, OUTPUT); // Initiate BUZZER output pin
	
	// Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
	Serial.begin(115200);
	Serial.println F("GPS AND TELEMETRY MODULE");
	Serial.println(VERSION);
	
	// Initialize the radio
	if (manager.init())
	{
		driver.setFrequency(FREQUENCY);
	}
	else
	{
		Serial.println F(("Init failed..."));
	}
	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

		
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

	useInterrupt(true); // We will use the timer 0 interrupt

	delay(1000);

#ifdef ADAFRUITGPS
		// Ask for firmware version
		mySerial.println(PMTK_Q_RELEASE);
#endif

	pinMode(BATT_MONITOR, INPUT);
	checkBattery(); // Make a first readout of the Battery voltage

}


void loop()
{

	// If a sentence is received, we can check the checksum and parse it
	// Only sent by radio if new GPS message received
	if (GPS.newNMEAreceived())
	{
		if (!GPS.parse(GPS.lastNMEA()))   // This also sets the newNMEAreceived() flag to false
		{
			return;                         // We can fail to parse a sentence in which case we should just wait for another
			Serial.println F("NMEA not parsed!");
		}

		if (GPS.fix)
		{
			digitalWrite(LED, HIGH);
		}

		if (GPS.speed < 10)
		{
			digitalWrite(BUZZER, HIGH);
			delay(500);
			digitalWrite(BUZZER, LOW);
		}

		// Fill in the Payload struct with new values
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
		Data.bat = batteryVolts;

		// Now Send data to base module
		if (!manager.sendtoWait((uint8_t*)&Data, sizeof(Data), SERVER_ADDRESS))
			Serial.println F("Sending Data Packet failed!");
		delay(500);

		// invert LED status on each packet sent by radio to give visual feedback
		LEDstatus = switchstate(LEDstatus);
		digitalWrite(LED, LEDstatus);

		// Check is the required number of loops are complete to read the Battery voltage again
		sendLoop++;
		if (sendLoop >= BATT_CYCLES)
		{
			checkBattery();
			sendLoop = 0;
		}
	}
}


// ********  Function definitions **************

void checkBattery()
{
	unsigned int reading = 0;
	for (byte i = 0; i<10; i++)
		reading += analogRead(BATT_MONITOR);

	batteryVolts = BATT_FORMULA(reading / 10);
	Serial.print F("Bat voltage:"); Serial.println(batteryVolts);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) 
{
		char c = GPS.read();
		// If you want to debug, this is a good time to do it!
#ifdef UDR0
		if (GPSECHO)
				if (c) UDR0 = c;
		// Writing direct to UDR0 is much much faster than Serial.print 
		// But only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) 
{
		if (v) 
		{
				// Timer0 is already used for millis() - we'll just interrupt somewhere
				// in the middle and call the "Compare A" function above
				OCR0A = 0xAF;
				TIMSK0 |= _BV(OCIE0A);
		}
}

// *** Switch boolean var state
bool switchstate(bool st)
{
		st = !st;
		return st;
}


