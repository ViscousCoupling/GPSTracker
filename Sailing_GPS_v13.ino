// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include "gps.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <stdlib.h>

// 11-Sep-2024 
// Started all over again, again. 
// New strategy is to just write the byte version of variables and then decode when read for analysis. 
// this should be much easier and work.  

// 18-Sep-2024 
// Now adding data monitoring 

// want the following buffer of data: 
// [year,month,day,hour,min,sec,millisecond,lat_hour, lat, long, angle, speed, heading,heel]
// Here's what it would looke like: 
// [YYYY,MO,DA,HR,SEC,MS,LD,LM,LS###,LD,LM,LS###,HEAD###,HEEL##]


// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define GPSECHO false
#define chipSelect 10
#define write_pin 9
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);



// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences

uint32_t timer = millis();
File myFile;


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
  while(!Serial); // wait for Serial to finish
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

//***********************************************************************************************

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");

  pinMode(write_pin, INPUT_PULLUP);

//**************************************************************************************************

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  } else {
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");

  }


}

void loop() // run over and over again
{
  
  char data_buff[DATA_SIZE]; 
  memset(data_buff, '0', sizeof(data_buff));

  char year_buff[UINT8_TO_CHAR], month_buff[UINT8_TO_CHAR], day_buff[UINT8_TO_CHAR], hour_buff[UINT8_TO_CHAR], minute_buff[UINT8_TO_CHAR], seconds_buff[UINT8_TO_CHAR];

  // Set char arrays to contain zeros 
  memset(year_buff, '0', sizeof(year_buff)); memset(month_buff, '0', sizeof(month_buff)); memset(day_buff, '0', sizeof(day_buff));
  memset(hour_buff, '0', sizeof(hour_buff)); memset(minute_buff, '0', sizeof(minute_buff)); memset(seconds_buff, '0', sizeof(seconds_buff));

  char accel_x_buff[MIN_WIDTH_2], accel_y_buff[MIN_WIDTH_2], accel_z_buff[MIN_WIDTH_2];
  memset(accel_x_buff, '0', sizeof(accel_x_buff));  memset(accel_y_buff, '0', sizeof(accel_y_buff));   memset(accel_z_buff, '0', sizeof(accel_z_buff)); 

  char magnetic_x_buff[MIN_WIDTH_2], magnetic_y_buff[MIN_WIDTH_2], magnetic_z_buff[MIN_WIDTH_2];
  memset(magnetic_x_buff, '0', sizeof(magnetic_x_buff));  memset(magnetic_y_buff, '0', sizeof(magnetic_y_buff));   memset(magnetic_z_buff, '0', sizeof(magnetic_z_buff)); 

  char lat_buff[CHAR_ARRAY_SIZE], long_buff[CHAR_ARRAY_SIZE], speed_buff[CHAR_ARRAY_SIZE], bearing_buff[CHAR_ARRAY_SIZE], angle_buff[CHAR_ARRAY_SIZE];
  memset(lat_buff, '0', sizeof(lat_buff));  memset(long_buff, '0', sizeof(long_buff));   memset(bearing_buff, '0', sizeof(bearing_buff)); memset(angle_buff, '0', sizeof(angle_buff)); 

  // create float variables. 
  float longitude, latitude, speed, bearing,  angle; 
  float accel_x, accel_y, accel_z; 
  float magnetic_x, magnetic_y, magnetic_z; 
  uint8_t day, month, year, minute, hour, sec; 
  uint16_t millisec;

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > UPDATE_TIME) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }

    // 11-Sep-2024: nmea_float_t confirmed to be of type float and 4 bytes long 
    // read in the data
    
    // year data 
    dtostrf(GPS.year, 2, 0, year_buff);
    data_buff[YEAR_POSITION] = year_buff[0];
    data_buff[YEAR_POSITION + 1] = year_buff[1];
    data_buff[YEAR_POSITION + 2] = ',';
    Serial.print("The data_buff before update_databuff is called: ");
    Serial.println(data_buff);

    // update_databuff(year_buff, UINT8_TO_CHAR, &data_buff[YEAR_POSITION]);

    // Serial.print("The data_buff after update_databuff is called: ");
    // Serial.println(data_buff);

    // month data
    dtostrf(GPS.month, 2, 0, month_buff);
    Serial.println(month_buff);
    data_buff[MONTH_POSITION] = month_buff[0];
    data_buff[MONTH_POSITION + 1] = month_buff[1];
    data_buff[MONTH_POSITION + 2] = ',';

    dtostrf(GPS.day, 2, 0, day_buff);
    data_buff[DAY_POSITION] = day_buff[0];
    data_buff[DAY_POSITION + 1] = day_buff[1];
    data_buff[DAY_POSITION + 2] = ',';

    dtostrf(GPS.hour, 2, 0, hour_buff);
    data_buff[HOUR_POSITION] = hour_buff[0];
    data_buff[HOUR_POSITION + 1] = hour_buff[1];
    data_buff[HOUR_POSITION + 2] = ',';

    dtostrf(GPS.minute, 2, 0, minute_buff);
    data_buff[MINUTE_POSITION] = minute_buff[0];
    data_buff[MINUTE_POSITION + 1] = minute_buff[1];
    data_buff[MINUTE_POSITION + 2] = ',';

    dtostrf(GPS.seconds, 2, 0, seconds_buff);
    data_buff[SECOND_POSITION] = seconds_buff[0];
    data_buff[SECOND_POSITION + 1] = seconds_buff[1];
    data_buff[SECOND_POSITION + 2] = ',';


    speed = GPS.speed;  //float
    angle = GPS.angle;  //float
  
    Serial.println("*************************************************************************************");
    
    // copy float lat long data over 
    latitude = GPS.latitude; // float
    longitude = GPS.longitude; // float
    dtostrf(GPS.latitude, MIN_WIDTH, PRECISION, lat_buff);
    dtostrf(GPS.longitude, MIN_WIDTH, PRECISION, long_buff);
    dtostrf(angle, MIN_WIDTH-3, PRECISION-3, angle_buff);

    for(int i = 0; i < MIN_WIDTH; i++){
      data_buff[LAT_POSITION + i] = lat_buff[i];
      data_buff[LONG_POSITION + i] = long_buff[i];
      data_buff[ANGLE_POSITION + i] = angle_buff[i];
    }

    for(int i = 0; i < MIN_WIDTH-5; i++){
      data_buff[SPEED_POSITION + i] = speed_buff[i];
    }

    data_buff[LAT_POSITION + MIN_WIDTH + 1] = ',';
    data_buff[LONG_POSITION + MIN_WIDTH + 1] = ',';
    data_buff[ANGLE_POSITION + MIN_WIDTH - 7] = ',';
    data_buff[SPEED_POSITION + MIN_WIDTH - 6] = ',';

    // copy float long data over 
    
    dtostrf(speed, MIN_WIDTH-5, PRECISION-5, speed_buff);




    Serial.println(lat_buff);
    Serial.println(long_buff);
    Serial.println(angle_buff);
    Serial.println(speed_buff);

// measure acceleration 
//*********************************************************************************************************************
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    dtostrf(accel_x, MIN_WIDTH_2, PRECISION_2, accel_x_buff);
    Serial.print("The value of x accel is: ");
    Serial.println(accel_x_buff);
    dtostrf(accel_y, MIN_WIDTH_2, PRECISION_2, accel_y_buff);
    Serial.print("The value of y accel is: ");
    Serial.println(accel_y_buff);
    dtostrf(accel_z, MIN_WIDTH_2, PRECISION_2, accel_z_buff);
    Serial.print("The value of z accel is: ");
    Serial.println(accel_z_buff);
  }

  for(int i = 0; i < MIN_WIDTH_2; i++){
    data_buff[ACCEL_X_POS + i] = accel_x_buff[i];
    data_buff[ACCEL_Y_POS + i] = accel_y_buff[i];
    data_buff[ACCEL_Z_POS + i] = accel_z_buff[i];
  }

  data_buff[ACCEL_X_POS + MIN_WIDTH_2] = ',';
  data_buff[ACCEL_Y_POS + MIN_WIDTH_2] = ',';
  data_buff[ACCEL_Z_POS + MIN_WIDTH_2] = ',';

// measure magnetic field. 


    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(magnetic_x, magnetic_y, magnetic_z);
      Serial.print("magnetic x: ");
      dtostrf(magnetic_x, MIN_WIDTH_2, PRECISION_2, magnetic_x_buff); 
      Serial.println(magnetic_x_buff);
      Serial.print("magnetic y: ");
      dtostrf(magnetic_y, MIN_WIDTH_2, PRECISION_2, magnetic_y_buff); 
      Serial.println(magnetic_y_buff);
      Serial.print("magnetic z: ");
      dtostrf(magnetic_z, MIN_WIDTH_2, PRECISION_2, magnetic_z_buff); 
      Serial.println(magnetic_z_buff);

    }

    for(int i = 0; i < MIN_WIDTH_2; i++){
      data_buff[MAG_X_POS + i] = magnetic_x_buff[i]; 
      data_buff[MAG_Y_POS + i] = magnetic_y_buff[i]; 
      data_buff[MAG_Z_POS + i] = magnetic_z_buff[i]; 

    }

      data_buff[MAG_X_POS + MIN_WIDTH_2 + 1] = ','; 
      data_buff[MAG_Y_POS + MIN_WIDTH_2 + 1] = ','; 
      data_buff[MAG_Z_POS + MIN_WIDTH_2 + 1] = '\n'; 

    Serial.println("The mag buffers are: ");
    Serial.println(magnetic_x_buff);
    Serial.println(magnetic_y_buff);
    Serial.println(magnetic_z_buff);

    Serial.print("The data_buff is: ");
    Serial.println(data_buff);

    // now to write data to the external drive... 
    //

    // SD CARD WRITE 
    if(!digitalRead(write_pin)){
        myFile = SD.open("DATA.txt", FILE_WRITE); // opens the file (Note name can't be longer than 8 letters)
        // for(int i = 0; i < DATA_SIZE; i++){
        //   myFile.print(data_buff[i]);
        // }
        // myFile.print("\n");
        myFile.println(data_buff);
        myFile.close(); // closes the file 
        Serial.println("Data written to external drive");

    }

    // need to fill the databuff with zeroes. 

  }

}

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

void update_databuff(char * ip, uint8_t length, char * data){

  int j = 0; 
  for(int i = 0; i < (length); i++){
      *(data + i) = *(ip + i);
  }
      *(data + length) = ',';

}

float heading(float x, float y) {
  double head = atan2(y, x); // Slope Y, Slope X
  return head;
}
