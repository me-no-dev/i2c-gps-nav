/*********************************************
 * I2CGPS  - Inteligent GPS and NAV module for MultiWii by EOSBandi
 * V0.1   
 *
 * This module is based on the idea about offloading GPS parsing and calculations from a low powered flight controller processor (AtMega328) 
 * and make all data available via the I2C bus.
 * Beside of GPS data the module also implement waypoints and pos hold functions. 
 * There are 15 Waypoints (#0 is the RTH position), these waypoints can be set via I2C and module gives back the distance and direction towards the active
 * waypoint. 
 * Pos hold can overwrite the active waypoint temporary and then can continue toward the last active waypoint.
 * waypoints also can be set to the current gps position, (This can be used for waypoint recording during flight and replay)
 *
 * Need an updated version of Wire library, since current one does not handle repeated start in slave mode.
 *  
 * ToDo: Implement cross track error calculations and wind estimation
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
*********************************************************************************/



#include <Wire.h>

#define I2C_ADDRESS        0x20                    //7 bit address 0x40 write, 0x41 read
#define REG_MAP_SIZE       sizeof(i2c_dataset)     //size of register map
#define MAX_SENT_BYTES 0x09                      //maximum amount of data that I could receive from a master device (register, plus 8 byte gps coordinate)

/*************** I2C GSP register definitions *********************************/

#define I2C_GPS_STATUS                            0x00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01
        #define I2C_GPS_STATUS_2DFIX          0x02
        #define I2C_GPS_STATUS_3DFIX          0x04
        #define I2C_GPS_STATUS_WP_REACHED     0x08      //Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0

#define I2C_GPS_COMMAND                           0x01 //(write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      //copy current position to internal target location register
        #define I2C_GPS_COMMAND_RESUME        0x02      //copy last active WP to internal target location register
        #define I2C_GPS_COMMAND_SET_WP        0x04      //copy current position to given WP
        #define I2C_GPS_COMMAND_ACTIVATE_WP   0x08      //copy given WP position to internal target location register
        #define I2C_GPS_COMMAND_WP            0xF0      //Waypoint number

#define I2C_GPS_WP_REG                            0x06   //Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE          0x0F      //Active Waypoint
        #define I2C_GPS_WP_REG_PERVIOUS        0xF0      //pervious Waypoint
        
#define I2C_GPS_GROUND_SPEED                      0x07   //GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                          0x09   //GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_TIME                              0x0b   //UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_DISTANCE                          0x0f   //Distance between current pos and internal target location register in meters (uint16_t) (Read Only)
#define I2C_GPS_DIRECTION                         0x11   //direction towards interal target location reg from current position (+/- 180 degree)    (read Only)
#define I2C_GPS_LOCATION                          0x13   //current position (8 bytes, lat and lon, 1 degree = 10 000 000                           (read only)
#define I2C_GPS_WP0                               0x1B   //Waypoint 0 used for RTH location      (R/W)
#define I2C_GPS_WP1                               0x23
#define I2C_GPS_WP2                               0x2B
#define I2C_GPS_WP3                               0x33
#define I2C_GPS_WP4                               0x3B
#define I2C_GPS_WP5                               0x43
#define I2C_GPS_WP6                               0x4B
#define I2C_GPS_WP7                               0x53
#define I2C_GPS_WP8                               0x5B
#define I2C_GPS_WP9                               0x63
#define I2C_GPS_WP10                              0x6B
#define I2C_GPS_WP11                              0x73
#define I2C_GPS_WP12                              0x7B
#define I2C_GPS_WP13                              0x83
#define I2C_GPS_WP14                              0x8B
#define I2C_GPS_WP15                              0x93
#define I2C_GPS_WP_NAV_PAR1			   0x9B	//Waypoint navigation parameter 1
		#define I2C_GPS_WP_NAV_PAR1_REACH_LIMIT	0x0F    //lover 4 bit, waypoint reached distance
#define I2C_GPS_GROUND_COURSE			  0x9C  //GPS ground course (uint16_t)
		
typedef struct {
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    wp_reached:1;
  uint8_t    numsats:4;
} STATUS_REGISTER;

typedef struct {
 uint8_t     poshold:1;
 uint8_t     resume:1;
 uint8_t     set_wp:1;
 uint8_t     activate_wp:1;
 uint8_t     wp:4;
} COMMAND_REGISTER;

typedef struct {
 uint8_t     active_wp:4;
 uint8_t     pervious_wp:4;
} WAYPOINT_REGISTER;

typedef struct {
  long      lat;            //degree*10 000 000
  long      lon;            //degree*10 000 000
} GPS_COORDINATES;

typedef struct {
  uint8_t   wp_reach_distance;		//If we are within this distance (in meters) then assumed that the waypoint is reached. Default value = 2m
  uint8_t   reserved;				//reserved for future use
} WP_NAV_PAR1;

typedef struct {
  STATUS_REGISTER       status;            // 0x00  status register
  COMMAND_REGISTER      command;           // 0x01  command register
  uint8_t               res1;              // 0x02  reserved for future use
  uint8_t               res2;              // 0x03  reserved for future use
  uint8_t               res3;              // 0x04  reserved for future use
  uint8_t               res4;              // 0x05  reserved for future use
  WAYPOINT_REGISTER     wp_reg;            // 0x06  active waypoint and pervious waypoint (good for cross track error calculation)
  uint16_t              ground_speed;      // 0x07-0x08 ground speed from gps m/s*100
  int16_t               altitude;          // 0x09-0x0a gps altitude
  uint32_t		time;	           // 0x0b-0x0e UTC Time from GPS

  uint16_t              distance;          // 0x0f-0x10 distance to active coordinates  (calculated)
  int16_t               direction;         // 0x11-0x12 direction to active coordinates (calculated)   
  GPS_COORDINATES       gps_loc;           // 0x13 current location (8 byte)
  GPS_COORDINATES       gps_wp[16];         // 16 waypoints, WP#0 is RTH position
  WP_NAV_PAR1			wp_nav_par1;		//waypoint navigation parameter register 1
  uint16_t				ground_course;		// 0x9c GPS ground cource
} I2C_REGISTERS;



static I2C_REGISTERS   i2c_dataset;

static GPS_COORDINATES _target;                    //internal target location register

static uint8_t         receivedCommands[MAX_SENT_BYTES];
static uint8_t         new_command;                        //new command received (!=0)

//Handler for requesting data
void requestEvent()
{
 //Serial.println("Q");
 if (receivedCommands[0] >= I2C_GPS_GROUND_SPEED) i2c_dataset.status.new_data = 0;        //Accessing gps data, switch new_data_flag;
 //Write data from the requested data register position
 Wire.write((uint8_t *)&i2c_dataset+receivedCommands[0],32);                    //Write up to 32 byte, since master is responsible for reading and sending NACK
 //32 byte limit is in the Wire library, we have to live with it unless writing our own wire library
//Serial.println(receivedCommands[0]);
//Serial.println(i2c_dataset.status.numsats);

}

//Handler for receiving data
void receiveEvent(int bytesReceived)
{
     uint8_t  *ptr;
     for (int a = 0; a < bytesReceived; a++) {
          if (a < MAX_SENT_BYTES) {
               receivedCommands[a] = Wire.read();
          } else {
               Wire.read();  // if we receive more data then allowed just throw it away
          }
     }

    if (receivedCommands[0] == I2C_GPS_COMMAND) { new_command = receivedCommands[1]; return; }  //Just one byte, ignore all others

     if(bytesReceived == 1 && (receivedCommands[0] < REG_MAP_SIZE)) { return; }        //read command from a given register
     if(bytesReceived == 1 && (receivedCommands[0] >= REG_MAP_SIZE)){                  //Addressing over the reg_map fallback to first byte
          receivedCommands[0] = 0x00;
          return;
     }

    //More than 1 byte was received, so there is definitely some data to write into a register
    //Check for writeable registers and discard data is it's not writeable
    
    if ((receivedCommands[0]>=I2C_GPS_WP0) && (receivedCommands[0]<=REG_MAP_SIZE)) {    //Writeable registers above I2C_GPS_WP0
     ptr = (uint8_t *)&i2c_dataset+receivedCommands[0];
     for (int a = 1; a < bytesReceived; a++) { *ptr++ = receivedCommands[a]; }
    }
}



/* this is an equirectangular approximation to calculate distance and bearing between 2 GPS points (lat/long)
   it's much more faster than an exact calculation
   the error is neglectible for few kilometers assuming a constant R for earth
   input: lat1/long1 <-> lat2/long2      unit: 1/1000000 degree
   output: distance in meters, bearing in degrees
*/
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
  float dLat = ((lat2 - lat1));                                    // difference of latitude in 1/100000 degrees
  float dLon = ((lon2 - lon1)) * cos(lat1*(PI/180/100000.0));      // difference of longitude in 1/100000 degrees
  *dist = 6372795 / 100000.0 * PI/180*(sqrt(sq(dLat) + sq(dLon)));
  if (lat1 != lat2)
    *bearing = 180/PI*(atan2(dLon,dLat));
  else
    *bearing = 0;
}

/* The latitude or longitude is coded this way in NMEA frames
  dm.m   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - m can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 100 000
*/
uint32_t GPS_coord_to_degrees(char* s) {
  char *p , *d = s;
  uint32_t sec , m = 1000;
  uint16_t min , dec = 0;
  
  if(!*s) return 0;
  for(p=s; *p!=0; p++) {
    if (d != s) { *p-='0'; dec+=*p*m; m/=10; }
    if (*p == '.') d=p;
  }
  m=10000;
  min = *--d-'0' + (*--d-'0')*10;
  sec = (m*min+dec)/6;
  while (d != s) { m*=10; *--d-='0'; sec+=*d*m; }
  return sec ;
}

/* This is am expandable implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA, GSA and RMC frames to decode on the serial bus
   Using the following data :
   GGA
     - time
     - latitude
     - longitude
     - GPS fix 
     - GPS num sat (5 is enough to be +/- reliable)
     - GPS alt
   GSA
     - 3D fix (it could be left out since we have a 3D fix if we have more than 4 sats  
   RMC
     - GPS speed over ground, it will be handy for wind compensation (future)  
     
*/

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3

bool GPS_newFrame(char c) {

  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, gps_frame = NO_FRAME;

  switch (c) {
    case '$': param = 0; offset = 0; parity = 0; 
              break;
    case ',':
    case '*':  string[offset] = 0;
                if (param == 0) { //frame identification
                  gps_frame = NO_FRAME;  
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
                }
                
                switch (gps_frame)
                {
                  //************* GPGGA FRAME parsing
                  case GPGGA_FRAME: 
                    switch (param)
                     {
                      case 1: i2c_dataset.time = (atof(string)*1000);      //up to .000 s precision not needed really but the data is there anyway
                              break;
                      case 2: i2c_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
                              break;
                      case 3: if (string[0] == 'S') i2c_dataset.gps_loc.lat = -i2c_dataset.gps_loc.lat;
                              break;
                      case 4: i2c_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
                              break;
                      case 5: if (string[0] == 'W') i2c_dataset.gps_loc.lon = -i2c_dataset.gps_loc.lon;
                              break;
                      case 6: i2c_dataset.status.gps2dfix = string[0]  > '0';
                              break;
                      case 7: i2c_dataset.status.numsats = atoi(string);
                              break;
                      case 9: i2c_dataset.altitude = atoi(string);
                              break;
                     }
                   break;         
                   //************* GPGSA FRAME parsing
                   case GPGSA_FRAME:
                     switch (param)
                     {
                      case 2: i2c_dataset.status.gps3dfix = string[0] == '3';
                      break;
                     }
                   break;
                   //************* GPGSA FRAME parsing
                   case GPRMC_FRAME:
                     switch(param)
                     {
                       case 7: i2c_dataset.ground_speed = (atof(string)*0.5144444)*10;      //convert to m/s*100
                               break; 
	               case 8: i2c_dataset.ground_course = (atof(string)*10);				//Convert to degrees *10 (.1 precision)
							   break;
                     }
                   
                   break;                   
                }
            
                param++; offset = 0;
                if (c == '*') checksum_param=1;
                else parity ^= c;
                break;
     case '\r':
     case '\n':  
                if (checksum_param) { //parity checksum
                  uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
                  if (checksum == parity) frameOK = 1;
                }
                checksum_param=0;
                break;
     default:
             if (offset < 15) string[offset++] = c;
             if (!checksum_param) parity ^= c;
             
  }
  return frameOK && (gps_frame == GPGGA_FRAME);
}
void setup() {

  uint8_t i;
  Serial.begin(115200);      //for GPS
  //Serial1.begin(115200);
//Init i2c_dataset;
  uint8_t *ptr = (uint8_t *)&i2c_dataset;
  for (i=0;i<sizeof(i2c_dataset);i++) { *ptr = 0; ptr++;}

  //Set up default parameters
  i2c_dataset.wp_nav_par1.wp_reach_distance = 2;			//If we are within 2 meters, consider the waypoint reached
  
  
  //Start I2C communication routines
  Wire.begin(I2C_ADDRESS);               // DO NOT FORGET TO COMPILE WITH 400KHz!!! else change TWBR Speed to 100khz on Host !!! Address 0x40 write 0x41 read
  Wire.onRequest(requestEvent);          // Set up event handlers
  Wire.onReceive(receiveEvent);
}

void loop() {
  
  static uint8_t GPS_fix_home;
  static uint8_t _command_wp;
  static uint8_t _command;
  
  static uint32_t _watchdog_timer = 0;
  
  //Get gps data and parse
  while (Serial.available()) {
    if (GPS_newFrame(Serial.read())) {
      if (i2c_dataset.status.gps2dfix == 1) {
        //Set current target after fix, this is for safety
        if (GPS_fix_home == 0) {
          GPS_fix_home = 1;
          _target.lat = i2c_dataset.gps_loc.lat;
          _target.lon = i2c_dataset.gps_loc.lon;
        }

        //Get distance and direction to _target location
        GPS_distance(_target.lat,_target.lon,i2c_dataset.gps_loc.lat,i2c_dataset.gps_loc.lon, &i2c_dataset.distance, &i2c_dataset.direction);
        i2c_dataset.status.new_data = 1;
        _watchdog_timer = millis();      //reset watchdog time at each valid gps packet
	if (i2c_dataset.distance <= i2c_dataset.wp_nav_par1.wp_reach_distance) {  i2c_dataset.status.wp_reached = 0; } //Set Waypoint reached flag
      }
    }
  } //While
  
  
//check watchdog timer, after 1200ms without valid packet, assume that gps communication is lost.
if (_watchdog_timer != 0)
{  
  if (_watchdog_timer+1200 < millis()) 
     {
       i2c_dataset.status.gps2dfix = 0;
       i2c_dataset.status.numsats = 0;
       i2c_dataset.gps_loc.lat = 0;
       i2c_dataset.gps_loc.lon = 0;
       i2c_dataset.distance = 0;
       i2c_dataset.direction = 0;
       _watchdog_timer = 0;
       i2c_dataset.status.new_data = 1;
     }
}

  //Check for new incoming command on I2C
  if (new_command!=0) {
    _command = new_command;                                                   //save command byte for processing
    new_command = 0;                                                          //clear it

    _command_wp = (_command & 0xF0) >> 4;                                     //mask 4 MSB bits and shift down
    _command = _command & 0x0F;                                               //empty 4MSB bits

   switch (_command) {
     case I2C_GPS_COMMAND_POSHOLD:
          _target.lon = i2c_dataset.gps_loc.lon;
          _target.lat = i2c_dataset.gps_loc.lat;
          i2c_dataset.status.new_data = 0;                                    //invalidate current dataset
          break;
     case I2C_GPS_COMMAND_RESUME:
          //Resume last active waypoint
          _target.lon = i2c_dataset.gps_wp[i2c_dataset.wp_reg.active_wp].lon;
          _target.lat = i2c_dataset.gps_wp[i2c_dataset.wp_reg.active_wp].lat;
          i2c_dataset.status.new_data = 0;                                    //invalidate current dataset
          break;
     case I2C_GPS_COMMAND_SET_WP:
          i2c_dataset.gps_wp[_command_wp].lat = i2c_dataset.gps_loc.lat;                  //set the selected WP to the current gps loc
          i2c_dataset.gps_wp[_command_wp].lon = i2c_dataset.gps_loc.lon;
          break;          
     case I2C_GPS_COMMAND_ACTIVATE_WP:
          i2c_dataset.wp_reg.pervious_wp = i2c_dataset.wp_reg.active_wp;      //Store current wp as pervious wp;
          i2c_dataset.wp_reg.active_wp   = _command_wp;                       //Set active wp to the one from command reg.
          _target.lon = i2c_dataset.gps_wp[_command_wp].lon;
          _target.lat = i2c_dataset.gps_wp[_command_wp].lat;
		  i2c_dataset.status.wp_reached = 0;								  //zero out wp_reached flag
          i2c_dataset.status.new_data = 0;                                    //invalidate current dataset
          break;
   } //ignore invalid command bytes (only one command bit could be set
  }
}

