/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Warning! It seems that the display needs a lot of free memory. If less than 1370 Bytes for variables
  are available you will see garbage in the lower right corner of the display. It looks like overwriting
  the disply buffer memory.
*/

// Version 1.4, 27.07.2020, AK-Homberger
// Version 1.5, 14.11.2020, Thomas Hoffmann, Initial
// Version 1.6, 04.04.2021, Thomas Hoffmann, Read data from Autohelm ST30 Bidata
// Version 1.7, 05.04.2021, Thomas Hoffmann, Read data from Autohelm ST30 Bidata and ST1000+

//#define DEBUG
#define CODES 2  // (set to Thomas_1: 1, Thomas_2: 2 or Harald: 3)

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print (x)
#define DEBUG_PRINTDEC(x) Serial.print (x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print (x, HEX)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLN(x)
#endif

#include <avr/pgmspace.h>
#include <RCSwitch.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define AUTO_STANDBY_SUPPORT 1  // Set this to 1 to support Standby and Auto for Key 5 and 6

#define KEY_DELAY 300      // 300 ms break between keys
#define BEEP_DURATION 150  // 150 ms beep time
#define NMEA_BUF_SIZE 50   // NMEA0183 string buffer size

RCSwitch mySwitch = RCSwitch();

#if CODES == 1
const unsigned long Key_Minus_1 PROGMEM = 1029860; // Change values to individual values programmed to remote control
const unsigned long Key_Plus_1 PROGMEM = 1029861;
const unsigned long Key_Minus_10 PROGMEM = 1029862;
const unsigned long Key_Plus_10 PROGMEM = 1029863;
#if AUTO_STANDBY_SUPPORT == 1
const unsigned long Key_Track PROGMEM = 1029864;
const unsigned long Key_Wind PROGMEM = 1029865;
const unsigned long Key_Auto PROGMEM = 1029866;
const unsigned long Key_Standby PROGMEM = 1029867;
#endif
#elif CODES == 2
const unsigned long Key_Minus_1 PROGMEM = 1029870; // Change values to individual values programmed to remote control
const unsigned long Key_Plus_1 PROGMEM = 1029871;
const unsigned long Key_Minus_10 PROGMEM = 1029872;
const unsigned long Key_Plus_10 PROGMEM = 1029873;
#if AUTO_STANDBY_SUPPORT == 1
const unsigned long Key_Track PROGMEM = 1029874;
const unsigned long Key_Wind PROGMEM = 1029875;
const unsigned long Key_Auto PROGMEM = 1029876;
const unsigned long Key_Standby PROGMEM = 1029877;
#endif
#elif CODES == 3
const unsigned long Key_Minus_1 PROGMEM = 1029880; // Change values to individual values programmed to remote control
const unsigned long Key_Plus_1 PROGMEM = 1029881;
const unsigned long Key_Minus_10 PROGMEM = 1029882;
const unsigned long Key_Plus_10 PROGMEM = 1029883;
#if AUTO_STANDBY_SUPPORT == 1
const unsigned long Key_Track PROGMEM = 6941089;
const unsigned long Key_Wind PROGMEM = 6941090;
const unsigned long Key_Auto PROGMEM = 6941092;
const unsigned long Key_Standby PROGMEM = 6941096;
#endif
#else
#error "Invalid CODES value!"
#endif

// Seatalk datagrams

const PROGMEM uint16_t ST_NMEA_BridgeID[] =  { 0x190, 0x00, 0xA3 };

const PROGMEM uint16_t ST_Minus_1[] =  { 0x186, 0x11, 0x05, 0xFA };
const PROGMEM uint16_t ST_Minus_10[] = { 0x186, 0x11, 0x06, 0xF9 };
const PROGMEM uint16_t ST_Plus_1[] =   { 0x186, 0x11, 0x07, 0xF8 };
const PROGMEM uint16_t ST_Plus_10[] =  { 0x186, 0x11, 0x08, 0xF7 };
#if AUTO_STANDBY_SUPPORT == 1
const PROGMEM uint16_t ST_Track[] =    { 0x186, 0x11, 0x28, 0xD7 };
const PROGMEM uint16_t ST_Wind[] =     { 0x186, 0x11, 0x23, 0xDC };
const PROGMEM uint16_t ST_Auto[] =     { 0x186, 0x11, 0x01, 0xFE };
const PROGMEM uint16_t ST_Standby[] =  { 0x186, 0x11, 0x02, 0xFD };
#endif

const PROGMEM uint16_t ST_BeepOn[] =  { 0x1A8, 0x53, 0x80, 0x00, 0x00, 0xD3 };
const PROGMEM uint16_t ST_BeepOff[] = { 0x1A8, 0x43, 0x80, 0x00, 0x00, 0xC3 };

boolean blink = true;
unsigned long ST30_timer = 0;   // timer for AWS display
unsigned long beep_timer2 = 0;  // timer to stop alarm sound
unsigned long bridge_timer = 0; // timer to send ST Bridge ID every 10 seconds

unsigned long key_time = 0;     // time of last key detected
unsigned long beep_time = 0;    // timer for beep duration
bool beep_status = false;

// Autopilot state
byte apState = 0;
#define AP_STANDBY_MODE ((apState & 0x02) == 0)
#define AP_AUTO_MODE ((apState & 0x02) == 2)
#define AP_VANE_MODE ((apState & 0x04) == 4)
#define AP_TRACK_MODE ((apState & 0x08) == 8)

// Relays state
byte relState = 0;
int relMap[] = {18, 15, 14, 16};

void toggleRelay(int rel) { 
  bool rv = false;
  char buf[12] = "";
  if (rel >= 0 && rel < 4) {
    rv = !bitRead(relState, rel);
    bitWrite(relState, rel, rv);
    digitalWrite(relMap[rel], rv ? HIGH : LOW);
    sprintf(buf, "R%d %s", rel + 1, rv ? "On" : "Off");
    Display(buf, 3);
  }
}

void initRelays() {
  relState = 0;
  for (int i = 0; i < 3; i++) {
    pinMode(relMap[i], OUTPUT);
    digitalWrite(relMap[i], LOW);
  }
}

boolean sendDatagram(const uint16_t data[]) {
  int i = 0; int j = 0;
  boolean ok = true;
  int bytes;
  unsigned int inbyte;
  unsigned int outbyte;

  bytes = (pgm_read_byte_near(data + 1) & 0x0f) + 3; // Messege length is minimum 3, additional bytes in nibble 4

  while (j < 5 ) { // CDMA/CD 5 tries
    while (Serial1.available ()) {  // Wait for silence on the bus
      inbyte = (Serial1.read());
      delay(3);
    }

    ok = true;
    for (i = 0; (i < bytes) & (ok == true); i++) { // Write and listen to detect collisions
      outbyte = pgm_read_word_near(data + i);
      Serial1.write(outbyte);
      delay(3);

      if (Serial1.available ()) {
        inbyte = Serial1.read();  // Not what we sent, collision!

        if (inbyte != outbyte) ok = false;
      }
      else ok = false; // Nothing received
    }

    if ( ok )return ok;

    j++; // Collision detected
    DEBUG_PRINTLN("Collision");
    // Display("Collision", 2);
    delay(random(2, 50));  // Random wait for next try
  }
  Display("Send Error", 2);
  return false;
}


void Display(const char *string, int size) {
  DEBUG_PRINTLN(string);
  display.clearDisplay();
  display.setTextSize(size);
  display.setCursor(0, 0);
  display.println(string);
  display.display();
  ST30_timer = millis();
}


// Receive apparent wind speed from bus
int checkWind(char * AWS) {
  unsigned int xx;
  unsigned int y;
  unsigned int inbyte;
  int wind = -1;

  if (Serial1.available ()) {
    inbyte = Serial1.read();
    if (inbyte == 0x111) {    // AWS Seatalk command - See reference from Thomas Knauf
      delay(3);
      inbyte = Serial1.read();
      if (inbyte == 0x01) {   // AWS Setalk command
        delay(3);
        xx = Serial1.read();
        delay(3);
        y = Serial1.read();
        wind = (xx & 0x7f) + (y / 10);  // Wind speed
        if (wind < 100) itoa (wind , AWS, 10);  // Greater 100 must be a receive error
      }
    }
  }
  return wind;
}

/*****************************************************************************
  00  02  YZ  XX XX  Depth below transducer: XXXX/10 feet
                       Flags in Y: Y&8 = 8: Anchor Alarm is active
                                  Y&4 = 4: Metric display units or
                                           Fathom display units if followed by command 65
                                  Y&2 = 2: Used, unknown meaning
                      Flags in Z: Z&4 = 4: Transducer defective
                                  Z&2 = 2: Deep Alarm is active
                                  Z&1 = 1: Shallow Depth Alarm is active
                    Corresponding NMEA sentences: DPT, DBT
*****************************************************************************/
bool parseDepth(char * AWS, char * buf)
{
  double value = -1;
  unsigned int inbyte;
  unsigned int yz;
  unsigned int xxl;
  unsigned int xxh;

  inbyte = Serial1.read();
  delay(3);
  if (inbyte & 0xF != 2 || inbyte >= 0x100) {
    return false;
  }
  yz = Serial1.read();
  delay(3);
  if (yz & 0x4 == 0x4 || yz >= 0x100) {
    return false;
  }
  xxl = Serial1.read();
  delay(3);
  if (xxl >= 0x100) {
    return false;
  }
  xxh = Serial1.read();
  delay(3);
  if (xxh >= 0x100) {
    return false;
  }
  value = (xxl + (xxh << 0x100)) / 10.0;
  if (value > 200.0) {
    return false;              // Depth out of range
  }
  char depthFh[6] = "";
  char depthFt[6] = "";
  char depthM[6] = "";
  dtostrf(value / 6.0, 3, 1, depthFh);
  dtostrf(value, 3, 1, depthFt);
  dtostrf(value * 0.3048, 3, 1, depthM);
  sprintf(buf, "IIDBT,%s,f,%s,M,%s,F", depthFt, depthM, depthFh);
  sprintf(AWS, "%s m", depthM);
  return true;
}

/*****************************************************************************
  20  01  XX  XX  Speed through water: XXXX/10 Knots
                 Corresponding NMEA sentence: VHW
 *****************************************************************************/
bool parseSTW(char * AWS, char *buf) {
  double value = -1;
  unsigned int inbyte;
  unsigned int xxl;
  unsigned int xxh;

  inbyte = Serial1.read();
  delay(3);
  if (inbyte & 0xF != 1 || inbyte >= 0x100) {
    return false;
  }
  xxl = Serial1.read();
  delay(3);
  if (xxl >= 0x100) {
    return false;
  }
  xxh = Serial1.read();
  delay(3);
  if (xxh >= 0x100) {
    return false;
  }
  value = (xxl + (xxh << 0x100)) / 10.0;
  if (value > 50.0) {
    return false;              // Speed out of range
  }
  char speedKn[5] = "";       
  char speedKmh[5] = "";       // only for values < 100.0 !!!!
  dtostrf(value, 3, 1, speedKn);
  dtostrf(value * 1.852, 3, 1, speedKmh);
  sprintf(buf, "IIVHW,0.0,T,0.0,M,%s,N,%s,K", speedKmh, speedKn);
  sprintf(AWS, "%s kn", speedKn);
  return true;
}

/*****************************************************************************
  84  U6  VW  XY 0Z 0M RR SS TT  Compass heading  Autopilot course and
                  Rudder position (see also command 9C)
                  Compass heading in degrees:
                    The two lower  bits of  U * 90 +
                    the six lower  bits of VW *  2 +
                    number of bits set in the two higher bits of U =
                    (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
                  Turning direction:
                    Most significant bit of U = 1: Increasing heading, Ship turns right
                    Most significant bit of U = 0: Decreasing heading, Ship turns left
                  Autopilot course in degrees:
                    The two higher bits of  V * 90 + XY / 2
                  Z & 0x2 = 0 : Autopilot in Standby-Mode
                  Z & 0x2 = 2 : Autopilot in Auto-Mode
                  Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular "10" datagrams
                  Z & 0x8 = 8 : Autopilot in Track Mode
                  M: Alarms + audible beeps
                    M & 0x04 = 4 : Off course
                    M & 0x08 = 8 : Wind Shift
                  Rudder position: RR degrees (positive values steer right,
                    negative values steer left. Example: 0xFE = 2° left)
                  SS & 0x01 : when set, turns off heading display on 600R control.
                  SS & 0x02 : always on with 400G
                  SS & 0x08 : displays “NO DATA” on 600R
                  SS & 0x10 : displays “LARGE XTE” on 600R
                  SS & 0x80 : Displays “Auto Rel” on 600R
                  TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer
 *****************************************************************************/
bool parseChApcRp(char * AWS, char *buf) {
  double value = -1;
  unsigned int inbyte;
  unsigned int u;
  unsigned int vw;
  unsigned int xy;
  unsigned int z;
  unsigned int m;
  unsigned int rr;
  unsigned int ss;
  unsigned int tt;

  inbyte = Serial1.read();
  delay(3);
  if (inbyte & 0xF != 6 || inbyte >= 0x100) {
    return false;
  }
  u = inbyte >> 4;
  vw = Serial1.read();
  delay(3);
  if (vw >= 0x100) {
    return false;
  }
  xy = Serial1.read();
  delay(3);
  if (xy >= 0x100) {
    return false;
  }
  z = Serial1.read();
  delay(3);
  if (z >= 0x100 || z & 0xF0 != 0) {
    return false;
  }
  m = Serial1.read();
  delay(3);
  if (m >= 0x100 || m & 0xF0 != 0) {
    return false;
  }
  rr = Serial1.read();
  delay(3);
  if (rr >= 0x100) {
    return false;
  }
  ss = Serial1.read();
  delay(3);
  if (ss >= 0x100) {
    return false;
  }
  tt = Serial1.read();
  delay(3);
  if (tt >= 0x100) {
    return false;
  }
  int ch = (u & 0x3) * 90 + (vw & 0x3F) * 2 + (u & 0xC ? (u & 0xC == 0xC ? 2 : 1) : 0);
  boolean turnsRight = u >> 3;
  int apc = (vw >> 6) * 90 + (xy >> 1);
  apState = z;
  bool apAlarmCourse = m & 0x04 == 4;
  bool apAlarmWind = m & 0x08 == 8;
  int8_t rp = rr;
  sprintf(buf, "IIHDT,%d.0,T", apc);
  sprintf(AWS, "%d deg ap", apc);
  return true;
}

/*****************************************************************************
  9C  U1  VW  RR    Compass heading and Rudder position (see also command 84)
                     Compass heading in degrees:
                       The two lower  bits of  U * 90 +
                       the six lower  bits of VW *  2 +
                       number of bits set in the two higher bits of U =
                       (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
                     Turning direction:
                       Most significant bit of U = 1: Increasing heading, Ship turns right
                       Most significant bit of U = 0: Decreasing heading, Ship turns left
                     Rudder position: RR degrees (positive values steer right,
                       negative values steer left. Example: 0xFE = 2° left)
                     The rudder angle bar on the ST600R uses this record
 *****************************************************************************/
bool parseChRp(char * AWS, char *buf) {
  double value = -1;
  unsigned int inbyte;
  unsigned int u;
  unsigned int vw;
  unsigned int rr;

  inbyte = Serial1.read();
  delay(3);
  if (inbyte & 0xF != 1 || inbyte >= 0x100) {
    return false;
  }
  u = inbyte >> 4;
  vw = Serial1.read();
  delay(3);
  if (vw >= 0x100) {
    return false;
  }
  rr = Serial1.read();
  delay(3);
  if (rr >= 0x100) {
    return false;
  }
  int ch = (u & 0x3) * 90 + (vw & 0x3F) * 2 + (u & 0xC ? (u & 0xC == 0xC ? 2 : 1) : 0);
  boolean turnsRight = u >> 3;
  int8_t rp = rr;
  sprintf(buf, "IIHDM,%d.0,M", ch);
  sprintf(AWS, "%d deg r", rp);
  return true;
}


void printWithChecksum(char * buf) {
  int chksum = 0;
  int count;
  for (char *c = buf, count = 0; *c != '\0' && count < NMEA_BUF_SIZE; c++, count++)
  {
    chksum ^= *c;
  }
  char chksumStr[5];
  sprintf(chksumStr, "%02x\r\n", chksum);
  Serial.print('$');
  Serial.print(buf);
  Serial.print('*');
  Serial.println(chksumStr);
}

// Receive Autohelm ST30 Bidata from bus
int checkST30(char * AWS) {
  unsigned int ttype = 0;
  unsigned int inbyte;
  char buf[NMEA_BUF_SIZE] = "<empty>";
  int rc = -1;
  boolean isParsed = false;

  if (Serial1.available ()) {
    ttype = Serial1.read();
    if (ttype & 0x100 != 0x100) {
      return -1;
    }
    delay(3);
    switch (ttype) {
      case 0x100:                             // Depth below transducer
        isParsed = parseDepth(AWS, buf);
        break;
      case 0x120:                             // Speed through water
//        isParsed = parseSTW(AWS, buf);
        break;
      case 0x184:                             // Compass heading Autopilot course and Rudder position
        isParsed = parseChApcRp(AWS, buf);
        break;
      case 0x19C:                             // Compass heading and Rudder position
//        isParsed = parseChRp(AWS, buf);
        break;
      default:
        break;
    }
    if (isParsed) {
      printWithChecksum(buf);
      rc = 0;
    } else {
    }
  }
  return rc;
}


// Dump Seatalk1 bus data
int dumpSeatalk1(char * AWS) {
  unsigned int i;
  unsigned int inbyte;
  unsigned int count;
  int wind = -1;
  char *ptr;

  if (Serial1.available ()) {
    inbyte = Serial1.read();
    if (inbyte >= 0x100 && inbyte < 0x200) {    // AWS Seatalk command - See reference from Thomas Knauf
      DEBUG_PRINTHEX(inbyte);
      DEBUG_PRINT(' ');
      delay(3);
      inbyte = Serial1.read();
      if (inbyte >= 0x100) {
        DEBUG_PRINTLN("?");
        return -1;                            // read error, stop
      }
      DEBUG_PRINTHEX(inbyte);
      DEBUG_PRINT(' ');
      count = (inbyte & 0xF) + 1;
      wind = 0;
      ptr = AWS;
      for (i = 0; i < count; i++) {
        delay(3);
        inbyte = Serial1.read();
        if (inbyte >= 0x100) {
          DEBUG_PRINTLN("?");
          return -1;                            // read error, stop
        }
        DEBUG_PRINTHEX(inbyte);
        DEBUG_PRINT(' ');
        if ((i * 3) < 39) {
          ptr = String(inbyte, HEX).c_str();
          ptr += 2;
          *ptr++ = ' ';
        } else {
          wind = -1;
          break;
        }
      }
      AWS[i * 3] = '\0';
      DEBUG_PRINTLN("!");
    }
  }
  return wind;
}


// Beep on if key received
void BeepOn(void) {

  if (beep_status == true) return;  // Already On

  sendDatagram(ST_BeepOn);
  digitalWrite(20, HIGH);
  beep_time = millis();
  beep_status = true;
}


// Beep off after BEEP_TIME
void BeepOff(void) {

  if (beep_status == true && millis() > beep_time + BEEP_DURATION) {
    sendDatagram(ST_BeepOff);
    digitalWrite(20, LOW);
    beep_status = false;
  }
}


void setup() {

  Serial.begin( 9600 );  // Serial out put for function checks with PC
  Serial1.begin( 4800, SERIAL_9N1 );  // Set the Seatalk modus - 9 bit
  Serial1.setTimeout(5);

  mySwitch.enableReceive(4);  // RF Receiver on inerrupt 4 => that is pin 7 on Micro

  pinMode(9, OUTPUT);         // LED to show if keys are received
  digitalWrite(9, HIGH);

  pinMode(20, OUTPUT);         // Buzzer to show if keys are received
  digitalWrite(20, LOW);

  initRelays();                // Reset relays
  apState = 0;                  // autpilot is in Standby-Mode

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64 from Conrad else 3D)
  display.setTextColor(WHITE);
  Display("Start", 4);

  sendDatagram(ST_NMEA_BridgeID);   // Send NMEA Seatakl BridgeID to make Seatalk to Seatalk NG converter happy
}


void loop() {

  char AWS[40] = "";
  unsigned long value = 0;

  if (millis() > ST30_timer + 2000 ) {
    Display("---", 7);              // Show --- after about two seconds when no wind data is received
    ST30_timer = millis();
  }

  if (millis() > beep_timer2 + 3000 ) {
    sendDatagram(ST_BeepOff);       // Additional Beep off after three seconds to avoid constant alarm
    beep_timer2 = millis();
  }

  if (millis() > bridge_timer + 10000 ) {
    sendDatagram(ST_NMEA_BridgeID); // Send NMEA Seatakl BridgeID every 10 seconds to make Seatalk to Seatalk NG converter happy
    bridge_timer = millis();
  }

#ifdef DEBUG
  if (dumpSeatalk1(AWS) > -1) {
#else
  if (checkST30(AWS) > -1) {
#endif
    Display(AWS, 3);
    ST30_timer = millis();
  }

  if (mySwitch.available()) {
    value = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
  }

  if (value > 0 && millis() > key_time + KEY_DELAY) {

    key_time = millis();      // Remember time of last key received
    digitalWrite(9, blink);   // LED on/off
    blink = !blink;           // Toggle LED to show received key

    switch (value) {
      case Key_Minus_1:
        if (AP_STANDBY_MODE) {
          toggleRelay(0);
        } else {
          Display("-1", 7);
          sendDatagram(ST_Minus_1);
          BeepOn();
        }
        break;
      case Key_Plus_1:
        if (AP_STANDBY_MODE) {
          toggleRelay(1);
        } else {
          Display("+1", 7);
          sendDatagram(ST_Plus_1);
          BeepOn();
        }
        break;
      case Key_Minus_10:
        if (AP_STANDBY_MODE) {
          toggleRelay(2);
        } else {
          Display("-10", 7);
          sendDatagram(ST_Minus_10);
          BeepOn();
        }
        break;
      case Key_Plus_10:
        if (AP_STANDBY_MODE) {
          toggleRelay(3);
        } else {
          Display("+10", 7);
          sendDatagram(ST_Plus_10);
          BeepOn();
        }
        break;
#if AUTO_STANDBY_SUPPORT == 1
      case Key_Track:
        Display("Track", 4);
        sendDatagram(ST_Track);
        BeepOn();
        break;
      case Key_Wind:
        Display("Wind", 4);
        sendDatagram(ST_Wind);
        BeepOn();
        break;
      case Key_Auto:
        Display("Auto", 4);
        sendDatagram(ST_Auto);
        BeepOn();
        break;
      case Key_Standby:
        Display("Standby", 4);
        sendDatagram(ST_Standby);
        BeepOn();
        break;
#endif
    }
  }
  BeepOff();
}
