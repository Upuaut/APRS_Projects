/*
 AVA/ATLAS 70cm/2Mtr RTTY/APRS Tracker
 
 By Anthony Stirk M0UPU / James Coxon M6JCX
 
 Latest code can be found: https://github.com/jamescoxon/APRS_Projects / https://github.com/Upuaut/APRS_Projects
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Thanks to : 
 http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 Big thanks to Dave Akerman!
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <SPI.h>
#include <RFM22.h>
#include <avr/pgmspace.h>
#include "ax25modem.h"
#include "geofence.h"

static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

/* CONFIGURABLE BITS */

#define APRS_TX_INTERVAL  120000  // APRS TX Interval 
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50     // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.450
#define RADIO_POWER  0x04
#define RADIO_REBOOT 20  // Reboot Radio every X telemetry lines
/*
 0x02  5db (3mW)
 0x03  8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */
#define POWERSAVING      // Comment out to turn Power Saving modes off

#define RFM22B_PIN 10
#define RFM22B_SDN A5
#define STATUS_LED 7            // PAVA R7 Boards have an LED on PIN4
#define GPS_ENABLE 5
#define HX1_POWER  6
#define HX1_ENABLE 4
#define HX1_TXD    3

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (25)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

char txstring[80];  
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile boolean lockvariables = 0;

int32_t lat = 514981000, lon = -530000, alt = 0,maxalt = 0,lat_dec = 0, lon_dec =0;
uint8_t hour = 0, minute = 0, second = 0, month = 0, day = 0, lock = 0, sats = 0;
int GPSerror = 0, count = 1, n, navmode = 0, lat_int=0,lon_int=0,errorstatus;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int aprs_status = 0, aprs_attempts = 0, psm_status = 0, radiostatus=0, countreset=0, aprs_permitted=0;
int32_t tslf=0;
uint8_t buf[60]; //GPS receive buffer
char comment[3]={
  ' ', ' ', '\0'};
unsigned long startTime;
int aprs_tx_status = 0;

rfm22 radio1(RFM22B_PIN);

void setup() {
  pinMode(STATUS_LED, OUTPUT);   
  pinMode(HX1_POWER, OUTPUT);   
  pinMode(HX1_ENABLE, OUTPUT);
  pinMode(GPS_ENABLE, OUTPUT); 
  pinMode(RFM22B_SDN, OUTPUT);
  digitalWrite(RFM22B_SDN,LOW);
  digitalWrite(HX1_POWER, LOW);
  digitalWrite(HX1_ENABLE, LOW);
  digitalWrite(GPS_ENABLE, LOW);
  wait(500);
  setupRadio();
  Serial.begin(9600);
  wait(150);
  resetGPS();
  wait(500);
  setupGPS();
  digitalWrite(STATUS_LED,LOW);
  initialise_interrupt();
#ifdef POWERSAVING
  ADCSRA = 0;
#endif

}

void loop() {
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();
  if(lock!=3) // Blink LED to indicate no lock
  {
    digitalWrite(STATUS_LED, HIGH);   
    wait(750);               
    digitalWrite(STATUS_LED, LOW); 
    errorstatus |=(1 << 5);     
  }
  else
  {
    errorstatus &= ~(1 << 5);
  }
  checkDynamicModel();

  if(sats>=4){
    if (aprs_tx_status==0)
    {
      startTime=millis();
      aprs_tx_status=1;
    }
    if(millis() - startTime > APRS_TX_INTERVAL) {
      aprs_permitted=0;
      geofence_location(lat,lon);
      aprs_tx_status=0;
      if( aprs_permitted==1 || (alt<300)) {
        send_APRS();
        aprs_attempts++;
      }
    }
  }
#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))
  {
    setGPS_PowerSaveMode();
    wait(1000);
    pinMode(STATUS_LED, INPUT); 
    psm_status=1;
    errorstatus &= ~(1 << 4);
  }
#endif
  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }

    if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=3) {
      tslf++;
    }
    else
    {
      tslf=0;
      errorstatus &= ~(1 << 0);
      errorstatus &= ~(1 << 1);
    }
    if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
      setupGPS();
      wait(125);
      setGps_MaxPerformanceMode();
      wait(125);
      //    errorstatus=1;
      errorstatus |=(1 << 0);
      psm_status=0;
      errorstatus |=(1 << 4); 
    }
    if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
      errorstatus |=(1 << 0);
      errorstatus |=(1 << 1);
      Serial.flush();
      resetGPS(); // Modify this to turn the GPS off ?
      wait(125);
      setupGPS();
    }
   if((count % RADIO_REBOOT == 0) && countreset!=count){
      digitalWrite(RFM22B_SDN, HIGH);
      wait(500);
      setupRadio();
      wait(500);
      radiostatus=0;
      countreset=count;
    }
  } 
}

//************Other Functions*****************

static int pointinpoly(const int32_t *poly, int points, int32_t x, int32_t y)
{
  int32_t p0, p1, l0, l1;
  int c = 0;

  /* Read the final point */
  p0 = pgm_read_dword(&poly[points * 2 - 2]);
  p1 = pgm_read_dword(&poly[points * 2 - 1]);

  for(; points; points--, poly += 2)
  {
    l0 = p0;
    l1 = p1;
    p0 = pgm_read_dword(&poly[0]);
    p1 = pgm_read_dword(&poly[1]);

    if(y < p1 && y < l1) continue;
    if(y >= p1 && y >= l1) continue;
    if(x < p0 + (l0 - p0) * (y - p1) / (l1 - p1)) continue;

    c = !c;
  }

  return(c);
}


int geofence_location(int32_t lat_poly, int32_t lon_poly)
{
  if(pointinpoly(UKgeofence, 10, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = ' ';
    aprs_permitted=0;
  }
  else if(pointinpoly(Netherlands_geofence, 18, lat_poly, lon_poly) == true)
  {
    comment[0] = 'P';
    comment[1] = 'A';
    aprs_permitted=1;
  }

  else if(pointinpoly(Belgium_geofence, 25, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'N';
    aprs_permitted=0;
  }

  else if(pointinpoly(Luxembourg_geofence, 11, lat_poly, lon_poly) == true)
  {
    comment[0] = 'L';
    comment[1] = 'X';
    aprs_permitted=0;
  }

  else if(pointinpoly(Switzerland_geofence, 22, lat_poly, lon_poly) == true)
  {
    comment[0] = 'H';
    comment[1] = 'B';
    aprs_permitted=1;
  }

  else if(pointinpoly(Spain_geofence, 29, lat_poly, lon_poly) == true)
  {
    comment[0] = 'E';
    comment[1] = 'A';
    aprs_permitted=1;
  }

  else if(pointinpoly(Portugal_geofence, 19, lat_poly, lon_poly) == true)
  {
    comment[0] = 'C';
    comment[1] = 'T';
    aprs_permitted=0;
  }

  else if(pointinpoly(France_geofence, 48, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'F';
    aprs_permitted=1;
  }

  else if(pointinpoly(Germany_geofence, 77, lat_poly, lon_poly) == true)
  {
    comment[0] = 'D';
    comment[1] = 'L';
    aprs_permitted=1;
  }
  else if(pointinpoly(Austria_geofence, 51, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'E';
    aprs_permitted=1;
  }
  else if(pointinpoly(Albania_geofence, 18, lat_poly, lon_poly) == true)
  {
    comment[0] = 'Z';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Belarus_geofence, 29, lat_poly, lon_poly) == true)
  {
    comment[0] = 'E';
    comment[1] = 'U';
    aprs_permitted=1;
  }
  else if(pointinpoly(Bosnia_geofence, 23, lat_poly, lon_poly) == true)
  {
    comment[0] = 'E';
    comment[1] = 'U';
    aprs_permitted=0;
  }
  else if(pointinpoly(Bulgaria_geofence, 20, lat_poly, lon_poly) == true)
  {
    comment[0] = 'L';
    comment[1] = 'Z';
    aprs_permitted=1;
  }
  else if(pointinpoly(Croatia_geofence, 33, lat_poly, lon_poly) == true)
  {
    comment[0] = '9';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Czech_geofence, 48, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'K';
    aprs_permitted=1;
  }
  else if(pointinpoly(Denmark_geofence, 10, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'Z';
    aprs_permitted=1;
  }
  else if(pointinpoly(Finland_geofence, 21, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'H';
    aprs_permitted=0;
  }
  else if(pointinpoly(Greece_geofence, 24, lat_poly, lon_poly) == true)
  {
    comment[0] = 'S';
    comment[1] = 'V';
    aprs_permitted=1;
  }
  else if(pointinpoly(Hungary_geofence, 31, lat_poly, lon_poly) == true)
  {
    comment[0] = 'H';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Italy_geofence, 27, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'I';
    aprs_permitted=0;
  }
  else if(pointinpoly(Macedonia_geofence, 11, lat_poly, lon_poly) == true)
  {
    comment[0] = 'Z';
    comment[1] = '3';
    aprs_permitted=0;
  }
  else if(pointinpoly(Moldova_geofence, 8, lat_poly, lon_poly) == true)
  {
    comment[0] = 'E';
    comment[1] = 'R';
    aprs_permitted=0;
  }
  else if(pointinpoly(Kaliningradskaya_geofence, 6, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'R';
    aprs_permitted=0;
  }
  else if(pointinpoly(Montenegro_geofence, 17, lat_poly, lon_poly) == true)
  {
    comment[0] = '4';
    comment[1] = 'O';
    aprs_permitted=0;
  }
  else if(pointinpoly(Norway_geofence, 20, lat_poly, lon_poly) == true)
  {
    comment[0] = 'L';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Poland_geofence, 47, lat_poly, lon_poly) == true)
  {
    comment[0] = 'S';
    comment[1] = 'P';
    aprs_permitted=1;
  }
  else if(pointinpoly(Romania_geofence, 15, lat_poly, lon_poly) == true)
  {
    comment[0] = 'Y';
    comment[1] = 'O';
    aprs_permitted=0;
  }
  else if(pointinpoly(Serbia_geofence, 25, lat_poly, lon_poly) == true)
  {
    comment[0] = 'Y';
    comment[1] = 'T';
    aprs_permitted=0;
  }
  else if(pointinpoly(Slovakia_geofence, 30, lat_poly, lon_poly) == true)
  {
    comment[0] = 'O';
    comment[1] = 'M';
    aprs_permitted=0;
  }
  else if(pointinpoly(Slovenia_geofence, 26, lat_poly, lon_poly) == true)
  {
    comment[0] = 'S';
    comment[1] = '5';
    aprs_permitted=0;
  }
  else if(pointinpoly(Sweden_geofence, 18, lat_poly, lon_poly) == true)
  {
    comment[0] = 'S';
    comment[1] = 'M';
    aprs_permitted=0;
  }
  else if(pointinpoly(Russia_geofence, 55, lat_poly, lon_poly) == true)
  {
    comment[0] = 'R';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Turkey_geofence, 21, lat_poly, lon_poly) == true)
  {
    comment[0] = 'T';
    comment[1] = 'A';
    aprs_permitted=0;
  }
  else if(pointinpoly(Ukraine_geofence, 27, lat_poly, lon_poly) == true)
  {
    comment[0] = 'U';
    comment[1] = 'T';
    aprs_permitted=1;
  }
  else
  {
    comment[0] = ' ';
    comment[1] = '#';
    aprs_permitted=0;
  }
}
void tx_aprs()
{
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  int32_t aprs_alt = alt * 32808 / 10000;


  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);

  ax25_frame(
  APRS_CALLSIGN, APRS_SSID,
  "APRS", 0,
  //0, 0, 0, 0,
  "WIDE1", 1, "WIDE2",1,
  //"WIDE2", 1,
  "!/%s%sO   /A=%06ld|%s|%s/M0UPU,%d,%i",
  ax25_base91enc(slat, 4, aprs_lat),
  ax25_base91enc(slng, 4, aprs_lon),
  aprs_alt, stlm, comment, count, errorstatus
    );

  seq++;
}

ISR(TIMER2_OVF_vect)
{
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;

  /* Update the PWM output */
  OCR2B = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */
        //PORTA &= ~TXENABLE;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;

        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2A */
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  /* Make sure radio is not enabled */
  //PORTA &= ~TXENABLE;

  /* Enable pins for output (Port A pin 4 and Port D pin 7) */
  //DDRA |= TXENABLE;
  pinMode(HX1_TXD, OUTPUT);
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  //PORTA |= TXENABLE;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

void send_APRS() {
  ax25_init();
  digitalWrite(HX1_POWER, HIGH);
  wait(200);
  digitalWrite(HX1_ENABLE, HIGH);
  wait(250);
  tx_aprs();
  wait(250);
  digitalWrite(HX1_ENABLE, LOW);
  wait(100);
  digitalWrite(HX1_POWER, LOW);

}


void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9                          };
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  wait(1000);
  setGPS_DynamicModel6();
  wait(1000);
}
void wait(unsigned long delaytime) // Arduino delayMicroseconds doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}

void setGPS_DynamicModel3()
{
  int gps_set_sucess=0;
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76                                                                                     };
  while(!gps_set_sucess)
  {
    sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                                                            };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}

bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5                                                                           };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}
void setupRadio(){
  digitalWrite(RFM22B_SDN, LOW);
  wait(500);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, RADIO_POWER);
  radio1.write(0x07, 0x08); 
  radio1.write(0x73,0x03); // Start High
}
void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission
    if(radiostatus==1)
    {
      break;
      // Just do nothing
    }
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
    lockvariables=1;
    sprintf(txstring, "$$$$$AVA,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,errorstatus);
    sprintf(txstring, "%s,%c%c,%i",txstring,comment[0]==' ' ? '-' : comment[0],comment[1]==' ' ? '-' : comment[1],aprs_attempts);
    sprintf(txstring, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
    maxalt=0;
    lockvariables=0;
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=0; // Should be finished
      txj=0;
      count++;
      if(count % RADIO_REBOOT == 0) 
      {
        radiostatus=1;
      }
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);	
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }

  }
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,0x03); // High
  }
  else
  {
    radio1.write(0x73,0x00); // Low
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}
uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84                                                                                     };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }

  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                                                     };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}

void prepare_data() {

  gps_check_lock();
  gps_get_position();
  gps_get_time();
}



void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A                                                                                                                                        };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
      (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/100;
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
      (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/100;


    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
      (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67                                                                                                                                      };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(hour > 23 || minute > 59 || second > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92                                                                                                   }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}
void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91                                                                                       }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}
void checkDynamicModel() {
  if(alt<=1000&&sats>4) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);      
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3);

    }
  }
}
void blink(int bdelay) {
  digitalWrite(STATUS_LED, HIGH);   
  wait(bdelay);               
  digitalWrite(STATUS_LED, LOW); 
  wait(bdelay);   
}














