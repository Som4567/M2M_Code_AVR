#include <TinyGPS.h>

#include <SoftwareSerial.h>

#include <AltSoftSerial.h>

#include <PString.h>

#include <avr/io.h>

#include <avr/wdt.h>

#include <Wire.h>

#define DEVICE(0x53) //ADXL345 device address
#define TO_READ(6) //num of bytes we are going to read each time (two bytes for each axis)
#define DATAX0 0x32 //X-Axis Data 0
byte buff[TO_READ]; //5 bytes buffer for saving data read from the device
//string buffer to transform data before sending it to the serial port
//-----------------------------------------Device id-----------------------
#include <MemoryFree.h>

#define deviceid 101
boolean debug = false;
//on every start it ask for sleep first. make the below to false if you do not want to ask device for sleep in beginning only.
boolean just_started = true;
int axis[3];
//to switch on the green led of pin 13.
int led = 13;
unsigned long start_watchdog = millis();

char sleep[7];
char freq[7];
unsigned long get_sleep = 0;
unsigned long get_freq = 7;
//initialize object for GSM and GPS
SoftwareSerial ss(6, 5);
SoftwareSerial ssDummy(10, 11);
AltSoftSerial mySerial;
TinyGPS gps;
char buffer[140];

PString req(buffer, sizeof(buffer));
//array to hold response
char resp[100];
char resp_first[50];
//var to hold latitude and longitude
float flat = 0, flon = 0;
boolean conEnd = true;
//var to check if network signal is there or not
boolean noSignal = true;
//var to check if IP exist for the previous context, var notDone is used to simply check if processing done for any loop
boolean noIp = true, isIpClosed = false, notDone = true;
// to check the current length of resp array declared
uint8_t curLen = 0;
//default function to initialize all the required
//int x=0;
//using this var we can chk for update in every 30 minutes if car is not moving.
boolean update_once = false;
void sleep_all() {
  wdt_disable();
  //close socket and connection .. put GSM in sleep and GPS in sleep and conEnd=true;
  //close socket
  //  mySerial.println(F("+++"));
  //                  mySerial.flush();
  //delay(3000);
  //smart_delay(5000,false);
  delay(30000);
  wdt_enable(WDTO_8S);
  mySerial.println(F("AT#SH=1"));
  mySerial.flush();
  smart_delay(2000, false);
  //close connection
  mySerial.println(F("AT#SGACT=1,0"));
  mySerial.flush();
  smart_delay(2000, false);
  //mySerial.println(F("AT+CFUN=0"));
  //mySerial.flush();
  //smart_delay(3000,false);
  wdt_reset();
  start_watchdog = millis();
  while (mySerial.available()) {
    mySerial.read();
    watchdog_reset();

  }
  wdt_reset();
  ss.listen();
  ss.println(F("$PMTK161,0*28"));
  ss.flush();
  smart_delay(3000, false);
  ssDummy.listen();
  conEnd = true;
  wdt_reset();
}
void wakeup_all() {
  //wake up gsm and GPS
  //mySerial.println(F("AT+CFUN=1"));
  //mySerial.flush();
  //smart_delay(3000,false);
  mySerial.println(F("AT#SGACT=1,1"));
  mySerial.flush();
  smart_delay(2000, false);

  wdt_reset();
  ss.listen();
  ss.println(F("$PMTK251,0*28"));
  ss.flush();
  ssDummy.listen();

  wdt_reset();
}
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array

void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address); //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num); // request 5 bytes from device
  int i = 0;
  while (Wire.available()) //device may send less than requested (abnormal)
  {
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

int a_xx = 0;
int a_yy = 0;
int a_zz = 0;

int a_x = 0;
int a_y = 0;
int a_z = 0;
unsigned long ismoving_start = 0;
void ismoving(boolean ismoving_flag) {
  wdt_reset();
  digitalWrite(led, LOW);
  //uint8_t ismoving_ctor=8;
  uint8_t ismoving_range = 3;
  ismoving_start = millis();
  unsigned int flag = 1;
  if (update_once == true || ismoving_flag == true) {
    //ismoving_ctor=10;
    ismoving_range = 4;
    flag = 30;
  }
  while (flag) {
    if (debug) {
      Serial.println(F("hi in ismoving"));
    }
    uint8_t i = 0;

    a_x = 0;
    a_y = 0;
    a_z = 0;

    //wdt_enable(WDTO_8S);
    wdt_reset();
    while (i < 8) {

      readFrom(DEVICE, DATAX0, sizeof(axis), (byte * ) axis); //read the acceleration data from the ADXL345
      a_x += axis[0];
      a_y += axis[1];
      a_z += axis[2];
      delay(100);
      if (flag >= 30)
        delay(400);
      i++;

    }
    //  wdt_disable();
    wdt_reset();
    a_x = a_x / 8;
    a_y = a_y / 8;
    a_z = a_z / 8;
    i = 0;

    a_xx = 0;
    a_yy = 0;
    a_zz = 0;
    // wdt_enable(WDTO_8S);
    wdt_reset();
    while (i < 8) {

      readFrom(DEVICE, DATAX0, sizeof(axis), (byte * ) axis); //read the acceleration data from the ADXL345
      a_xx += axis[0];
      a_yy += axis[1];
      a_zz += axis[2];

      delay(100);
      i++;
      if (flag >= 30)
        delay(400);
    }
    //wdt_disable();
    wdt_reset();
    axis[0] = a_xx / 8;
    axis[1] = a_yy / 8;
    axis[2] = a_zz / 8;

    if ((a_x - axis[0]) > ismoving_range || (axis[0] - a_x) > ismoving_range || (a_y - axis[1]) > ismoving_range || (axis[1] - a_y) > ismoving_range || (a_z - axis[2]) > ismoving_range || (axis[2] - a_z) > ismoving_range) //|| (conEnd==true && mySerial.available())){
    {

      digitalWrite(led, HIGH);
      if (debug) {
        Serial.println(F("Hi moving started so going out of moving"));
      }
      if (conEnd == true) {
        wakeup_all();

      }
      flag = 0;
      update_once = false;
    } else {
      if (((conEnd == false || ismoving_flag == true) && ((unsigned long)(millis() - ismoving_start) > 300000)) || update_once) {
        sleep_all();
        update_once = false;
        ismoving_flag = false;
      }
      if (++flag % 70 == 0) {
        smart_delay(5000, false);
        //after almost 5 minutes low down the sensitivity to avoid false wakeup.
        //Serial.println("after almost 5 minutes low down the sensitivity to avoid false wakeup.");

        ismoving_range = 4;
      } else if (flag > 30 && flag % 20 == 0) {
        wdt_reset();
        delay(5000);
        wdt_reset();
      }
      if (flag >= 65000) flag = 30;
      if (((unsigned long)(millis() - ismoving_start) > 1800000)) {
        if (debug) {
          Serial.println(F("just update once and go to sleep "));
        }
        wakeup_all();
        flag = 0;
        update_once = true;
      }
    }
  }

  wdt_reset();
}

void getGpsdata() {
  unsigned long age;
  if ((gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES) && (gps.hdop() != TinyGPS::GPS_INVALID_HDOP)) {
    gps.f_get_position( & flat, & flon, & age);

  }
  gpsdelay(1000, true);
}
//function designed to put GPS delay.
static void gpsdelay(unsigned int ms, boolean flag) {
  unsigned long start = millis();
  do {
    while (ss.available() > 0) {
      if (flag == true)
        gps.encode(ss.read());
      else
        ss.read();
    }
  } while ((unsigned long)(millis() - start) < ms);

}
void setup() {
  wdt_disable();
  GSM_setup();
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  //Send data 
  send_data();
}
void send_data() {
  //We are taking this counter so that when ever we start sending data to server then we should try couple of times as we found that at starting it requires couple of data size to fill the buffer.
  //uint8_t high_freq_ctor=0;

  uint8_t i = 0;
  //uint8_t ctor=0;
  get_freq = (get_freq * 1000) - 3000;
  uint8_t newcon_ctor = 0;
  uint8_t sending_ctor = 0;
  unsigned long getinterval_start = millis();
  uint8_t k;
  //to calculate the time interval to check ismoving()
  unsigned long ctor = millis();
  while (1) {
    //check movement functionality here 
    //in loop of movement check we also need to check the sleep time call so that if sleep times come then it will go to sleep without doing any movement       uint8_t k=0;
    k = 0;
    wdt_reset();
    if (conEnd) {
      //high_freq_ctor=0;
      k = 0;
      noSignal = true;
      noIp = false, notDone = true;
      while (mySerial.available() > 0) mySerial.read(); //clean input buffer

      do {
        //wdt_enable(WDTO_8S);
        mySerial.println(F("AT+CREG?"));
        //wdt_disable();
        //wdt_reset();
        if (k++ < 15)
          smart_delay(3000, false);
        else
          smart_delay(10000, false);
        if (k % 20 == 0) {
          ismoving(true);
        }

        if ((strstr(resp, "CREG: 0,1") != NULL) || (strstr(resp, "CREG: 0,5") != NULL))
          noSignal = false;
        else if (strstr(resp, "CREG: 0,2") != NULL) {} else {
          mySerial.println(F("AT+COPS=0,0"));
          //  wdt_reset();
          smart_delay(3000, false);
          if (k % 20 == 0) {
            //wdt_enable(WDTO_2S);
            delay(10000);
          }
        }
        if (k > 250) {

          k = 0;
        }

      }
      while (noSignal);
      wdt_reset();
      mySerial.println(F("AT#CGPADDR=1"));
      mySerial.flush();
      //wdt_reset();
      smart_delay(4000, false);
      if (strstr(resp, "CGPADDR: 1,\"\"") != NULL)
        noIp = true;
      while (noIp) {
        k = 0;
        //wdt_enable(WDTO_8S);
        wdt_reset();
        mySerial.println(F("AT#SGACT=1,1"));
        //wdt_disable();
        mySerial.flush();
        //wdt_reset();
        if (k++ < 15)
          smart_delay(4000, false);
        else
          smart_delay(10000, false);
        if (strstr(resp, "OK") != NULL)
          noIp = false;
        else if (strstr(resp, "ERROR") != NULL)
          noIp = false;
      }
      k = 0;
      do {

        wdt_disable();
        mySerial.println(F("AT#SD=1,0,80,\"xyz.com\""));
        mySerial.flush();
        wdt_enable(WDTO_8S);

        if (k++ < 15)
          smart_delay(4000, false);
        else
          smart_delay(8000, false);
        if (strstr(resp, "CONNECT") != NULL)
          notDone = false;
        if (strstr(resp, "OK") != NULL)
          notDone = false;
        else if (strstr(resp, "ERROR") != NULL) {
          k = 0;
          break;
        }
        wdt_reset();
      } while (notDone);

      if ((++newcon_ctor) % 50 == 0) {
        if (debug) {
          Serial.println(F("start the gsm as it's more than 50 times"));
        }
        ismoving(true);
        wdt_reset();
        delay(10000);
      }
      if ((newcon_ctor) % 10 == 0) {
        ismoving(true);
        mySerial.println(F("AT#SH=1"));
        mySerial.flush();
        smart_delay(2000, false);
        //close connection
        mySerial.println(F("AT#SGACT=1,0"));
        mySerial.flush();
        smart_delay(2000, false);

        mySerial.println(F("AT#SGACT=1,1"));
        mySerial.flush();
        smart_delay(2000, false);

      }

      if (k == 0) {
        conEnd = true;
        continue;

      }
      conEnd = false;
    }
    newcon_ctor = 0;
    k = 0;
    //To make sure to get new GPS data always before sending it to server set flat,flon=0 

    wdt_reset();
    if (update_once == false) {
      flat = 0;
      flon = 0;

    }
    //for testing
    /*else
    {

    flat=flat+100; 
    flon=flon-100;
    }*/
    while ((flat == 0) && (flon == 0)) {
      ss.listen();
      //wdt_enable(WDTO_8S);
      getGpsdata();
      gpsdelay(1000, false);
      ssDummy.listen();
      //wdt_disable();
      if (debug) {
        Serial.println(F("Got GPS Data"));
      }
      //Serial.print(F("freeMemory()="));
      //Serial.println(freeMemory());

      k++;
      wdt_reset();
      gpsdelay(1000, false);
      if (k > 10)
        smart_delay(5000, false);
      if (k % 50 == 0) {
        ismoving(true);
      }
      if (k > 250) {
        wdt_reset();
        ss.listen();
        //Restart the GPS
        ss.println("$PMTK103*30");
        ss.flush();
        gpsdelay(1000, false);
        //To get high accuracy
        ss.println(F("$PMTK397,2.0*3F"));
        ss.flush();
        gpsdelay(1000, false);
        ssDummy.listen();
        k = 0;
        wdt_reset();
      }

    }

    //if(millis() - getinterval_start < 10800000)
    //{
    //wdt_enable(WDTO_8S);
    wdt_reset();
    req = "\r\nGET /api.php?func=updateDeviceLoc&params=";
    req.print(deviceid);
    req += ":";
    req.print(flat, 6);
    req += ":";
    req.print(flon, 6);
    req += " HTTP/1.1\r\nHost: xyz.com\r\nConnection:keep-alive\r\n";
    mySerial.println(req);
    mySerial.flush();
    //wdt_disable();    
    //wdt_reset();
    if (get_freq > 58000) {

      smart_delay(18000, false);
    } else {
      smart_delay(get_freq, false);
    }
    if (strstr(resp, "CARRIER") != NULL) {
      conEnd = true;
      /*  mySerial.println(F("AT#SH=1"));
      mySerial.flush();
      smart_delay(2000,false);
      //close connection
      mySerial.println(F("AT#SGACT=1,0"));
      mySerial.flush();
      smart_delay(2000,false);*/
    }
    wdt_reset();
    //high_freq_ctor++;

    if (strstr(resp_first, "200 OK") != NULL) {
      sending_ctor = 0;
    } else {
      if (debug) {
        Serial.println("hi looks like not connected");
      }

      if (++sending_ctor >= 5) {
        mySerial.println(F("AT#SH=1"));
        mySerial.flush();
        smart_delay(2000, false);
        //close connection
        mySerial.println(F("AT#SGACT=1,0"));
        mySerial.flush();
        smart_delay(2000, false);
        mySerial.println(F("AT#SGACT=1,1"));
        mySerial.flush();
        smart_delay(2000, false);

        sending_ctor = 0;
        conEnd = true;
      }

    }
    if (get_freq > 58000 && update_once == false && curLen >= 10 && conEnd == false) {

      sleep_all();
      wdt_disable();
      delay(get_freq - 40000);
      //high_freq_ctor=0;
      wdt_enable(WDTO_8S);
      wakeup_all();
      wdt_reset();
    }

    //}
    //else
    //If system restart or just started then ask for sleep 
    if (just_started == true || ((unsigned long)(millis() - getinterval_start) >= 10800000)) {
      wdt_reset();
      //Serial.println("Hi in else");                 
      get_sleep = -1;
      //wdt_enable(WDTO_8S);
      req = "\r\nGET /api.php?func=getSleepTime&params=";
      req.print(deviceid);
      req += " HTTP/1.1\r\nHost: xyz.com\r\nConnection:keep-alive\r\n";
      mySerial.println(req);
      mySerial.flush();
      //wdt_disable();
      //wdt_reset();                        
      smart_delay(20000, true);
      //Serial.println("Send the sleep req url");    

      if (get_sleep == -1 || (strstr(resp, "CARRIER") != NULL) || curLen < 10)
        conEnd = true;
      else {
        //here req is the string from smartdelay() method
        //set sleep timing and frequency 
        //if sleeptime>0 then send for sleep(make sure that in sleep function it first close 
        if (get_freq >= 5) {
          get_freq = (get_freq * 1000) - 3000;
        } else {
          get_freq = (20 * 1000) - 3000;

        }

        get_sleep = get_sleep * 60 * 1000;
        wdt_reset();
        if (get_sleep > 0) {
          sleep_all();
          if (debug) {
            Serial.println(F("going to sleep"));
            Serial.println(get_sleep);
          }
          // Serial.println(get_freq);
          digitalWrite(led, LOW);
          wdt_disable();
          delay(get_sleep);
          wdt_enable(WDTO_8S);
          wakeup_all();

          digitalWrite(led, HIGH);
          //wdt_disable();
          wdt_reset();

        }
        //reset the start to count the 3 hour again.
        just_started = false;
        getinterval_start = millis();
        //reset 10 minutes counter also otherwise soon after server sleep it will go to ismoving
        ctor = millis();

      }
      wdt_reset();

    }
    wdt_reset();
    //ctor++;
    //Call Accelerometer to check the current status of bus ie moving or not in interval of 25 or in case of update_once in every 30 minutes. 

    if (((unsigned long)(millis() - ctor) > 600000) || (update_once && conEnd == false && curLen >= 10)) {
      ismoving(false);
      //high_freq_ctor=0;
      ctor = millis();
    }
  }
}

void GSM_setup() {
  if (debug) {
    Serial.begin(9600); // start serial for output

    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());

    Serial.println(F("Starting the GSM now"));

  }
  mySerial.begin(9600);

  mySerial.println(F("AT#ENHRST=1,0"));
  mySerial.flush();
  delay(7000);
  wdt_enable(WDTO_8S);
  mySerial.println("AT+CMEE=2");
  smart_delay(3000, false);

  Wire.begin();
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);
  writeTo(DEVICE, 0x2D, 15);
  writeTo(DEVICE, 0x2D, 8);

  ssDummy.begin(9600);
  watchdog_reset();
  ss.begin(9600);
  ss.println(F("$PMTK397,2.0*3F"));
  ss.flush();
  gpsdelay(1000, false);
  ssDummy.listen();
  wdt_reset();
  //set hardware flow control to auto
  mySerial.println(F("AT&K=0"));
  wdt_reset();
  smart_delay(2000, false);
  mySerial.println("AT+CPIN=\"1865\"");
  wdt_reset();
  smart_delay(3000, false);
  mySerial.println(F("AT#SH=1"));
  mySerial.flush();
  smart_delay(2000, false);
  //close connection
  mySerial.println(F("AT#SGACT=1,0"));
  mySerial.flush();
  smart_delay(2000, false);

  //enable auto network reg
  mySerial.println(F("AT+COPS=0,0"));
  wdt_reset();
  smart_delay(3000, false);
  /*for reliance setup
  mySerial.println(F("AT+CGDCONT=1,\"IP\",\"rcomnet\",\"0.0.0.0\",0,0"));
  mySerial.flush();
  smart_delay(3000,false);
  */
  /*for Aircel prepaid APN setup
  mySerial.println(F("AT+CGDCONT=1,\"IP\",\"aircelgprs.pr\",\"0.0.0.0\",0,0"));
  mySerial.flush();
  smart_delay(3000,false);
  */
  /*for BSNL APN setup
  mySerial.println(F("AT+CGDCONT=1,\"IP\",\"bsnlnet\",\"0.0.0.0\",0,0"));
  mySerial.flush();
  smart_delay(3000,false);
  */
  //for vodafone APN setup
  mySerial.println(F("AT+CGDCONT=1,\"IP\",\"www\",\"0.0.0.0\",0,0"));
  mySerial.flush();
  wdt_reset();
  smart_delay(3000, false);
  //for Airtel APN setup
  mySerial.println(F("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\",\"0.0.0.0\",0,0"));
  mySerial.flush();
  wdt_reset();
  smart_delay(3000, false);

  mySerial.println(F("AT#GPRS=1"));
  mySerial.flush();
  wdt_reset();
  smart_delay(3000, false);
  mySerial.println(F("AT#SCFG=1,1,15,90,600,15"));
  mySerial.flush();
  wdt_reset();
  smart_delay(5000, false);
  mySerial.println(F("AT&W"));
  mySerial.flush();
  wdt_reset();
  smart_delay(2000, false);

}
void loop() {}

// function to put delay for the output as well as read the output in between
//it will delay for the specified no. of millis

void watchdog_reset() {

  //watchdog reset
  if ((((unsigned long)(millis() - start_watchdog)) / 7) >= 1) {

    wdt_reset();
    start_watchdog = millis();
  }
}
static void smart_delay(unsigned long ms, boolean specialCheck) {

  watchdog_reset();
  start_watchdog = millis();
  memset(resp, '\0', 100);
  memset(resp_first, '\0', 50);
  //Serial.println("In smart delay");
  uint8_t x = 0;
  uint8_t x_first = 0;
  char c;
  uint8_t z = 0;
  uint8_t y = 0;
  boolean countNow_star = false;
  boolean countNow_hash = false;
  unsigned long start = millis();

  do {
    while (mySerial.available()) {
      c = mySerial.read();
      if (specialCheck) {
        if (countNow_star) countNow_star = (c == '*') ? false : true;
        if (countNow_star) {
          sleep[z++] = c;
        }
        if (!countNow_star && z == 0) countNow_star = (c == '*') ? true : false;
        if (countNow_hash) countNow_hash = (c == '#') ? false : true;
        if (countNow_hash) {
          freq[y++] = c;
        }
        if (!countNow_hash && y == 0) countNow_hash = (c == '#') ? true : false;

      }
      resp[x] = c;
      x++;
      x = x % 100;

      if (x_first < 50) {

        resp_first[x_first] = c;
        x_first++;
        //Serial.print((c));
      }
      if (debug) {
        Serial.print((c));
      }
      watchdog_reset();

    }

    watchdog_reset();
  } while ((unsigned long)(millis() - start) < ms);
  wdt_reset();
  resp[x] = '\0';
  resp[x_first] = '\0';
  curLen = x;
  if (specialCheck) {
    sleep[z] = '\0';
    freq[y] = '\0';
    //get_sleep and get_freq are int variable
    //wdt_enable(WDTO_8S);
    if (sleep != NULL && freq != NULL) {

      get_sleep = atoi(sleep);
      get_freq = atoi(freq);
    } else {
      get_sleep = 0;
      get_freq = 20;
    }
    if (get_sleep > 1440) {
      get_sleep = 1440;
    }
    //wdt_disable();
  }

  wdt_reset();
}
