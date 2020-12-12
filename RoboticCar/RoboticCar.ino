#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
//#include "DFR_Key.h"

#define GPSECHO  false
Adafruit_GPS GPS(&Serial3);                   // define GPS object connected to Serial 3
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that are changed across functions
int STEERANGLE = 90;       // servo initial angle (range is 0:180)
float HEADING = 0;  // heading
int carSpeed = 0;
boolean usingInterrupt = false;
int carSpeedPin = 2;      // pin for DC motor (PWM for motor driver)
float refHeading = 0;
double lat;  // GPS latitude in degree decimal multiplied by 100000
double lon;  // GPS latitude in degree decimal multiplied by 100000
long int latDestination = 33.421164 * 100000;     // reference destination
long int lonDestination = -111.934012 * 100000;   // reference destination
float BEARING = 0;
boolean carTurning = false;
double distance; 

//DFR_Key keypad;
//int key = 0;

void setup() {
  myservo.attach(44);     // servo is connected to pin 44
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //if you want to calibrate using another mode, set it here. OPERATION_MODE_COMPASS for a precise tilt compensated compass (Section 3.3.2 / 3.3.3)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //  byte c_data[22] = {236, 255, 220, 255, 3, 0, 0, 6, 162, 4, 16, 6, 255, 255, 255, 255, 0, 0, 232, 3, 125, 2};    // CALIBRATION DATA
  byte c_data[22] = {239, 255, 220, 255, 222, 255, 107, 1, 141, 2, 185, 2, 0, 0, 254, 255, 1, 0, 232, 3, 52, 3};
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  
/*
  lcd.clear();
  lcd.print("hit SELECT");
  while (key != 1) {

    key = keypad.getKey();
    delay(250);

  }
*/

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate // it's more stable than 10Hz
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
}

SIGNAL(TIMER0_COMPA_vect) { // leave this function unchanged//
  char c = GPS.read();    // this function is for reading chars from GPS module
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) { // This function will be called every 1 second
  sei();        //   set interrupt flag // don't change this
  TCNT4  = 336; //   re-initialize timer4's value
  ReadGPS();    //   read GPS data
}

void ReadGPS() {
  //GPS.satellites;
  //lat = GPS.latitudeDegrees * 100000;
  //lon = GPS.longitudeDegrees * 100000;
  int degree;
  double mint;
  if (GPS.newNMEAreceived())
          GPS.parse(GPS.lastNMEA()); //Parse GPS Sentences
      if (GPS.fix){ //if at least five fixed satellites are found
             lat = GPS.latitude;
             lon = GPS.longitude;
      }
      degree = lat/100;
      mint = lat - (degree*100);
      lat = degree + (mint/60);
      lat = lat * 100000;
      
      degree = lon/100;
      mint = lon - (degree*100);
      lon = degree + (mint/60);  
      lon = lon * -100000;   
      //11156.34

}


void ReadHeading() { // Output: HEADING
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  HEADING = euler.x();
}

void CalculateBearing() {
  // calculate bearing angle based on current and destination locations (GPS coordinates)
  BEARING = atan((lon - lonDestination)/(lat - latDestination));
  BEARING = (BEARING*180)/3.14159;
}

void CalculateSteering() { // Input: HEADING // Output: STEERANGLE
  // calculate steering angle based on heading and bearing

  if (abs(BEARING - HEADING) <= 5 || abs(BEARING-HEADING) >= 355)
    {
      //straight
      carTurning = false;
       STEERANGLE = 90;
    }
    else if (abs(BEARING - HEADING) > 180)
    {
      if (BEARING > HEADING)
      {
        //right
         STEERANGLE = 50;
      }
      else
      {
        //left
         STEERANGLE = 130;
      }
      carTurning = true;
    }
    else
    {
      if(BEARING > HEADING)
      {
        //left
         STEERANGLE = 130;
      }
      else
      {
        //right
         STEERANGLE = 50;
      }
      carTurning = true;
    }

  /*int high = refHeading + 5, low = refHeading - 5;
  if (refHeading == 0) {
    high = 5;
    low = 355;
  }
  if ((HEADING <= high && HEADING >= low && refHeading != 0) || ((HEADING <= high || HEADING >= low) && refHeading == 0))
    STEERANGLE = 90;
  else if (refHeading < 180) {
    if (HEADING <= refHeading + 180 && HEADING > refHeading + 5)
      STEERANGLE = 50;
    if (HEADING >= refHeading + 181 || HEADING < refHeading - 5)
      STEERANGLE = 130;
  }
  else if (refHeading >= 180) {
    if (HEADING >= (refHeading + 181 - 360) && HEADING < refHeading - 5)
      STEERANGLE = 130;
    if (HEADING <= (refHeading + 180 - 360) || HEADING > refHeading + 5)
      STEERANGLE = 50;
  }
  else {
    STEERANGLE = 90;
  }*/
}

void CalculateDistance() {
  // calculate distance to destination based on current and destination coordinates
  distance = sqrt((lonDestination - lon)*(lonDestination - lon) + (latDestination - lat)*(latDestination - lat));
  if(distance <= 4){
    //speed = 0
    carSpeed = 0;
  }
  else if(carTurning == true){
    //speed = medum
    carSpeed = 0;
  } 
  else{
    //speed = fast
    carSpeed = 0;
  }
}

void CalculateSpeed() {
  
}

void Actuate() {
  if (millis() > 400000000) {
    analogWrite(carSpeedPin, 0);
    myservo.write(90);
  } else {
    myservo.write(STEERANGLE);
    analogWrite(carSpeedPin, carSpeed);
  }
}

ISR(TIMER1_OVF_vect) {        // This function will be called every 0.1 second
  sei();                  // set interrupt flag // don't change this
  TCNT1  = 59016;         // reinitialize the timer1's value
  ReadHeading();          // read heading
  CalculateBearing();     // calc bearing
  CalculateSteering();    // calc steering
  CalculateDistance();    // calc distance
  Actuate();              // Actuate
}


void printHeadingOnLCD() {
  //lcd.print(lat);
}

void printLocationOnLCD() {
  //lcd.print("\n");
  lcd.print(distance);
  
  Serial.print(lon);
  Serial.print("\n");
  Serial.print(lat);
  Serial.print("\n");
  Serial.print(lonDestination);
  Serial.print("\n");
  Serial.print(latDestination);
  Serial.print("\n");
  Serial.print(BEARING);
  Serial.print("\n");
}

void printDistanceOnLCD() {

}

void loop() {
  lcd.clear();    // clear the LCD
  // You can print anything on the LCD to debug your program!!!
  printHeadingOnLCD();
  printLocationOnLCD();
  delay(100);
}

// not sure why * 100000
// bearing angle
// refHeading = angle towards dest
// distance = sqrt((x2-x1)^2 + (y2-y1)^2)
// change actuate(), instead of millis = 40s, make it stop when it reaches dest
// use lcd to figure out what values do.

