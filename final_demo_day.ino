#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <analogWrite.h>
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

uint32_t timer = millis();

QMC5883LCompass compass;
const int motorPin1 = 13;
const int motorPin2 = 12;
const int motorPin3 = 27;
const int motorPin4 = 33;
const int motorPin5 = 15;
const int motorPin6 = 32;
const int motorPin7 = 14;
const int buttonPin = 26;
const int switchPin = 25;

int previousMotorPin = 34;
int previousSwitchState = 0;
int switchState = 0;
int buttonState = 0;


struct geoLocFloat {
  //Patrick Suhm(2020) SelfDrivingRobot [Source code]. https://github.com/PatrickSuhm/SelfDrivingGPSRobot/blob/master/gps.h#L10
  float lat = 0;
  float lon = 0;
} wayPoint, current;


float getBearing(geoLocFloat a, geoLocFloat b) {
  //Patrick Suhm(2020) SelfDrivingRobot [Source code]. https://github.com/PatrickSuhm/SelfDrivingGPSRobot/blob/master/gps.h#L128
  //@brief: returns the angle between (a,b) and (a,North) in Degree.
  const float toRad = 0.0174533f;
  const float toDeg = 57.2957795f;
  float aLat = a.lat * toRad;
  float aLon = a.lon * toRad;
  float bLat = b.lat * toRad;
  float bLon = b.lon * toRad;
  float y = sin(bLon - aLon) * cos(bLat);
  float x = cos(aLat) * sin(bLat) - sin(aLat) * cos(bLat) * cos(bLon - aLon);
  return atan2(y, x) * toDeg;
}

void setMotorPin(int motorPin, float distanceToWayPoint) {
  if (previousMotorPin != 34) {
    analogWrite(previousMotorPin, 0);
  }
  analogWrite(motorPin, 255);
  delay(200);
  analogWrite(motorPin, 0);
  delay(distanceToWayPoint*10);
  previousMotorPin = motorPin;
}

void buttonOnFeedback() {
  int motorPins [7] = { motorPin1, motorPin2, motorPin3, motorPin4, motorPin5, motorPin6, motorPin7 };
  for (int i = 0; i < 7; i = i + 1) {
    analogWrite(motorPins[i], 255);
  }
  delay(1000);
  for (int i = 0; i < 7; i = i + 1) {
    analogWrite(motorPins[i], 0);
  }
}

void switchOnFeedback() {
  int motorPins [7] = { motorPin1, motorPin2, motorPin3, motorPin4, motorPin5, motorPin6, motorPin7 };
  for (int i = 0; i < 7; i = i + 1) {
    analogWrite(motorPins[i], 255);
    delay(200);
    analogWrite(motorPins[i], 0);
    delay(200);
  }
}



void setup() {
  Serial.begin(115200);
  compass.init(); 
//
//  GPS.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PGCMD_ANTENNA);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(switchPin, INPUT);

}

void loop() {

  switchState = digitalRead(switchPin);
  if (previousSwitchState != switchState) {
    if (previousSwitchState == 0) {
      switchOnFeedback();
    }
    previousSwitchState = switchState;
  }
  if (switchState == 1) {
//    char c = GPS.read();
//    if (GPSECHO)
//      if (c) Serial.print(c);
//    if (GPS.newNMEAreceived()) {
//      Serial.println(GPS.lastNMEA());
//      if (!GPS.parse(GPS.lastNMEA()))
//        return;
//    }

    if (millis() - timer > 2000) {
      timer = millis();
      buttonState = digitalRead(buttonPin);

//      if (GPS.fix) {
//        Serial.print("Location: ");
//        Serial.print(GPS.latitudeDegrees);
//        Serial.println(GPS.longitudeDegrees);
//      }
      wayPoint.lat = 51.449317743199195;
      wayPoint.lon = 5.487614638462321;

      if (buttonState == 1) {
        buttonOnFeedback();
      }
      Serial.print("button state");
      Serial.println(buttonState);
      int compassDirection;

      compass.read();
      
      int compassDegree = atan2( compass.getY(), compass.getZ() ) * 180.0 / PI;
      compassDirection = compassDegree < 0 ? 360 + compassDegree : compassDegree;
      Serial.print("compass degree");
      Serial.println(compassDirection);

      current.lat = 51.44755213228156;
      current.lon = 5.486264183735993;

      unsigned long distanceToWayPoint =
        (unsigned long)TinyGPSPlus::distanceBetween(
          wayPoint.lat,
          wayPoint.lon,
          current.lat,
          current.lon);
      Serial.println("distance to way point");
      Serial.println(distanceToWayPoint);

      float trueBearing = getBearing(current, wayPoint);
      if (trueBearing < 0) {
        trueBearing = trueBearing + 360;
      }
      Serial.print("trueBearing");
      Serial.println(trueBearing);

      float relativeBearing = trueBearing - compassDirection;
      if (relativeBearing < 0) {
        relativeBearing = relativeBearing + 360;
      }

      Serial.print("relativeBearing");
      Serial.println(relativeBearing);

      if (relativeBearing < 22.5 || relativeBearing > 337.5) {
      }
      else if (relativeBearing < 67.5) {
        setMotorPin(motorPin1, distanceToWayPoint);
      }
      else if (relativeBearing < 112.5) {
        setMotorPin(motorPin2, distanceToWayPoint);
      }
      else if (relativeBearing < 157.5) {
        setMotorPin(motorPin3, distanceToWayPoint);
      }
      else if (relativeBearing < 202.5) {
        setMotorPin(motorPin4, distanceToWayPoint);
      }
      else if (relativeBearing < 247.5) {
        setMotorPin(motorPin5, distanceToWayPoint);
      }
      else if (relativeBearing < 292.5) {
        setMotorPin(motorPin6, distanceToWayPoint);
      }
      else if (relativeBearing < 337.5) {
        setMotorPin(motorPin7, distanceToWayPoint);
      }
    }
  }
}
