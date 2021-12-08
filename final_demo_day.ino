#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <analogWrite.h>

QMC5883LCompass compass;
int motorPin1 = A1;
int motorPin2 = A2;
int motorPin3 = A3;
int motorPin4 = A4;
int motorPin5 = A5;
int motorPin6 = A6;
int motorPin7 = A7;

int previousMotorPin;

struct geoLocFloat {
  float lat;
  float lon;
} wayPoint, current;

float getBearing(geoLocFloat a, geoLocFloat b) {
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

void setMotorPin(int motorPin, float intensity) {
   analogWrite(previousMotorPin, 0);
   analogWrite(motorPin, intensity);
   previousMotorPin = motorPin;
}


void setup() {
  Serial.begin(9600);
  compass.init();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
  pinMode(motorPin7, OUTPUT);
}

void loop() {
  int compassDirection;

  compass.read();

  compassDirection = compass.getAzimuth();



  wayPoint.lat = 51.449390;
  wayPoint.lon = 5.488626;

  current.lat =  51.448723;
  current.lon =   5.485464;

  unsigned long distanceToWayPoint =
    (unsigned long)TinyGPSPlus::distanceBetween(
      wayPoint.lat,
      wayPoint.lon,
      current.lat,
      current.lon);
  Serial.println(distanceToWayPoint);

  float trueBearing = getBearing(current, wayPoint);
  if (trueBearing < 0) {
    trueBearing = trueBearing + 360;
  }
  Serial.println(trueBearing);

  float relativeBearing = trueBearing - compassDirection;
  if (relativeBearing < 0) {
    relativeBearing = relativeBearing + 360;
  }

  Serial.println(relativeBearing);

  float intensity = (255 * distanceToWayPoint) / 200;

  if (relativeBearing < 22.5 || relativeBearing > 337.5) {
  }
  else if (relativeBearing < 67.5) {
    setMotorPin(motorPin1, intensity);
  }
  else if (relativeBearing < 112.5) {
    setMotorPin(motorPin2, intensity);
  }
  else if (relativeBearing < 157.5) {
    setMotorPin(motorPin3, intensity);
  }
  else if (relativeBearing < 202.5) {
    setMotorPin(motorPin4, intensity);
  }
  else if (relativeBearing < 247.5) {
    setMotorPin(motorPin5, intensity);
  }
  else if (relativeBearing < 292.5) {
    setMotorPin(motorPin6, intensity);
  }
  else if (relativeBearing < 337.5) {
    setMotorPin(motorPin7, intensity);
  }

  delay(2000);
}
