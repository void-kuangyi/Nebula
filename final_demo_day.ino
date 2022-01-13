#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

uint32_t timer = millis();

QMC5883LCompass compass;
const int motorPin1 = 12;
const int motorPin2 = 11;
const int motorPin3 = 10;
const int motorPin4 = 9;
const int motorPin5 = 8;
const int motorPin6 = 7;
const int motorPin7 = 6;
const int buttonPin = 5;
const int switchPin = 2;

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
  if (distanceToWayPoint < 3000) {
    analogWrite(motorPin, 255);
    delay(200);
    analogWrite(motorPin, 0);
    delay(distanceToWayPoint);
  } else {
    analogWrite(motorPin, 255);
    delay(200);
    analogWrite(motorPin, 0);
    delay(3000);
  }
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
  ss.begin(GPSBaud);

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
    while (ss.available() > 0)
      if (gps.encode(ss.read())) {
        Serial.print(F("Location: "));
        if (gps.location.isValid()) {
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(","));
          Serial.print(gps.location.lng(), 6);
          current.lat = gps.location.lat();
          current.lon = gps.location.lng();
        }
      }
    if (millis() - timer > 300) {
      timer = millis();
      buttonState = digitalRead(buttonPin);
      wayPoint.lat = 51.447918488941546;
      wayPoint.lon = 5.490951480719493;
      Serial.print("button state");
      Serial.println(buttonState);

      int compassDirection;
      compass.read();
      int compassDegree = atan2( compass.getY(), compass.getZ() ) * 180.0 / PI;
      compassDirection = compassDegree < 0 ? 360 + compassDegree : compassDegree;
      Serial.print("compass degree");
      Serial.println(compassDirection);

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
