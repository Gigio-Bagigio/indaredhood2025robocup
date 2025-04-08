#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// in1   in2   spinning direction
// LOW   LOW   motor off
// HIGH  LOW   forward
// LOW   HIGH  backward
// HIGH  HIGH  motor off

/*
Input1  Input2  Spinning Direction
Low(0)  Low(0)  Motor OFF
High(1) Low(0)  Forward
Low(0)  High(1) Backward
High(1) High(1) Motor OFF
*/
// Motor A connections
int enA = 6; // pin del pwm
int in1 = 7; // direzione 
int in2 = 8; // direzione
// se in1 HIGH e in2 LOW -> avanti
// se in1 LOW e in2 HIGH -> indietro
// se entrambi su HIGH o LOW -> fermo
// Motor B connections
int enB = 5; // pin del pwm
int in3 = 4; // direzione
int in4 = 3; // direzione
// se in3 HIGH e in4 LOW -> avanti
// se in3 LOW e in4 HIGH -> indietro
// se entrambi su HIGH o LOW -> fermo

int r = 9;
int g = 10;
int b = 11;

bool laserSensorState = true;

#define LONG_RANGE


void setup() {
  Wire.begin();
  if (!sensor.init()) {
    laserSensorState = false;
  } else {
    #if defined LONG_RANGE
      sensor.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif
  }
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  stop();

  unsigned long startTime = millis();
  unsigned int timeout = 20000; // 20 secondi 
  if (laserSensorState) {
    uint16_t distance = sensor.readRangeSingleMillimeters();
    while (distance >= 8190 && millis() - startTime < timeout) {
      distance = sensor.readRangeSingleMillimeters();
    }
    if (millis() - startTime >= timeout) {
      viola();
      delay(5000);
    }
  } else {
    rosso();
    delay(10000);
  }

  avanti(128, 2000);
}

void loop() {}

void avanti(int v, int t) {
  // imposto i motori alla massima velocità (ma sono ancora spenti i motori) non serve in realtà
  analogWrite(enA, v);
  analogWrite(enB, v);

  // imposto il motore A per andare avanti
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);

  // imposto il motore B per andare avanti
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  delay(t);
  
  // spengo i motori
  stop();
}


void indietro(int v, int t) {
  // imposto i motori alla massima velocità (ma sono ancora spenti i motori) non serve in realtà
  analogWrite(enA, v);
  analogWrite(enB, v);

  // imposto il motore A per andare indietro
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // imposto il motore B per andare indietro
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(t);

  // spengo i motori
  stop();
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Imposta il colore con valori RGB (0–255)
void setColor(int red, int green, int blue) {
  analogWrite(r, red);
  analogWrite(g, green);
  analogWrite(b, blue);
}

void rosso() {
  setColor(255, 0, 0);
}

void verde() {
  setColor(0, 255, 0);
}

void blu() {
  setColor(0, 0, 255);
}

void giallo() {
  setColor(255, 255, 0);
}

void ciano() {
  setColor(0, 255, 255);
}

void magenta() {
  setColor(255, 0, 255);
}

void bianco() {
  setColor(255, 255, 255);
}

void viola() {
  setColor(128, 0, 128);
}

void arancione() {
  setColor(255, 80, 0);
}

void spento() {
  setColor(0, 0, 0);
}