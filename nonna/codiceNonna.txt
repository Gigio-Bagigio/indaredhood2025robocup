#include <SoftwareSerial.h>
#include <VL53L0X.h>

VL53L0X sensor; 

#define LONG_RANGE

const int RXPin = 0;     // da collegare su TX di HC05
const int TXPin = 1;     // da collegare su RX di HC05
const int ritardo = 200; // tempo necessario tra un comando ed il successivo
SoftwareSerial mySerial(RXPin, TXPin); // RX, TX
// in1   in2   spinning direction
// LOW   LOW   motor off
// HIGH  LOW   forward
// LOW   HIGH  backward
// HIGH  HIGH  motor off

/*
Input1	Input2	Spinning Direction
Low(0)	Low(0)	Motor OFF
High(1)	Low(0)	Forward
Low(0)	High(1)	Backward
High(1)	High(1)	Motor OFF
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
bool laserSensore = true; 
void setup() {
  Serial.begin(38400);
  mySerial.begin(38400);
  sensor.setTimeout(500);


  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    laserSensore = false;
    //rosso();
  }
  #if defined LONG_RANGE
  sensor.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  // imposto tutti i pin come output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // spengo tutti i motori
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  attendiSegnaleBlue(30000, 'l');
  avanti(128, 4000);
  while(true);
}

void loop() {

  // va avanti a v per t
  avanti(128, 2000);

  // va indietro a v per t
  indietro(128, 2000);
  /*
  // va a sinistra a v per t
  sinistra(128, 2000);

  // va a destra a v per t
  destra(128,2000);
  */

}

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
  
  // ritardo, trovere soluzione alternativa
  delay(t);
  
  // spengo i motori
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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
  
  // ritardo, trovere soluzione alternativa
  delay(t);

  // spengo i motori
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void attendiSegnaleBlue(int timeOut, char r) {
  unsigned int startTime = millis();
  while (startTime - millis() <= timeOut) {
    if (laserSensore){
      if (sensor.readRangeSingleMillimeters()/10 < 120){
        break;
      }
      
    }
    if (mySerial.available()) { 
      char receivedChar = mySerial.read(); // Legge SOLO se disponibile
      Serial.print(receivedChar);  // Debug sulla seriale
      if (receivedChar == r) {
        Serial.println("\nRicevuto carattere corretto! Esco dal loop.");
        break; // Esce dal loop
      }
    }
  }
}
