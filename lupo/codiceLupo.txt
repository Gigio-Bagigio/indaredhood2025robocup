// codice_lupo
/* 
  rosso = errore sensore di distanza
  giallo = errore sensore gesture
  blu = errore sensore distanza e gesture
*/
#include <Encoder.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include "Gesture.h" // seeed studio paj7620
#define GES_REACTION_TIME    800
#define GES_QUIT_TIME        1000

paj7620 Gesture;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // servo quando abbassa
#define SERVOMAX  600 // servo quando alza

#define BT_NAME   "Master" // bluetooth

const int RXPin = 0;     // da collegare su TX di HC05
const int TXPin = 1;     // da collegare su RX di HC05
const int ritardo = 200; // tempo necessario tra un comando ed il successivo

SoftwareSerial mySerial(RXPin, TXPin); // RX, TX

#define LONG_RANGE

VL53L0X sensor; // sensore laser di distanza , TCS34725

int braccioSx = 0;
int braccioDx = 1;

int r = 2;
int g = 3;
int b = 4;

const float raggioRuota = 5.0;
const float circonferenzaRuota = 2*raggioRuota*PI; 

////////////////////              variabili giroscopio                 ////////////////////
float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // accelerometro: offset e scale
float G_off[3] = {0.0, 0.0, 0.0}; // gyro offsets
#define gscale ((250.0/32768.0)*(PI/180.0))  // Conversione: 250 LSB/dps -> rad/s
float q[4] = {1.0, 0.0, 0.0, 0.0};

const int MPU_addr = 0x68;
const int cal_gyro = 1;

unsigned long lastTimeMicros;
unsigned long lastSensorUpdateTime = 0;
const unsigned long sensorIntervalMs = 5;  // Aggiornamento sensore ogni 5 ms (modifica in base alle esigenze)

// Variabili per gli angoli (in gradi)
float yaw, pitch, roll;
///////////////////////////////////
float Kp = 30.0;
float Ki = 0.0;

unsigned long now_ms, last_ms = 0;
unsigned long print_ms = 200;
float yaw_total = 0.0; // Variabile per accumulare il valore della yaw
///////////////////////////////////

// motore sinistro
Encoder myEnc(2, 3);
int enA = 9;
int in1 = 8;
int in2 = 7;
// motore destro
Encoder myEnc2(4, 5);
int enB = 11;
int in3 = 13;
int in4 = 12;

int angoloDiTolleranza = 2; // angolo di tolleranza per destra e sinistra
int angoloDiTolleranzaAv = 0;

bool onAdjustAngle = false;

bool laserSensore = true;
bool gestureSensor = true;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // setup
  
  Serial.begin(9600);
  Wire.begin();

  if (Gesture.init()) {
      Serial.println("INIT ERROR");
      gestureSensor = false;
      giallo();
  }

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
  sensor.setTimeout(500);
  // bluetooth
  mySerial.begin(38400);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////sensore laser di distanza////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    laserSensore = false;
    rosso();
    if (!gestureSensor) {
      blu();
    }
  }
  #if defined LONG_RANGE
  sensor.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////encoder ruote//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  stop(); // imposto i sensori come spenti e a velocità 0
  //Serial.println("la circonferenza della ruota è: " + (String) circonferenzaRuota);

  //////////giroscopio/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Inizializza MPU-6050
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  lastTimeMicros = micros();
  delay(2000);
  int calibrationTime = millis();
  calibration();
  Serial.println("tempo di calibrazione: " + (String) (millis() - calibrationTime));

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////movimenti/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float angle;
  float distanza;
  float startingYaw;
  // avanti
  long newPosition = 0;

  attendiDistanza(20000);
  avantiMod2(170);
  myDelay(2000);
  destra(90);
  spedisciSegnaleBlue('l');
  myDelay(500);
  avantiMod2(200);
  myDelay(1000);
  bracciaSu();
  aspettaDestraSinistra(10000);
  bracciaGiu();
  avantiMod2(50);
  // verificare che il collegamento bluetooth sia stato effettuato e mettere un timeout
  // while (!mySerial.available()){}
  stop();
}

void sinistra(float angle) {
  float finalAngle = yaw_total - angle;
  motoriSinistra();
  while (yaw_total > finalAngle) {
    updateSensor();
  }
  stop();
  myDelay(300);
  if (finalAngle > yaw_total + angoloDiTolleranza) {
    aggiustaAngolo(finalAngle);
  }
  stop();
}

void destra(float angle) {
  float finalAngle = yaw_total + angle;
  motoriDestra();
  while (yaw_total < finalAngle) {
    updateSensor();
  }
  onAdjustAngle = true;
  stop();
  onAdjustAngle = false;
  myDelay(300);
  if (finalAngle < yaw_total - angoloDiTolleranza) {
    aggiustaAngolo(finalAngle);
  }

  stop();
}

void aggiustaAngolo(float angolo) { // angolo == angolo a cui voglio che vada, startingYaw in "avanti"
  // 
  onAdjustAngle = true;
  if (yaw_total < angolo){
    destra(angolo-yaw_total);
  } 
  if (yaw_total > angolo) {
    sinistra(yaw_total-angolo);
  }
  onAdjustAngle = false;
}


void stop(){
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void motoriIndietro() {
  stop();
  analogWrite(enA, 80);
  analogWrite(enB, 80);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void motoriAvanti() {
  stop();
  analogWrite(enA, 80);
  analogWrite(enB, 80);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void motoriDestra() {
  stop();
  if (onAdjustAngle) {
    analogWrite(enA, 45);
    analogWrite(enB, 45);
  } else {
    analogWrite(enA, 55);
    analogWrite(enB, 55);
  }
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void motoriSinistra() {
  stop();
  if (onAdjustAngle) {
    analogWrite(enA, 45);
    analogWrite(enB, 45);
  } else {
    analogWrite(enA, 55);
    analogWrite(enB, 55);
  }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
////////////////////////////////////////////////////////////////////////////////////
void loop(){}
///////////////////////////funzioni braccia/////////////////////////////////////////
void bracciaSu(){
  int b = SERVOMAX; // Servo 2 parte da massimo mentre il primo parte da minimo

  for (int a = SERVOMIN; a <= SERVOMAX; a++) {
    pwm.setPWM(braccioSx, 0, a);  // Servo 1 si muove da min a max
    pwm.setPWM(braccioDx, 0, b);  // Servo 2 si muove da max a min
    b--;  // Decrementa b per ottenere il movimento opposto
    delay(2);
  }
}

void bracciaGiu(){
  int c = SERVOMIN;  // Servo 2 parte da minimo mentre il primo parte da massimo
  
  for (int a = SERVOMAX; a >= SERVOMIN; a--) {
    pwm.setPWM(braccioSx, 0, a);  // Servo 1 si muove da max a min
    pwm.setPWM(braccioDx, 0, c);  // Servo 2 si muove da min a max
    c++;  // Incrementa c per ottenere il movimento opposto
    delay(2);
  }
}

void dxSu(){
  for (int a = SERVOMAX; a >= SERVOMIN; a--) {
    pwm.setPWM(braccioDx, 0, a);
    delay(2);
  }
}

void sxSu(){
  for (int a = SERVOMIN; a <= SERVOMAX; a++) {
    pwm.setPWM(braccioSx, 0, a);
    delay(2);
  }
}

void dxGiu(){
  for (int a = SERVOMIN; a <= SERVOMAX; a++) {
    pwm.setPWM(braccioDx, 0, a);
    delay(2);
  }
}

void sxGiu(){
  for (int a = SERVOMAX; a >= SERVOMIN; a--) {
    pwm.setPWM(braccioSx, 0, a);
    delay(2);
  }
}

////////////////////////////////////////////////////////////////////////////////////

void myDelay(int time) {
  int now = millis();
  while(time > millis() - now){
    updateSensor();
  }
}

void calibration() {
  const int numReadings = 500;
  long sum[3] = {0, 0, 0};

  for (int i = 0; i < numReadings; i++) {
    int16_t gx, gy, gz;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);
    int t = Wire.read() << 8;
    gx = t | Wire.read();
    t = Wire.read() << 8;
    gy = t | Wire.read();
    t = Wire.read() << 8;
    gz = t | Wire.read();

    sum[0] += gx;
    sum[1] += gy;
    sum[2] += gz;
    delay(5); // breve pausa per la calibrazione (è accettabile in fase di avvio)
  }
  for (int i = 0; i < 3; i++) {
    G_off[i] = (float)sum[i] / numReadings;
  }
  Serial.print("G_Off: ");
  Serial.print(G_off[0]);
  Serial.print(", ");
  Serial.print(G_off[1]);
  Serial.print(", ");
  Serial.println(G_off[2]);
}

void updateSensor() {
  static unsigned int i = 0;
  static float deltat = 0;
  static unsigned long now = 0, last = 0;
  static long gsum[3] = {0};

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp;

  float Axyz[3];
  float Gxyz[3];

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14);
  int t = Wire.read() << 8;
  ax = t | Wire.read();
  t = Wire.read() << 8;
  ay = t | Wire.read();
  t = Wire.read() << 8;
  az = t | Wire.read();
  t = Wire.read() << 8;
  Tmp = t | Wire.read();
  t = Wire.read() << 8;
  gx = t | Wire.read();
  t = Wire.read() << 8;
  gy = t | Wire.read();
  t = Wire.read() << 8;
  gz = t | Wire.read();

  i++;
  
  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];
  Gxyz[0] = ((float) gx - G_off[0]) * gscale;
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;


  now = micros();
  deltat = (now - last) * 1.0e-6;
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  float prev_yaw = yaw;
  yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
  yaw *= 180.0 / PI;


  /* MODIFICA: Calcolo della variazione della yaw per mantenerla accumulata senza reset */
  float yaw_diff = yaw - prev_yaw;
  if (yaw_diff > 180) yaw_diff -= 360;
  if (yaw_diff < -180) yaw_diff += 360;
  yaw_total += yaw_diff; // Accumulo della yaw
  /* FINE MODIFICA */

  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3])) * 180.0 / PI;
  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2])) * 180.0 / PI;

  Serial.print(yaw_total, 0); // Stampa della yaw accumulata
  Serial.print(", ");
  Serial.print(pitch, 0);
  Serial.print(", ");
  Serial.println(roll, 0);  
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
 
  // Normalizzazione dell'accelerometro
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0) return;
  norm = 1.0 / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;


  // Aggiornamento del quaternione
  float qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz);
  float qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy);
  float qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx);
  float qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx);
 
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
 
  // Normalizzazione del quaternione
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  if (norm == 0.0) return;
  norm = 1.0 / norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
  q4 *= norm;
 
  q[0] = q1; q[1] = q2; q[2] = q3; q[3] = q4;
}

// motore sinistro = enA
// motore destro = enB

void avantiMod2(float distanza) {
  stop();
  analogWrite(enA, 80); // motore sinistro
  analogWrite(enB, 80); // motore destro
  motoriAvanti();
  int potenzaEnB = 80;
  int potenzaEnA = 80;
  float startingYaw = yaw_total;
  float startingPosition = myEnc.read();
  float newPosition = 0;
  bool adjustA = false;
  bool adjustB = false;
  unsigned long time = millis();
  while (((newPosition-startingPosition)/6533.)*circonferenzaRuota < distanza) { 
    newPosition = myEnc.read();
    updateSensor();
    if (yaw_total - angoloDiTolleranzaAv > startingYaw){//&& !adjustB) { // ha virato a destra
      // dare più potenza al motore destro (enB)
      if (potenzaEnB > 80) { // hai gia accellerato?
        if (millis() > time + 350) {
          time = millis();
          potenzaEnB += 5;
        }
      } else {
        time = millis();
        potenzaEnB = 81;
      }
      analogWrite(enB, potenzaEnB); 
      adjustB = true;
      // reimposto ai valori di default l'altro motore e la variabile di correzzione perchè se no rischio che ci sia un aumento da entrambi i motori continuo arrivando ad una velocità totale molto elevata
      adjustA = false;
      potenzaEnA = 80;
      analogWrite(enA, potenzaEnA); 
    }  
    if (yaw_total + angoloDiTolleranzaAv < startingYaw ) {// && !adjustA) { // ha virato a sinistra
      // dare più potenza al motore sinistro (enA)
      if (potenzaEnA > 80) { // hai gia accellerato?
        if (millis() > time + 350) { 
          time = millis();
          potenzaEnA += 5;
        }
      } else {
        time = millis();
        potenzaEnA = 81;
      }
      analogWrite(enA, potenzaEnA); 
      adjustA = true;

      adjustB = false;
      potenzaEnB = 80;
      analogWrite(enB, potenzaEnB); 
    }

    if (yaw_total + angoloDiTolleranzaAv > startingYaw && yaw_total - angoloDiTolleranzaAv < startingYaw) {
      adjustA = false;
      adjustB = false;
      potenzaEnB = 80;
      potenzaEnA = 80;
      analogWrite(enA, 80); // default power
      analogWrite(enB, 80); 
    }
  }  
  //motoriIndietro();
  //myDelay(50);
  aggiustaAngolo(startingYaw);
  stop();
}

void accelerazione() {
  analogWrite(enA, 0); // motore sinistro
  analogWrite(enB, 0); // motore destro
  for (int i = 10; i <= 80; i+=10){
    analogWrite(enA, i); // motore sinistro
    analogWrite(enB, i);
    myDelay(30);
  }
}
void decelerazzione() {
  for (int i = 80; i > 0; i-=10){
    analogWrite(enA, i); // motore sinistro
    analogWrite(enB, i);
    myDelay(30);
  }
}

void setColor(uint16_t red, uint16_t green, uint16_t blue) {
  pwm.setPWM(r, 0, red);   // Canale 0 → rosso
  pwm.setPWM(g, 0, green); // Canale 1 → verde
  pwm.setPWM(b, 0, blue);  // Canale 2 → blu
}

void rosso() {
  setColor(4095, 0, 0);
}
void verde() {
  setColor(0, 4095, 0);
}
void blu() {
  setColor(0, 0, 4095);
}
void giallo() {
  setColor(4095, 4095, 0);
}
void ciano() {
  setColor(0, 4095, 4095);
}
void magenta() {
  setColor(4095, 0, 4095);
}
void bianco() {
  setColor(4095, 4095, 4095);
}
void viola() {
  setColor(2048, 0, 2048); // viola più soft
}
void arancione() {
  setColor(4095, 1500, 0);
}
void spento() {
  setColor(0, 0, 0);
}
void aspettaDestraSinistra(int timeOut) {
  int destra = 0;
  int sinistra = 0;
  unsigned long startTime = millis();

  paj7620_gesture_t result;

  while ((destra < 3 && sinistra < 3) && (millis() - startTime < timeOut)) { // destra < 3 || sinistra < 3 ? testare come funziona il gesture...
    updateSensor(); // in teoria non serve perche viene aggioranto dentro myDelay
    if (Gesture.getResult(result)) {
        switch (result) {
            case RIGHT:
                Serial.println("Right");
                destra++;
                break;
            case LEFT:
                Serial.println("Left");
                sinistra++;
                break;
            default:
                break;
        }
    }
    myDelay(100);
  }
}

void attendiSegnaleBlue(int timeOut, char r) {
  unsigned int startTime = millis();
  while (startTime - millis() <= timeOut) {
    if (mySerial.available()) { 
      char receivedChar = mySerial.read(); // Legge SOLO se disponibile
      Serial.print(receivedChar);  // Debug sulla seriale
      if (receivedChar == r) {
        Serial.println("\nRicevuto carattere corretto! Esco dal loop.");
        break; // Esce dal loop
      }
    }
    updateSensor(); // Continua ad aggiornare il sensore
  }
}

void spedisciSegnaleBlue(char c){
  if (mySerial.available()) {
    for (int i = 0; i < 10; i++) {  
      Serial.write(c);
      myDelay(50);
    }
  } else {
    viola();
  }
}

void attendiDistanza(int timeOut) {
  unsigned long startTime = millis();
  while (sensor.readRangeSingleMillimeters()/10 < 100 && millis() - startTime < timeOut){
    updateSensor();
  }
}
