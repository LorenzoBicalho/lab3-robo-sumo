#include <Arduino.h>

#define LED_PIN 2

// Pinout Ponte H 
#define D0_PIN 13
#define D1_PIN 12
#define D2_PIN 14
#define D3_PIN 27

bool CW = true;
bool CCW = false;

bool debug = true;

// Parâmetros PWM para o ESP32
const int freq = 5000;
const int ledChannel0 = 0;
const int ledChannel1 = 1;
const int ledChannel2 = 2;
const int ledChannel3 = 3;
const int resolution = 8;

// put function declarations here:
void led();
void M1(bool, int);
void M2(bool, int);
void brake(int);
void fullBrake();
void debugPrint(int, bool, int, bool);
void testeMotor();

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(D0_PIN, OUTPUT);
  pinMode(D1_PIN, OUTPUT);
  pinMode(D2_PIN, OUTPUT);
  pinMode(D3_PIN, OUTPUT);

  // Configuração dos canais PWM
  ledcSetup(ledChannel0, freq, resolution);
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);

  // Atribuição dos canais PWM aos pinos
  ledcAttachPin(D0_PIN, ledChannel0);
  ledcAttachPin(D1_PIN, ledChannel1);
  ledcAttachPin(D2_PIN, ledChannel2);
  ledcAttachPin(D3_PIN, ledChannel3);
}

void loop() {
  // put your main code here, to run repeatedly:
  testeMotor();
  
}

// put function definitions here:
void led() {
  digitalWrite(LED_PIN, HIGH);
  Serial.print("Pisca\n");
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void M1(bool direction, int speed) {
  int pwm = map(speed, 0, 100, 0, 255);
  if(direction == CW) {
    ledcWrite(ledChannel0, pwm);
    ledcWrite(ledChannel1, 0);
  } else {
    ledcWrite(ledChannel1, pwm);
    ledcWrite(ledChannel0, 0);
  }
  debugPrint(1, direction, speed, false); 
}

void M2(bool direction, int speed) {
  int pwm = map(speed, 0, 100, 0, 255);
  if(direction == CW) {
    ledcWrite(ledChannel2, pwm);
    ledcWrite(ledChannel3, 0);
  } else {
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, pwm);
  }
  debugPrint(2, direction, speed, false); 
}

void brake(int motor) {
  if(motor == 1) {
    ledcWrite(ledChannel0, 255);
    ledcWrite(ledChannel1, 255);
  } else {
    ledcWrite(ledChannel2, 255);
    ledcWrite(ledChannel3, 255);
  }
  debugPrint(motor, true,  0, true);
}

void fullBrake() {
  brake(1);
  brake(2); 
}

void debugPrint(int motor, bool direction, int speed, bool stop) {
  if(debug) {
    Serial.print("Motor: ");
    Serial.print(motor);
    if(stop && motor > 0) {
      Serial.println(" Stopped");
    } else {
      if(direction) {
        Serial.print(" CW at ");
      } else {
        Serial.print(" CCW at ");     
      }
      Serial.print(speed);       
      Serial.println(" %");    
    }
  } 
}

void testeMotor(){
  led();
  M1(CW, 80);
  M2(CW, 80);
  delay(3000);
  fullBrake();
  delay(1000);

  led();
  led();
  M1(CCW, 80);
  M2(CCW, 80);
  delay(3000);
  fullBrake();
  delay(1000);

  led();
  led();
  led();
  for (int i = 0; i <= 100; i++) {
    M1(CW, i);
    M2(CW, i);
    delay(100);
  }
  delay(3000);
  fullBrake();
  delay(1000);
}
