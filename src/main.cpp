#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>

#define INTERVALO_LEITURA 250 //(ms)
#define LED_PIN 2

// Variáveis para o servomotor
Servo servoMotor;
const int PIN_SERVO_MOTOR = 32;

// variáveis para os sensores infravermelho
const int PIN_SENSOR_INFRAVERMELHO1 = 26;
const int PIN_SENSOR_INFRAVERMELHO2 = 25;
const int PIN_SENSOR_INFRAVERMELHO3 = 33;

// Pinout Ponte H 
#define D0_PIN 13
#define D1_PIN 12
#define D2_PIN 14
#define D3_PIN 27

//variáves para o sensor ultrassônico
unsigned int distancia1 = 0;
unsigned int distancia2 = 0;
const int PIN_TRIGGER_ULTRASSONICO1 = 4;
const int PIN_ECHO_ULTRASSONICO1 = 2;
Ultrasonic ultrasonic1(PIN_TRIGGER_ULTRASSONICO1, PIN_ECHO_ULTRASSONICO1);
const int PIN_TRIGGER_ULTRASSONICO2 = 21;
const int PIN_ECHO_ULTRASSONICO2 = 19;
Ultrasonic ultrasonic2(PIN_TRIGGER_ULTRASSONICO2, PIN_ECHO_ULTRASSONICO2);

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
void getDistance();

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

  // Atribuição dos canais dos sensores Infravermelho
  pinMode(PIN_SENSOR_INFRAVERMELHO1, INPUT);
  pinMode(PIN_SENSOR_INFRAVERMELHO2, INPUT);
  pinMode(PIN_SENSOR_INFRAVERMELHO3, INPUT);
  
  // Atribuição do Canal do Servo
  servoMotor.attach(PIN_SERVO_MOTOR);
  
  // Inicializa a saída serial
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  testeMotor();

  // Fazendo leituras do sensor infravermelho
   bool estadoSensor1 = digitalRead(PIN_SENSOR_INFRAVERMELHO1);
   bool estadoSensor2 = digitalRead(PIN_SENSOR_INFRAVERMELHO2);
   bool estadoSensor3 = digitalRead(PIN_SENSOR_INFRAVERMELHO3);
   if (estadoSensor1) {
    Serial.println("PRETO");
  } else {
    Serial.println("BRANCO (diferente de preto)");
  }

  // Movendo o Servo motor
  servoMotor.write(90);
  
  // Fazendo leituras do sensor ultrassônico
  getDistance();
  delay(INTERVALO_LEITURA);
  
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

void getDistance()
{
    //faz a leitura das informacoes do sensor (em cm)
    int distanciaCM1;
    long microsec1 = ultrasonic1.timing();
    // pode ser um float ex: 20,42 cm se declarar a var float 
    distanciaCM1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
    Serial.println(distanciaCM1);

        //faz a leitura das informacoes do sensor (em cm)
    int distanciaCM2;
    long microsec2 = ultrasonic2.timing();
    // pode ser um float ex: 20,42 cm se declarar a var float 
    distanciaCM2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);
    Serial.println(distanciaCM2);
}