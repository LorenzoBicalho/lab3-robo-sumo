// Inclui as bibliotecas
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>

// Pinagens
const int PIN_SERVO_MOTOR = 32;
const int PIN_SENSOR_INFRAVER_ESQUER = 2;
const int PIN_SENSOR_INFRAVER_DIREIT = 15;
const int PIN_SENSOR_INFRAVER_FUNDO = 4;
const int PIN_TRIGGER_ULTRASSONICO_DIR = 19;
const int PIN_ECHO_ULTRASSONICO_DIR = 21;
const int PIN_TRIGGER_ULTRASSONICO_ESQ = 22;
const int PIN_ECHO_ULTRASSONICO_ESQ = 23;
const int PIN_D0 = 13;
const int PIN_D1 = 12;
const int PIN_D2 = 14;
const int PIN_D3 = 27;

// Variáveis para o controle do motor
bool CW = true;
bool CCW = false;
bool debug = true;
const int freq = 5000;
const int ledChannel0 = 0;
const int ledChannel1 = 1;
const int ledChannel2 = 2;
const int ledChannel3 = 3;
const int resolution = 8;

// Declaração das funções
void led();
void M1(bool, int);
void M2(bool, int);
void brake(int);
void fullBrake();
void debugPrint(int, bool, int, bool);
void testeMotor();
void getDistance();
void processarControleAnalogico(String);

// Inicializa a classe do servomotor
Servo servoMotor;

//variáves para o sensor ultrassônico
unsigned int distancia1 = 0;
unsigned int distancia2 = 0;
Ultrasonic ultrasonic1(PIN_TRIGGER_ULTRASSONICO_DIR, PIN_ECHO_ULTRASSONICO_DIR);
Ultrasonic ultrasonic2(PIN_TRIGGER_ULTRASSONICO_ESQ, PIN_ECHO_ULTRASSONICO_ESQ);

// Configurações Bluetooth
BLECharacteristic *characteristicUS; //através desse objeto iremos enviar dados para o client

bool deviceConnected = false; //controle de dispositivo conectado

#define SERVICE_UUID           "ab0828b1-198e-4351-b779-901fa0e0371e" // UART service UUID
#define CHARACTERISTIC_UUID_US "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define CHARACTERISTIC_UUID_AN "5b59a827-bbca-4565-a0c9-3ae70c49fba4"
 
//callback para receber os eventos de conexão de dispositivos
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//callback  para envendos das características
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) {
      //retorna ponteiro para o registrador contendo o valor atual da caracteristica
      String rxValue = characteristic->getValue().c_str(); 
      
      //verifica se existe dados (tamanho maior que zero)
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        // Chama a função que faz o controle dos motores
        processarControleAnalogico(rxValue);
        
        /*
        int indiceX, indiceY; // Índices para localizar os valores de X e Y
        float valorX, valorY; // Variáveis para armazenar os valores convertidos
        bool sinalXPositivo, sinalYPositivo; // Variáveis booleanas para sinais
      
        // Localizar o índice do separador ':' após X
        indiceX = rxValue.indexOf(':');
      
        // Extrair o valor de X e convertê-lo para float
        valorX = rxValue.substring(indiceX + 2, rxValue.indexOf(',', indiceX)).toFloat();
      
        // Verificar o sinal de X
        if (valorX >= 0) {
          sinalXPositivo = true;
        } else {
          sinalXPositivo = false;
          valorX *= -1; // Remover sinal negativo para conversão
        }
      
        // Localizar o índice do separador ';' após Y
        indiceY = rxValue.indexOf(';', indiceX + 1);
      
        // Extrair o valor de Y e convertê-lo para float
        valorY = rxValue.substring(indiceY + 2).toFloat();
      
        // Verificar o sinal de Y
        if (valorY >= 0) {
          sinalYPositivo = true;
        } else {
          sinalYPositivo = false;
          valorY *= -1; // Remover sinal negativo para conversão
        }*/


        
        // Aqui adiciona-se qualquer tratamento relacionado às informações recebidas 

        Serial.println();
        Serial.println("*********");
      }
    }
};


void setup() {

  Serial.begin(115200);

  // Associa a pinagem dos sensores de refletância
  pinMode(PIN_SENSOR_INFRAVER_ESQUER, INPUT);
  pinMode(PIN_SENSOR_INFRAVER_DIREIT, INPUT);
  pinMode(PIN_SENSOR_INFRAVER_FUNDO, INPUT);
  pinMode(PIN_D0, OUTPUT);
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_D2, OUTPUT);
  pinMode(PIN_D3, OUTPUT);

  // Configuração dos canais PWM
  ledcSetup(ledChannel0, freq, resolution);
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);

  // Atribuição dos canais PWM aos pinos
  ledcAttachPin(PIN_D0, ledChannel0);
  ledcAttachPin(PIN_D1, ledChannel1);
  ledcAttachPin(PIN_D2, ledChannel2);
  ledcAttachPin(PIN_D3, ledChannel3);
  
  // Associa a pinagem do servomotor
  servoMotor.attach(PIN_SERVO_MOTOR);
  
   // Cria o dispositivo 
  BLEDevice::init("Robô Pink"); // nome do dispositivo bluetooth
  
  // Cria o servidor
  BLEServer *server = BLEDevice::createServer(); //cria um BLE server 
  server->setCallbacks(new ServerCallbacks()); //seta o callback do server
  
  // Cria o serviço
  BLEService *service = server->createService(SERVICE_UUID);
  
  // Cria a característica para envio de dados
  characteristicUS = service->createCharacteristic(
                      CHARACTERISTIC_UUID_US,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  characteristicUS->addDescriptor(new BLE2902());

  // Cria a característica para recebimento de dados
  BLECharacteristic *characteristic = service->createCharacteristic(
                                         CHARACTERISTIC_UUID_AN,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  characteristic->setCallbacks(new CharacteristicCallbacks());
  
  // Inicia o serviço
  service->start();
  
  // Start advertising (descoberta do ESP32)
  server->getAdvertising()->start(); 
  
  Serial.println("Waiting a client connection to notify...");

}

void loop() {

  // Envio de informações pelo bluetooth
  characteristicUS->setValue("Hello!"); // Sending a test message
  characteristicUS->notify(); // Send the value to the app!

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

void M1(bool direction, int speed) {
  int pwm = map(speed, 0, 100, 0, 255);
  if(direction == CW) {
    ledcWrite(ledChannel0, pwm);
    ledcWrite(ledChannel1, 0);
  } else {
    ledcWrite(ledChannel1, pwm);
    ledcWrite(ledChannel0, 0);
  }
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
}

void brake(int motor) {
  if(motor == 1) {
    ledcWrite(ledChannel0, 255);
    ledcWrite(ledChannel1, 255);
  } else {
    ledcWrite(ledChannel2, 255);
    ledcWrite(ledChannel3, 255);
  }
}

void fullBrake() {
  brake(1);
  brake(2); 
}

void processarControleAnalogico(String sinal) {
    // Encontrar os valores de X e Y no sinal recebido
    int posicaoX = sinal.indexOf("X:");
    int posicaoY = sinal.indexOf("Y:");

    // Se X e Y forem encontrados
    if (posicaoX != -1 && posicaoY != -1) {
        // Extrair os valores de X e Y
        int valorX = sinal.substring(posicaoX + 3, sinal.indexOf(';', posicaoX)).toInt();
        int valorY = sinal.substring(posicaoY + 3, sinal.indexOf(';', posicaoY)).toInt();

        // Determinar a direção e a velocidade dos motores com base nos valores de X e Y
        bool direcaoM1, direcaoM2;
        int velocidadeM1, velocidadeM2;

        if (valorX > 0) {
            // Movimento para frente
            direcaoM1 = true;
            direcaoM2 = true;
        } else if (valorX < 0) {
            // Movimento para trás
            direcaoM1 = false;
            direcaoM2 = false;
        } else {
            // X é zero, não há movimento horizontal
            // Você pode adicionar lógica adicional aqui, se necessário
        }

        // Lógica semelhante para o movimento vertical (Y)
        if (valorY > 0) {
            // Ajustar velocidade baseada no valor absoluto de Y
            velocidadeM1 = abs(valorY);
            velocidadeM2 = abs(valorY);
        } else if (valorY < 0) {
            // Ajustar velocidade baseada no valor absoluto de Y
            velocidadeM1 = abs(valorY);
            velocidadeM2 = abs(valorY);
        } else {
            // Y é zero, não há movimento vertical
            // Você pode adicionar lógica adicional aqui, se necessário
        }

        // Controlar os motores com base nas direções e velocidades determinadas
        M1(direcaoM1, velocidadeM1);
        M2(direcaoM2, velocidadeM2);
    }
}
