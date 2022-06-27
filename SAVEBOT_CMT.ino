//================= Bibliotecas ===================
#include <mecanum_Wheels.h> //biblioteca para controle das rodas mecanum
#include <RGBsensor.h> // biblioteca para controle dos sensores rgb
#include <Adafruit_VL53L0X.h> // biblioteca para controle do sensor de distancia a laser
#include <Servo.h> // biblioteca para controle dos servo motores
#include <Adafruit_SSD1306.h> // biblioteca para controle do display oled
#include <Adafruit_GFX.h> // biblioteca para controle do display oled
#include <Wire.h>// biblioteca para controle do display oled
#include <ButtonLibrary.h> // biblioteca para controle dos botões

//================ Constantes ===================
int left_h[6] = {4,5,6,7,2,3}; // declaração dos pinos da ponte h esquerda
int right_h[6] = {10,11,12,13,8,9}; // declaração dos pinos da ponte h direita


#define time_RGB_sensor 10


#define DISTANCE_OBSTACLE 15 //constante utilizada para detecar o objeto a uma distancia de 15 centimetros

//configuração do hardware do display oled
#define OLED_RESET -1 
#define OLED_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

//================ Declaração de Objetos ==================
Mecanum robot(left_h,right_h); //iniciação da biblioteca de controle dos motores
RGBsensor leftRGBSensor; // iniciação do sensor de cor rgb esquerdo
RGBsensor rightRGBSensor; // iniciação do sensor de cor rgb direito
Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X(); // iniciação do sensor de distancia a laser
Adafruit_SSD1306 display(OLED_WIDTH,OLED_HEIGHT,&Wire,OLED_RESET); // iniciação do display;
Servo servoLidar; // iniciação do servo motor utilizado no lidar
Button rightButton(30,INPUT_PULLUP); // inciação do botão direito da giga de teste
Button middleButton(32,INPUT_PULLUP); // iniciação do botão do meio da giga de teste
Button leftButton(34,INPUT_PULLUP); // iniciação do botão esquerdo da giga de teste

//================= Variáveis Globais ======================
bool start = false; // variavel responsavel por iniciar o robô
float distance = 0; // variavel responsavel por armazenar a distancia lida pelo sensor de distancia
int counter = 0; // variavel responsavel pela testagem dos motores

//================ Declaração de Funções ==================
void displayBegin(void); // realiza a configuração do display
void rgbSensorsBegin(void); // realiza a configuação dos sensores de cor
void starRobot (void); // função pra iniciar o robô no modo seguir linha
void readColors(void);
float readDistance (void); // função responsável por ler a distancia do sensor de distancia a laser
bool detectedObstacle(void); // função responsável por verificar se algum obstáculo foi detectado pelo sensor de distancia
void passObstacle(bool hasObtacle); //função responsável por realizar a ultrapassagem do obstaculo
void followLine(bool leftColor, bool rightColor); // função responsável por seguir linha;
void incrementCounter(void); //função para testar os motores do robô
void testingMotors(int counter = 0); //função para testar os motores do robô
void startControlBluetooth(void); //função para iniciar o robô no modo controle bluetooth
byte readSerial(void); // função responsavel por ler e amarzenar os dados da serial
void controlBluetooth (byte command = 'P'); // função responsável por realizar os comandos via bluetooth;


// Configuração dos pinos e inicalização dos comandos
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  displayBegin();
  rgbSensorsBegin();
  servoLidar.attach(25);
  if(!distanceSensor.begin()){
    Serial.println("Error distance sensor");  
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  rightButton.pressed(startRobot);
  middleButton.pressed(incrementCounter);
  leftButton.pressed(startControlBluetooth);
  while(start){
    while(!detectedObstacle()){
      followLine(leftRGBSensor.isBlack(), rightRGBSensor.isBlack());  
    }
    passObstacle(detectedObstacle()); 
  }
}

//================= Criação das funções ==============
void displayBegin(){
  display.begin(SSD1306_SWITCHCAPVCC,OLED_ADDRESS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,5);
  display.println(("Starting the program"));
  display.display();
  delay(1500);
  display.clearDisplay();
}

void rgbSensorsBegin(void){
  //sensor esquerdo
  leftRGBSensor.setLDRpin(A0);
  leftRGBSensor.setRGBpins(40,42,44);
  leftRGBSensor.setBlackPercentage(1);
  leftRGBSensor.setHighTime(time_RGB_sensor);
  leftRGBSensor.setLowTime(time_RGB_sensor - 10);
  leftRGBSensor.setPins();
  //sensor direito
  rightRGBSensor.setLDRpin(A1);
  rightRGBSensor.setRGBpins(46,48,50);
  rightRGBSensor.setBlackPercentage(1);
  rightRGBSensor.setHighTime(time_RGB_sensor);
  rightRGBSensor.setLowTime(time_RGB_sensor - 10);
  rightRGBSensor.setPins();

  rightRGBSensor.setBlank();
  leftRGBSensor.setBlank();
  
}

void startRobot(void){
  start = !start;  
}

void readColors(){
  leftRGBSensor.readColor();
  rightRGBSensor.readColor();  
}

float readDistance(void){
  float distance = 0;
  VL53L0X_RangingMeasurementData_t measure;
  distanceSensor.rangingTest(&measure,false);
  if(measure.RangeStatus != 4){
    distance = measure.RangeMilliMeter/10;  
  }
  else{
    distance = 200;  
  }

  return distance;
}

bool detectedObstacle(void){
  bool hasObstacle = readDistance() <= DISTANCE_OBSTACLE ? true : false ;
  return hasObstacle;
}

void passObstacle(bool hasObstacle){
  int timePassObstacle = 1000;
  int adjustmentTime = 1500;
  if(hasObstacle){
    robot.left();
    delay(timePassObstacle);
    robot.forward();
    delay(timePassObstacle + adjustmentTime);
    robot.right();
    delay(timePassObstacle);
  }  
}

void followLine(bool leftColor, bool rightColor){
  //frente
  if((!leftColor) && (!rightColor)){
      robot.forward();
  }
  //direita
  else if((!leftColor) && (rightColor)){
    robot.spinCentral_cw();
  }
  //esquerda
  else if((leftColor) && (!rightColor)){
    robot.spinCentral_acw();
  }
  //somente segue em frente
  else {
    robot.forward();  
  }
}

void incrementCounter(void){
  counter++;
  testingMotors(counter);
}

void testingMotors(int counter){
  switch(counter){
    case 1:
      robot.clockwise_m1();
      break;

    case 2:
      robot.antiClockwise_m1();
      break;
      
    case 3:
      robot.clockwise_m2();
      break;

    case 4:
      robot.antiClockwise_m2();
      break;

    case 5:
      robot.clockwise_m3();
      break;

    case 6:
      robot.antiClockwise_m3(); 
      break;

    case 7:
      robot.clockwise_m4();
      break;

    case 8:
      robot.antiClockwise_m4();
      break;
      
  default:
    break;
  }
}

void startControlBluetooth(void){
  while(true){
    controlBluetooth(readSerial());
  }  
}

byte readSerial(void){
  byte command = 'P';
  if(Serial.available() > 0){
    command = Serial.read();
  }
  return command;
}

void controlBluetooth(byte command){
  switch(command){
    case 'W':
      robot.forward();
      break;

    case 'A':
      robot.left();
      break;

    case 'S':
      robot.back();
      break;

    case 'D':
      robot.right();
      break;

    case 'P':
      robot.stop();
   default:
    break;  
  }  

}
