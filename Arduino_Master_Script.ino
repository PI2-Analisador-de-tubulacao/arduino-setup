//Bibliotecas Auxiliares

//Biblioteca usada no controle do motor de passo
#include <AFMotor.h> 

//Biblioteca usada no controle dos servomotores
#include <Servo.h>

//Bibliotecas usada no controle dos motores, uso dos sensores e mapeamento em 2D
#include <Wire.h>
#include <HMC5883L.h>
#include <NewPing.h>

-----------------------------------------------------------------------------
// Controlando motor de passo 5V 28BYJ-48 com Arduino e o driver L293D
//referencia: FILIPEFLOP
 
//Quantidade de passos na rotacao 1
double passos_total = 2048;
//Para motor em M1/M2 e 2 para motor em M3/M4 
int porta_motor = 2;
//Angulo de rotacao do eixo 
int angulo = 30;  
//Armazena a quantidade de passos a qual o eixo do motor girará
double numero_de_passos = 0; 
//Estabelece parametros do motor 
AF_Stepper arduino(passos_total, porta_motor); 
 
void setup()
{
//Estabelece a velocidade de rotação
arduino.setSpeed(10); 
Serial.begin(9600);
}
 
void loop()
{
//Calcula a quantidade de passos, conforme o angulo 
numero_de_passos = angulo / (360 / passos_total);
 
//Imprime no serial monitor a quantidade de passos calculados
Serial.println(numero_de_passos);
 
//Acinonar o motor --- Rotação
//FORWARD --- sentido horario, e BACKWARD --- anti-horario
arduino.step(numero_de_passos, FORWARD, SINGLE);
arduino.release();
 
delay(2000);
}
-----------------------------------------------------------------------------
//Controlando os servomotores com Arduino e o driver L293D
// referencoa: https://create.arduino.cc/projecthub/Fouad_Roboticist/arduino-servo-motor-control-with-motor-driver-shield-l293d-f60a7c 
//Mapa de Hardware
#define servo1 9 //servo1 ligado ao pino 9
#define servo2 10 //servo2 ligado ao pino 10
 
// --- Objetos do Software ---
Servo sv1, sv2; //cria objetos para os servos
 
 
//Variáveis Globais
int servo_position = 0 ;
unsigned short angulo1 = 90, angulo2 = 90; //angulo dos servos
unsigned short anguloAtual1, anguloAtual2; //auxiliares para armazenar angulo atual
  
// --- Configurações Iniciais ---
void setup()
{
 pinMode(servo1, OUTPUT); //saída servo1
 pinMode(servo2, OUTPUT); //saída servo2
 
 sv1.attach(servo1); //pino de sv1
 sv2.attach(servo2); //pino de sv2
 
 sv1.write(angulo1); //centraliza sv1
 sv2.write(angulo2); //centraliza sv2
 
 
 
} //end setup
 
// --- Loop Infinito ---
void loop()
{
 
 servoControl(); //chama função para controle do servo

 for (servo_position = 0; servo_position <=180; servo_position +=1){

    name_servo.write(servo_position);
    delay(10);
 
 for (servo_position=180; servo_position >= 0; servo_position -=1){

    name_servo.write(servo_position);
    delay(10);
 
 } //end loop
 
 
// --- Funções Auxiliares ---
void servoControl()
{
 
 sv1.write(angulo1); //atualiza angulo do servo1
 sv2.write(angulo2); //atualiza angulo do servo2
 
} //end servoControl

-----------------------------------------------------------------------------
//Controlando os LEDs de alta Potência
// https://wiki.ifsc.edu.br/mediawiki/index.php/AULA_7_-_Microcontroladores_-_Eng

// Acionando os LEDs de alta Potencia usando o L2935D

 #define ENABLE 7
 #define LEDs 8
 
 int i;
 
 void setup() {
  pinMode(ENABLE,OUTPUT);
  pinMode(LEDs,OUTPUT);
  Serial.begin(9600);
 }
 
 void loop() {
 //---acionando os Leds
  Serial.println("Ligar Leds");
  digitalWrite(ENABLE,HIGH); // enable on
  for (i=0;i<5;i++) {
    digitalWrite(LEDs,HIGH); //Ligado
    delay(500);
  }
  digitalWrite(ENABLE,LOW); // desliga os Leds
  delay(2000);
  
  Serial.println("PWM para intensidade de iluminação");
  //---exemplo PWM, intensidade máxima, depois mínima
  analogWrite(ENABLE,255); //enable on
  digitalWrite(LEDs,HIGH); //
  delay(2000);
  analogWrite(ENABLE,180); //intensidade alta
  delay(2000);
  analogWrite(ENABLE,128); //intensidade média
  delay(2000);
  analogWrite(ENABLE,50); //intensidade baixa
  delay(2000);
  analogWrite(ENABLE,128); //intensidade média
  delay(2000);
  analogWrite(ENABLE,180); //intensidade alta
  delay(2000);
  analogWrite(ENABLE,255); //intensidade máxima
  delay(2000);
  digitalWrite(ENABLE,LOW); //desligar
  delay(10000);
}
-----------------------------------------------------------------------------

// Controlando os dois motores por meio da L293D
//Traçando um mapa 2D do percurso --- sensores HCSR04 e Seeed Digital Compass Grove
//Referência: https://create.arduino.cc/projecthub/avirup_basu/autonomous-navigation-and-2d-mapping-0f91d3
//GIT Referência: https://github.com/avirup171/Genesis-App-for-Autonomoua-Navigation-and-2D-mapping

NewPing sonar1(A0, A1, 100);
NewPing sonar2(A2, A3, 100);
NewPing sonar3(A4, A5, 100);

HMC5883L compass; //porta digital 3 altererada para digital 10 (verificar biblioteca HMC5883L) 

//pinos dos Motores
int d1=8; 
int  val2=82,val=90;
int d2=9;
int brake1=22; //porta alterada de 4 para 22
int brake2=12;
int op1=23; //porta alterada de 5 para 23
int op2=7;
int pwm1=6;
int pwm2=11;
int td=35;

int error = 0;
String s="",s1,s2,s3;
//codificação
/* 
 *  0   -> N
 *  90  -> E
 *  180 -> S
 *  270 -> W
 */
int fin_heading=0, curr_heading;
int m11=3,m12=4,m21=5,m22=6;
//Variaveis Globais
int t=2,e=7; //Controle dos pinos do sensor HCSR04
int d,dt; //Distancia calculada
//int servoPin=9;
int trans[1000], count =0;
int dis1,dis2,dis3;
//Distancia obtida
void distance()
{
  dis1=sonar1.ping_cm();
  dis2=sonar2.ping_cm();
  dis3=sonar3.ping_cm();
  
}
void heading()
{
  MagnetometerRaw raw = compass.readRawAxis();
  MagnetometerScaled scaled = compass.readScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = -0.0457;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  // Verificar em volta devido a adição de declinação.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Converter radianos em graus para facilitar a leitura.
  float headingDegrees = heading * 180/M_PI;
  curr_heading=(int)headingDegrees;
  delay(66); 
}

void setup()
{
  Serial.begin(9600);
  pinMode(d1,OUTPUT);
  pinMode(d2,OUTPUT);
  pinMode(brake1,OUTPUT);
  pinMode(brake2,OUTPUT);
  pinMode(op1,OUTPUT);
  pinMode(op2,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  Wire.begin();
  error = compass.setScale(1.3); 
  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS); 
}

void loop() 
{
  String sc="+";
  heading();
  distance();
  s1=String(dis1);
  s2=String(dis2);
  s3=String(dis3);
  String head=String((int)curr_heading);
  //String head="360";
  s=sc+s1+sc+s2+sc+s3+sc+head+"arg";
  Serial.print(s);
  Serial.println();
  //Algoritmo
  char cd='f';
  cd= dist_empty();
  switch(cd)
  {
    
    case 'l':
    left();
    delay(420);
    stoop();
    delay(100);
    break;
    case 'r': right();
    delay(420);
    stoop();
    delay(100);
    break;
    case 'f': forward();
    delay(150);
    stoop();
    delay(100);
    break;
    default: stoop();
  }
  Serial.println(cd);
  delay(100);
}
char dist_empty()
{
  //Seguir RHR
  td=18;
  if(dis1<td&&dis1!=0)
  {
    if(dis2>td||dis2==0)
    return 'l';
    else if(dis3>td||dis3==0)
    return 'r';
    else
    return 's';
  }
  else
  {
    /*if(dis3<=10)
    return 'l';//a=l
    else if(dis2<=10)
    return 'r';//b=r
    else*/
    return 'f';
  }
}
void left()
{
  digitalWrite(brake2,LOW);
      analogWrite(pwm2,val);
      digitalWrite(d1,HIGH);
      analogWrite(pwm1,val);
      digitalWrite(d2,LOW);
      digitalWrite(brake1,LOW);
      digitalWrite(op1,LOW);
      digitalWrite(op2,LOW);
}
void right()
{
 digitalWrite(brake2,LOW);
      analogWrite(pwm2,val);
      digitalWrite(d1,LOW);
      analogWrite(pwm1,val);
      digitalWrite(d2,HIGH);
      digitalWrite(brake1,LOW);
      digitalWrite(op1,LOW);
      digitalWrite(op2,LOW);
}
void forward()
{
  digitalWrite(d1,LOW);
      analogWrite(pwm1,val);
      digitalWrite(brake2,LOW);
      analogWrite(pwm2,val2);
      digitalWrite(d2,LOW);
      digitalWrite(brake1,LOW);
      digitalWrite(op1,LOW);
      digitalWrite(op2,LOW);  
}
void stoop()
{
   digitalWrite(brake2,HIGH);
      digitalWrite(d1,LOW);
      digitalWrite(d2,LOW);
      digitalWrite(brake1,HIGH);
      digitalWrite(op1,LOW);
      digitalWrite(op2,LOW);
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
}


