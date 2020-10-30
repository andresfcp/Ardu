/*
 * ROBO-tico Mark 11
 * HC-SR04
 * mejoras: - se incluye el sensor cuadruple de IR
*/

#include <Servo.h>   //Libreria para manejar el servo. Es una libreria estandard
#include <NewPing.h> //Librería para el manejo del HC-SR04. Buscarla ya gregarla
                     //https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

//pines para controlar el puente h L298
const int MotorIzquierdoAdelante = 5; //In4 del L298
const int MotorIzquierdoReversa = 4;  //In3 del L298
const int MotorDerechoAdelante = 3;   //In2 del L298
const int MotorDerechoReversa = 2;    //In1 del L298

//Pines del sensor HC-SR04
#define trigPin A1
#define echoPin A2
// Pines del sensor cuadruple IR
#define PinIzquierdo 6
#define PinCentroIzq 7
#define PinCentroDer 8
#define PinDerecho 9

#define servoPin 11 // Pin del servo

#define distanciaMaxima 200 //distancia máxima a la que va a reivsar el sensor

int izq;
int der;
int cizq;
int cder;
int obstaculo;

boolean goesForward = false;
int distancia = 100; //Variable con la que medimos los objetos delante de ROBO-tico

int tiempoDeGiro = 800; // Le puse esta variable para controlar mejor el tiempo de giro

NewPing sonar(trigPin, echoPin, distanciaMaxima); //declaracion del sensor hc-04
Servo servo_motor;                                //declaración del servo

void setup()
{
  pinMode(MotorDerechoAdelante, OUTPUT);
  pinMode(MotorIzquierdoAdelante, OUTPUT);
  pinMode(MotorIzquierdoReversa, OUTPUT);
  pinMode(MotorDerechoReversa, OUTPUT);

  pinMode(PinIzquierdo, INPUT);
  pinMode(PinDerecho, INPUT);
  pinMode(PinCentroIzq, INPUT);
  pinMode(PinCentroDer, INPUT);

  servo_motor.attach(servoPin);

  servo_motor.write(90);
  delay(2000);
  distancia = leerPing();
  delay(100);
  distancia = leerPing();
  delay(100);
  distancia = leerPing();
  delay(100);
  distancia = leerPing();
  delay(100);
}

void loop()
{

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distancia <= 20 || obstaculo == 1) //incluiremos la obstaculo=1
  {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    /* tiene preferencia girar a la derecha ya que no compara donde hay mas espacio *
     * hacer una compracion entre distanceRight y distanceLeft para definir a que  *
     * lado debe girar.
     * 
     * if(distanceRight > distanceLeft ){}
        */
    if (distanceRight > distanceLeft)
    {
      turnRight();
      moveStop();
    }
    else
    {
      turnLeft();
      moveStop();
    }
  }
  else
  {
    moveForward();
  }
  distancia = leerPing(); //Medimos la distacia
  obstaculo = leerIR();   //revisamos obstaculos por IR
}

int lookRight()
{
  servo_motor.write(10);
  delay(500);
  int distancia = leerPing();
  delay(100);
  servo_motor.write(90);
  return distancia;
}

int lookLeft()
{
  servo_motor.write(170);
  delay(500);
  int distancia = leerPing();
  delay(100);
  servo_motor.write(90);
  return distancia;
  delay(100);
}

int leerPing()
{
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250; //por qué 250???
  }
  return cm;
}

int leerIR()
{
  int obst;
  izq = digitalRead(PinIzquierdo);
  der = digitalRead(PinDerecho);
  cizq = digitalRead(PinCentroIzq);
  cder = digitalRead(PinCentroDer);

  if (izq == 0 || der == 0 || cizq == 0 || cder == 0)
  {
    obst = 1;
    return obst;
  }
  else
  {
    obst = 0;
    return obst;
  }
}

void moveStop()
{

  digitalWrite(MotorDerechoAdelante, LOW);
  digitalWrite(MotorIzquierdoAdelante, LOW);
  digitalWrite(MotorDerechoReversa, LOW);
  digitalWrite(MotorIzquierdoReversa, LOW);
}

void moveForward()
{

  if (!goesForward)
  {

    goesForward = true;

    digitalWrite(MotorIzquierdoAdelante, HIGH);
    digitalWrite(MotorDerechoAdelante, HIGH);

    digitalWrite(MotorIzquierdoReversa, LOW);
    digitalWrite(MotorDerechoReversa, LOW);
  }
}

void moveBackward()
{

  goesForward = false;

  digitalWrite(MotorIzquierdoReversa, HIGH);
  digitalWrite(MotorDerechoReversa, HIGH);

  digitalWrite(MotorIzquierdoAdelante, LOW);
  digitalWrite(MotorDerechoAdelante, LOW);
}

void turnRight()
{

  digitalWrite(MotorIzquierdoAdelante, HIGH);
  digitalWrite(MotorDerechoReversa, HIGH);

  digitalWrite(MotorIzquierdoReversa, LOW);
  digitalWrite(MotorDerechoAdelante, LOW);

  delay(tiempoDeGiro);

  digitalWrite(MotorIzquierdoAdelante, HIGH);
  digitalWrite(MotorDerechoAdelante, HIGH);

  digitalWrite(MotorIzquierdoReversa, LOW);
  digitalWrite(MotorDerechoReversa, LOW);
}

void turnLeft()
{

  digitalWrite(MotorIzquierdoReversa, HIGH);
  digitalWrite(MotorDerechoAdelante, HIGH);

  digitalWrite(MotorIzquierdoAdelante, LOW);
  digitalWrite(MotorDerechoReversa, LOW);

  delay(tiempoDeGiro);

  digitalWrite(MotorIzquierdoAdelante, HIGH);
  digitalWrite(MotorDerechoAdelante, HIGH);

  digitalWrite(MotorIzquierdoReversa, LOW);
  digitalWrite(MotorDerechoReversa, LOW);
}
