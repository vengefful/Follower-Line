/*
 * File name: PID_LF_example
 * 
 * Hardware requirements: an Arduino Pro Mini
 *                        a QTR-8RC Reflectance Sensor Array
 *                        a DRV8835 Dual Motor Driver Carrier 
 *                        
 * Description: The basic PID control system implemented with 
 *              the line follower with the specified hardware. 
 *              The robot can follow a black line on a white surface 
 *              (or vice versa). 
 * Related Document: See the written documentation or the LF video from
 *                   Bot Reboot.
 *                   
 * Author: Bot Reboot
 */

#include <QTRSensors.h> //Make sure to install the library
#include <NewPing.h>

/*************************************************************************
* Sensor Array object initialisation 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.025; //related to the proportional control term;     //0.015 0.025
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0;//related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;
                                  //0.015 ; 0; 0 ; 80; 50
int motormaxspeed = 80; //80
int motorbasespeed = 50; //50

long prevTA = 0;
long prevTB = 0;
float eprevA = 0;
float eprevB = 0;
float eintegralA = 0;
float eintegralB = 0;

/*************************************************************************
* Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = true;
int changeDirection = 0;
int lastChangeDirection = 0;
int posA = 0;
int posB = 0;
int dirA = 1;
int dirB = 1;
int pos = 0;

int volta = 810;
float raio_roda = 3.4;

int und = 38; //1 cm para 38 unidades
int giro_360 = 4502; //4582

int threshold = 780;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 80;
const uint8_t maxspeedb = 80;
const uint8_t basespeeda = 50;
const uint8_t basespeedb = 50;

/*************************************************************************
* DRV8835 GPIO pins declaration
*************************************************************************/
int mode = 8;
int aphase = 13;
int aenbl = 11;
int bphase = 12;
int benbl = 10;

/*************************************************************************
* Encoder GPIO pins declaration
*************************************************************************/
int enca = 2;
int encb = 3;

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = 17; //or pin A3
int buttonstart = 2;
int buzzer = 4;

int trigger = 9;
int echo = 8;
int max_distance = 200;

int button = 6;
byte lastButtonState = LOW;
unsigned long debounceDuration = 1000;
unsigned long lastTimeButtonStateChanged = 0;

NewPing sonar(trigger, echo, max_distance);

/*************************************************************************
* Function Name: setup
**************************************************************************
* Summary:
* This is the setup function for the Arduino board. It first sets up the 
* pins for the sensor array and the motor driver. Then the user needs to 
* slide the sensors across the line for 10 seconds as they need to be 
* calibrated. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(7);//LEDON PIN

  // pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  // digitalWrite(mode, HIGH); //one of the two control interfaces 
                            //(simplified drive/brake operation)
  delay(500);
  pinMode(buzzer, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  //calibrating
  delay(2000);
  calibration();
  delay(5000);

  //starting encoders
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  attachInterrupt(digitalPinToInterrupt(enca), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encb), readEncoderB, RISING);
  
  //Starting Serial Monitor
  Serial.begin(9600);

  forward_brake(0, 0); //stop the motors
}

/*************************************************************************
* Function Name: calibration
**************************************************************************
* Summary:
* This is the calibration function for the QTR-8RC Reflectance Sensor Array. 
* The function calls the method 'qtr.calibrate()' offered by the imported 
* library. For approx. 10 seconds, each of the 8 sensors will calibrate with
* readings from the track. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void calibration() {
  digitalWrite(buzzer, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(buzzer, LOW);
}

/*************************************************************************
* Function Name: readEncoderA
**************************************************************************
* Summary:
* This is the calibration function for the QTR-8RC Reflectance Sensor Array. 
* The function calls the method 'qtr.calibrate()' offered by the imported 
* library. For approx. 10 seconds, each of the 8 sensors will calibrate with
* readings from the track. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void readEncoderA(){
  if(dirA>0){
    posA++;
  }
  else{
    posA--;
  }
}

/*************************************************************************
* Function Name: readEncoderB
**************************************************************************
* Summary:
* This is the calibration function for the QTR-8RC Reflectance Sensor Array. 
* The function calls the method 'qtr.calibrate()' offered by the imported 
* library. For approx. 10 seconds, each of the 8 sensors will calibrate with
* readings from the track. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void readEncoderB(){
  if(dirB>0){
    posB++;
  }
  else{
    posB--;
  }
}

/*************************************************************************
* Function Name: loop
**************************************************************************
* Summary:
* This is the main function of this application. When the start button is
* pressed, the robot will toggle between following the track and stopping.
* When following the track, the function calls the PID control method. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void loop() {
  //int botao = digitalRead(button);
  //if(onoff == false && botao == 0){
    //onoff = true;
    //Serial.println("Iniciando");
    //delay(3000);
  //}
  uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
  print_sensors(position);
  if (onoff == false) {

    PID_control_v2(position);
    //PID_control(0,giro_360/4);
    //print_sensors(position);

    //avoid_obstacle();
    //centralizar(position);

    unsigned int distance = sonar.ping_cm();
    //Serial.println(distance);

    if(distance > 0 && distance < 14){
      set_motor(dirA, 0, dirB, 0);
      delay(5000);
      avoid_obstacle();
    }

    if(turn_90(position) > 0){
      Serial.println("90 graus");
      make_turn_90(1);
      //uint16_t position = qtr.readLineBlack(sensorValues);
      //print_sensors(position);
      delay(5000);
    }

    if(gap(position) == true){
      set_motor(1, 0, 1, 0);
      Serial.println(position);
      delay(10000);
      make_gap();
    }

    //if(sharp_turn(position) == true){
     // set_motor(1, 0, 1, 0);
    //  Serial.println(position);
    //  Serial.println("Sharp Turn");
    //  delay(5000);
    //  make_sharp_turn();
    //}
  }
  else {
    forward_brake(0,0); //stop the motors
  }
}

int turn_90(uint16_t position){
  if(sensorValues[0] > threshold && sensorValues[1] > threshold){
    //if(sensorValues[2] > threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold){
     // return 1;
    //}
    if(sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] < threshold && sensorValues[5] < threshold){
      return 1;
    }
    else if(sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] < threshold){
      return 1;
    }  
  }
  return 0;
}

void make_turn_90(int direction){
  int lastPosA = 0;
  int lastPosB = 0;
  //FRENTE
  set_motor(dirA, 0, dirB, 0);
  posA = 0;
  posB = 0;
  delay(3000);
  int frente = und * 9.7; //8.7
  while(posA < frente || posB < frente){
    PID_control(frente,frente);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(frente);
  }
  //GIRO DE 90 GRAUS
  Serial.println("Inicio giro");
  set_motor(dirA, 0, dirB, 0);
  delay(3000);
  posA = 0;
  posB = 0;
  int giro = giro_360 * 0.25;
  while(posB < giro){
    PID_control(0,giro);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
    
  }
  //PARA TRAS
  set_motor(dirA, 0, dirB, 0);
  delay(3000);
  posA = 0;
  posB = 0;
  delay(3000);
  int tras = -1 * und * 23;
  while(fabs(posA) < fabs(tras) || fabs(posB) < fabs(tras)){
    PID_control(tras,tras);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(tras);
  }
  //reseting variables
  Serial.println("Fim");
  set_motor(dirA, 0, dirB, 0);
  delay(3000);
  posA = 0;
  posB = 0;
  prevTA = 0;
  prevTB = 0;
  eprevA = 0;
  eprevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  dirA = 1;
  dirB = 1;
}

void avoid_obstacle(){
  posA = 0;
  posB = 0;
  int giro = giro_360 / 4;
  while(posB < giro){
    PID_control(0,giro);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
    
  }
  //PARA frente
  set_motor(dirA, 0, dirB, 0);
  delay(3000);
  posA = 0;
  posB = 0;
  int frente = und * 2 ;
  while(posA < frente || posB < frente){
    PID_control(frente, frente);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  giro = giro_360 / 4;
  while(posA < giro){
    PID_control(giro,0);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
    
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  frente = und * 22;
  while(posA < frente || posB < frente){
    PID_control(frente, frente);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  giro = giro_360 / 4;
  while(posA < giro){
    PID_control(giro,0);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
    
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  frente = und * 4 ;
  while(posA < frente || posB < frente){
    PID_control(frente, frente);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  giro = giro_360 / 4;
  while(posB < giro){
    PID_control(0,giro);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
    
  }
  //PARA frente
  set_motor(dirA, 0, dirB, 0);
  delay(3000);
  posA = 0;
  posB = 0;
  delay(3000);
  int tras = -1 * und * 15;
  while(fabs(posA) < fabs(tras) || fabs(posB) < fabs(tras)){
    PID_control(tras,tras);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(tras);
  }
  //reseting variables
  Serial.println("Fim");
  set_motor(dirA, 0, dirB, 0);
  delay(10000);
  posA = 0;
  posB = 0;
  prevTA = 0;
  prevTB = 0;
  eprevA = 0;
  eprevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  dirA = 1;
  dirB = 1;
}

bool sharp_turn(uint16_t position){
  int threshold = 780;
  if(sensorValues[0] > threshold){
      return true;
    }
    return false;
}

void make_sharp_turn(){
  posA = 0;
  posB = 0;
  int giro = -1 * giro_360 / 36;
  while(fabs(posA) < fabs(giro)){
    PID_control(giro,0);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(giro);
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  prevTA = 0;
  prevTB = 0;
  eprevA = 0;
  eprevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  dirA = 1;
  dirB = 1;
}

bool gap(uint16_t position){
  int threshold = 700;
  if(sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold){
      return true;
    }
    return false;
}

void make_gap(){
  posA = 0;
  posB = 0;
  int frente = und * 10 ;
  while(posA < frente || posB < frente){
    PID_control(frente, frente);
    Serial.print(posA);
    Serial.print('\t');
    Serial.print(posB);
    Serial.print('\t');
    Serial.println(frente);
  }
  set_motor(dirA, 0, dirB, 0);
  delay(5000);
  posA = 0;
  posB = 0;
  prevTA = 0;
  prevTB = 0;
  eprevA = 0;
  eprevB = 0;
  eintegralA = 0;
  eintegralB = 0;
  dirA = 1;
  dirB = 1;
}

void centralizar(uint16_t position){
  posA = 0;
  posB = 0;
  uint16_t positionTmp = qtr.readLineBlack(sensorValues);
  if(position < 2500){
    while(positionTmp < 2500){
      PID_control_v2(positionTmp);
      positionTmp = qtr.readLineBlack(sensorValues);
      print_sensors(positionTmp);
    }
  }
  else if(position > 2500){
    while(positionTmp < 2500){
      PID_control_v2(positionTmp);
      positionTmp = qtr.readLineBlack(sensorValues);
      print_sensors(positionTmp);
    }
  }
  set_motor(dirA, 0, dirB, 0);
  delay(10000);
  posA = 0;
  posB = 0;
}

/*************************************************************************
* Function Name: forward_brake
**************************************************************************
* Summary:
* This is the control interface function of the motor driver. As shown in
* the Pololu's documentation of the DRV8835 motor driver, when the MODE is 
* equal to 1 (the pin is set to output HIGH), the robot will go forward at
* the given speed specified by the parameters. The phase pins control the
* direction of the spin, and the enbl pins control the speed of the motor.
* 
* A warning though, depending on the wiring, you might need to change the 
* aphase and bphase from LOW to HIGH, in order for the robot to spin forward. 
* 
* Parameters:
*  int posa: int value from 0 to 255; controls the speed of the motor A.
*  int posb: int value from 0 to 255; controls the speed of the motor B.
* 
* Returns:
*  none
*************************************************************************/
void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(aphase, HIGH);
  digitalWrite(bphase, HIGH);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

/*************************************************************************
* Function Name: Print_Sensors and Positions
**************************************************************************/
void print_sensors(uint16_t position){
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}

/*************************************************************************
* Function Name: set_motor
**************************************************************************/
void set_motor(int dir, int pwmVal, int dir2, int pwmVal2){
  if(dir == 1){
    digitalWrite(aphase, HIGH);
  }
  else if(dir == -1){
    digitalWrite(aphase, LOW);
  }
  analogWrite(aenbl, pwmVal);

  if(dir2 == 1){
    digitalWrite(bphase, HIGH);
  }
  else if(dir2 == -1){
    digitalWrite(bphase, LOW);
  }
  analogWrite(benbl, pwmVal2);
}

/*************************************************************************
* Function Name: PID_control_v2
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
void PID_control_v2(uint16_t position) {
  int target = 2500 - position;
  int targetA = target; //set target position

  //time difference
  long currTA = micros();

  float deltaTA = ((float)(currTA-prevTA))/1.0e6;
  
  prevTA = currTA;

  //error
  int eA = posA - targetA;
  

  //derivative
  float dedtA = (eA - eprevA)/(deltaTA);
  

  //integral
  eintegralA = eintegralA + eA*deltaTA;
  

  //control signal
  float pwr = Kp*eA + Kd*dedtA + Ki*eintegralA;

  float motorspeeda = motorbasespeed + pwr;
  float motorspeedb = motorbasespeed - pwr;
  
  //Serial.print('\t');
  //Serial.print(pwr);

  //motor power
  //float pwrA = fabs(uA);
  if(motorspeeda > motormaxspeed){
    motorspeeda = motormaxspeed;
  }
  else if (motorspeeda < 0){
    motorspeeda = 0;
  }

  if(motorspeedb > motormaxspeed){
    motorspeedb = motormaxspeed;
  }
  else if (motorspeedb < 0){
    motorspeedb = 0;
  }
  
  //set_motor(dirA, motorspeeda, dirB, motorspeedb);
  //set_motor(1, motorspeeda, 1, motorspeedb);
  
  Serial.print(position);
  Serial.print('\t');
  Serial.print(motorspeeda);
  Serial.print('\t');
  Serial.println(motorspeedb);
  
  

  //store previous error
  eprevA = eA;
  //eprevB = eB;

  //reset positions
  posA = 0;
  posB = 0;

  
}

/*************************************************************************
* Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
void PID_control(int positionA, int positionB) {
  int targetA = positionA; //set target position
  int targetB = positionB; //set target position

  float kp = 0.015;
  float kd = 0;
  float ki = 0;

  //time difference
  long currTA = micros();

  float deltaTA = ((float)(currTA-prevTA))/1.0e6;
  

  //error
  int eA = posA - targetA;
  

  //derivative
  float dedtA = (eA - eprevA)/(deltaTA);
  

  //integral
  eintegralA = eintegralA + eA*deltaTA;
  

  //control signal
  float uA = kp*eA + kd*dedtA + ki*eintegralA;
  

  //motor power
  float pwrA = fabs(uA);
  if(pwrA > 255){
    pwrA = 255;
  }
  if(pwrA < 50 && pwrA > 0){
    pwrA = 50;
  }
  

  //motor direction
  dirA = 1;
  if(uA > 0){
    dirA = -1;
  }
  

  long currTB = micros();
  float deltaTB = ((float)(currTB-prevTB))/1.0e6;
  prevTB = currTB;
  int eB = posB - targetB;
  float dedtB = (eB - eprevB)/(deltaTB);
  eintegralB = eintegralB + eB*deltaTB;
  float uB = kp*eB + kd*dedtB + ki*eintegralB;
  float pwrB = fabs(uB);
  if(pwrB > 255){
    pwrB = 255;
  }
  if(pwrB < 50 && pwrB > 0){
    pwrB = 50;
  }
  dirB = 1;
  if(uB > 0){
    dirB = -1;
  }

  //signal de motor
  //set_motor(dirA, pwrA, dirB, pwrB);

  //store previous error
  eprevA = eA;
  eprevB = eB;

  //Serial.print(posA);
  //Serial.print('\t');
  //Serial.print(posB);
  //Serial.println();
}