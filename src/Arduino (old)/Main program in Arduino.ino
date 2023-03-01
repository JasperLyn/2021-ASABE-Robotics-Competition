/*************************************************************/
#include <Arduino.h>
#include <LiquidCrystal.h>//lcd
#include <SoftwareSerial.h>
#include <Servo.h>
#include <String.h>



Servo My_Servo; //use Servo to define a servo named My_Servo
#define Servo_1 2 //define the signal pin to servo as Arduino digital pin 2

#define L1_DIR 4           //define the DIR_signal_pin to the left stepper motor as Arduino digital pin 4
#define L1_EN 7            //define the en_signal_pin to the left stepper motor as Arduino digital pin 7
#define L1_STP 5           //define the STP_signal_pin to the left stepper motor as Arduino digital pin 5

#define R1_DIR 49          //define the DIR_signal_pin to the right stepper motor as Arduino digital pin 49
#define R1_EN 53           //define the EN_signal_pin to the right stepper motor as Arduino digital pin 53
#define R1_STP 51          //define the STP_signal_pin to the right stepper motor as Arduino digital pin 51

#define L_infraredray A6   //define the signal pin to the left infrared sensor as Arduino analog pin A6
#define R_infraredray A10  //define the signal pin to the right infrared sensor as Arduino analog pin A10

#define RaspberryPi 22     //define the signal pin to  Raspberry Pi as Arduino digital pin 22
#define raspberrypi 24     //define the signal pin to  Raspberry Pi as Arduino digital pin 24
#define RaspberryPiback 26     //define the signal pin to  Raspberry Pi as Arduino digital pin 26
#define raspberrypiback 28 //define the signal pin to  Raspberry Pi as Arduino digital pin 28

#define STOP 22     
/**************************参数定义***********************************/

volatile int b[2]; // array for tracking
int number=0;
/*************************************************************/

void Track_PWM(int Pin, int delay_us) {
digitalWrite(Pin, HIGH);
delayMicroseconds(delay_us);
digitalWrite(Pin, LOW);
delayMicroseconds(delay_us);
}

/*************************************************************/

void Track_Setup(){
pinMode(L1_DIR, OUTPUT);
pinMode(L1_EN, OUTPUT);
pinMode(L1_STP, OUTPUT);
pinMode(R1_DIR, OUTPUT);
pinMode(R1_EN, OUTPUT);
pinMode(R1_STP, OUTPUT);
digitalWrite(L1_EN, HIGH);          // put L1_EN HIGH，to activate the control of the left stepper motor
digitalWrite(R1_EN, HIGH);         // put R1_EN HIGH，to activate the control of the right stepper motor
}

void Stop()
{
  digitalWrite(L1_EN,LOW);          //put L1_EN LOW，to freeze the control of the left stepper motor
  digitalWrite(R1_EN,LOW);         //put R1_EN LOW，to freeze the control of the right stepper motor

  }

void GoStraight(int Step)
{
  digitalWrite(L1_EN, HIGH);
  digitalWrite(L1_DIR, LOW);

  digitalWrite(R1_EN, HIGH);
  digitalWrite(R1_DIR, HIGH);
  
  for (int x = 0; x < Step; x++)
  {
      Track_PWM(L1_STP, 80); 
      Track_PWM(R1_STP, 80);
  }
}

void TrackStraightRight(int Step){
  digitalWrite(L1_EN, HIGH);
  digitalWrite(L1_DIR, LOW);

  digitalWrite(R1_EN, HIGH);
  digitalWrite(R1_DIR, HIGH);

  for (int xlyy = 0; xlyy < Step; xlyy++)
  {
    digitalWrite(R1_STP, HIGH);
    digitalWrite(L1_STP, HIGH);
    delayMicroseconds(200);
    digitalWrite(L1_STP, LOW);
    delayMicroseconds(200);
    digitalWrite(L1_STP, HIGH);
    digitalWrite(R1_STP, LOW);
    delayMicroseconds(200);
    digitalWrite(L1_STP, LOW);
    delayMicroseconds(200);
  }
  
}

void TrackStraightLeft(int Step)
{
  digitalWrite(L1_EN, HIGH);
  digitalWrite(L1_DIR, LOW);

  digitalWrite(R1_EN, HIGH);
  digitalWrite(R1_DIR, HIGH);
  for (int xLYY = 0; xLYY < Step; xLYY++)
  {
    digitalWrite(L1_STP, HIGH);
    digitalWrite(R1_STP, HIGH);
    delayMicroseconds(200);
    digitalWrite(R1_STP, LOW);
    delayMicroseconds(200);
    digitalWrite(R1_STP, HIGH);
    digitalWrite(L1_STP, LOW);
    delayMicroseconds(200);
    digitalWrite(R1_STP, LOW);
    delayMicroseconds(200);
  }
}


void TrackStraight(int step){
  int lyy;
  for(lyy=0;lyy<step;lyy++){
    b[0]=digitalRead(L_infraredray);
    b[1]=digitalRead(R_infraredray);

    if(b[0]==1&& b[1]==1)            //Go straight
      {GoStraight(20);}
    else if(b[0]==1 && b[1]==0)   //Turn left with tiny angle until the robot is parallel to the venue
      {TrackStraightLeft(1);
      }
    else if(b[0] ==0 && b[1]==1)   //Turn right with tiny angle until the robot is parallel to the venue
       {TrackStraightRight(1);}
    else
       {GoStraight(20);}         //Go straight
  }
}


/*************************************************************/

void Servo_Setup(){
  pinMode(Servo_1, OUTPUT);
  My_Servo.attach(Servo_1);
  for(int servoi=122;servoi>=50;servoi=servoi-9){
    My_Servo.write(servoi);
    delay(20);
  }
  delay(200);
  My_Servo.detach();
}

void Servo_Setupfinal() {
  pinMode(Servo_1, OUTPUT);           // set Servo_1 pin as output mode to control the servo
  My_Servo.attach(Servo_1);              // attach the pin with the servo
  My_Servo.write(150);                      // write the angle with the first absolute angle 20 degree
  delay(500);                                     // maintain a liitle period of time to make sure the servo rotate correctly
  My_Servo.detach();                        //detach the pin with the servo to avoid bug
}

void sensorsetup(){
  pinMode(RaspberryPi,OUTPUT);
  pinMode(raspberrypi,OUTPUT);
  pinMode(RaspberryPiback,INPUT);
  pinMode(raspberrypiback,INPUT);
  pinMode(L_laser,INPUT);
  pinMode(R_laser,INPUT);
  pinMode(L_infraredray,INPUT);
  pinMode(R_infraredray,INPUT);
}
/**************************************************************/

void setup() {
    sensorsetup();
    Track_Setup();  
    Servo_Setupfinal();
}       //void setup END!!!

void loop() {
  TrackStraight(1);
      if(raspberrypiback==1&&RaspberryPiback==0){
        number++;
        while(1){
          digitalWrite(raspberrypi,LOW);
          if(raspberrypiback==0&&RaspberryPiback==1){
            break;
          }
        }     
      }
      
//      for(int i=0;i<3000;i++){
//         digitalWrite(L1_DIR, LOW); 
//         digitalWrite(R1_DIR, HIGH);
//         Track_PWM(L1_STP, 80); 
//         Track_PWM(R1_STP, 80);
//      }  
    
 
//    }
  
  if(number==12){
    for(int j=-3000;j<3000;j++){
         digitalWrite(L1_DIR, LOW); 
         digitalWrite(R1_DIR, HIGH);
         Track_PWM(L1_STP, 80); 
         Track_PWM(R1_STP, 80);
         }
      Servo_Setupfinal();



//         for(int final=0;final<2;final++){
//       for(int i=0;i<5000;i++){
//  digitalWrite(L1_DIR, HIGH);   //control the rotate orient,clockwise or unclockwise according to the adapt method of stepper motor on robot
//  digitalWrite(R1_DIR, HIGH);
//  Track_PWM(L1_STP, 50); //utilize Track_PWM() function to output PWM wave to the STP port of the stepper motor to make it work
//  Track_PWM(R1_STP, 50);// the value of delay_ms smaller, speed higher, but too fast may causing the friction between the robot and venue become too small and slippery
//    }
//    }
//    
//    Servo_Setupagain();
//    delay(1000);
//    Servo_Setupfinal();
//    delay(1000);
//    Servo_Setupagain();
//    delay(1000);
//    Servo_Setupfinal();
//    delay(1000);
   
       
    while(1){
         Stop();
   digitalWrite(Stop,LOW);
    }

    }
  }        // loop END!!!
