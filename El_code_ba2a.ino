#include <TimerOne.h> 
#include <SoftwareSerial.h> 
#include <Wire.h> 
#include "Arduino.h"
#include "MPU9250.h"
#include <MPU6050_tockn.h>
char data = 0; char BT_input; long timer = 0; const unsigned int MAX_DIST = 23200;

MPU6050 mpu6050(Wire); MPU9250 IMU (Wire, 0x68);

int distance1; int distance2; int distance3; int distance4; int distance5; int distance6; int time1; int time2; int time3; int time4; int time5; int time6; int status;

int LEDMotor1 = 18; int LEDMotor2 = 19; int LEDMotor3 = 22; int LEDBlue1 = 23; int LEDBlue2 = 24; int LEDMPU1 = 25; int LEDMPU2 = 38;

int LEDU1 = 39; int LEDU2 = 40; int LEDU3 = 41; int LEDU4 = 42; int LEDU5 = 43; int LEDU6 = 44;

int PWM1 = 9; int PWM2 = 10;

int direction1 = 4; int direction2 = 12; int direction3 = 3; int direction4 = 5; int direction5 = 0; int brake1 = 6; int brake2 = 7; int brake3 = 11; int brake4 = 13; int brake5 = 8; int brake6 = 1;

int trig_1 = 36; int trig_2 = 26; int trig_3 = 28; int trig_4 = 30; int trig_5 = 32; int trig_6 = 34; int echo_1 = 37; int echo_2 = 27; int echo_3 = 29; int echo_4 = 31; int echo_5 = 33; int echo_6 = 35;

SoftwareSerial BlueTooth (15, 14);

void setup()
{
  BlueTooth.begin(9600);
  Serial.begin(9600);

  pinMode ((LEDMotor1 && LEDMotor2 && LEDMotor3 && LEDBlue1 && LEDBlue2 && LEDMPU1 && LEDMPU2 && LEDU1 && LEDU2 && LEDU3 && LEDU4 && LEDU5 && LEDU6), OUTPUT);
  pinMode ((trig_1 && trig_2 && trig_3 && trig_4 && trig_5 && trig_6 && echo_1 && echo_2 && echo_3 && echo_4 && echo_5 && echo_6), OUTPUT);
  pinMode ((direction1 && direction2 && direction3 && direction4 && direction5 && brake1 && brake2 && brake3 && brake4 && brake5 && brake6), OUTPUT);
  digitalWrite ((PWM1 && PWM2), 0);

  while(!Serial){}
  status = IMU.begin();

  if (status<0){
    
    Serial.println ("IMU initialization unsuccessful");
    Serial.println ("Check IMU wiring or try cycling power");
    Serial.print ("status:");
    Serial.println (status);

    while (1) {}
  }

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void forwardomniwheel()
{
      digitalWrite ((brake3 && direction3 && direction4), HIGH);
      digitalWrite ((brake4 && direction5), LOW);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println (" MOVE FORWARD OMNIWHEEL");
}

void backwardomniwheel()
{
      digitalWrite ((brake5 && direction4), LOW);
      digitalWrite ((brake6 && direction3 && direction5), HIGH);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println (" MOVE BACKWARD OMNIWHEEL");
}

void rightomniwheel()
{
      digitalWrite ((brake3 && direction3), LOW);
      digitalWrite ((brake4 && direction4 && direction5), HIGH);
      analogWrite (PWM1, -1023);
      analogWrite (PWM2, 1023);
      Serial.println (" MOVE RIGHT OMNIWHEEL");
}

void leftomniwheel()
{
      digitalWrite ((brake3 && direction3), HIGH);
      digitalWrite ((brake4 && direction4 && direction5), LOW);
      analogWrite (PWM1, -1023);
      analogWrite (PWM2, 1023);
      Serial.println (" MOVE LEFT OMNIWHEEL");
}

void spinclockwiseomniwheel()
{
      digitalWrite ((brake3 && brake6), LOW);
      digitalWrite ((brake4 && brake5 && direction3 && direction4 && direction5), HIGH);
      analogWrite (PWM1, 1023);
      analogWrite (PWM2, 1023);
      Serial.println (" SPIN CLOCKWISE OMNIWHEEL");
}

void spincounterclockwiseomniwheel()
{
      digitalWrite ((brake3 && brake6), HIGH);
      digitalWrite ((brake4 && brake5 && direction3 && direction4 && direction5), LOW);
      analogWrite (PWM1, 1023);
      analogWrite (PWM2, 1023);
      Serial.println (" SPIN COUNTER CLOCKWISE OMNIWHEEL");
}

void stop_xomniwheel()
{
      digitalWrite ((brake3 && brake4 && direction3 && direction4 && direction5), LOW);
      analogWrite ((PWM1 && PWM2), 0);
      Serial.println (" STOP_X OMNIWHEEL");
}

void stop_yomniwheel()
{
      digitalWrite ((brake3 && brake4 && direction3 && direction4 && direction5), LOW);
      analogWrite ((PWM1 && PWM2), 0);
      Serial.println (" STOP_Y OMNIWHEEL");
}

void forwardDIYrobot()
{
      digitalWrite ((brake1 && direction1), LOW);
      digitalWrite ((brake2 && direction2), HIGH);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println ("MOVE FORWARD DIY ROBOT");
}

void backwardDIYrobot()
{
      digitalWrite ((brake1 && brake2), HIGH);
      digitalWrite ((direction1 && direction2), LOW);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println ("MOVE BACKWARD DIY ROBOT");
}

void rightDIYrobot()
{
      digitalWrite ((brake1 && direction2), HIGH);
      digitalWrite ((direction1 && brake2), LOW);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println ("MOVE RIGHT DIY ROBOT");
}

void leftDIYrobot()
{
      digitalWrite ((brake1 && direction2), LOW);
      digitalWrite ((direction1 && brake2), HIGH);
      analogWrite ((PWM1 && PWM2), 1023);
      Serial.println ("MOVE LEFT DIY ROBOT");
}

void stopall()
{
      digitalWrite ((LEDMotor1 && LEDMotor2 && LEDMotor3 && LEDBlue1 && LEDBlue2 && LEDMPU1 && LEDMPU2 && LEDU1 && LEDU2 && LEDU3 && LEDU4 && LEDU5 && LEDU6), LOW);
      digitalWrite ((brake1 && brake2 && brake3 && brake4 && brake5 && brake6 && direction1 && direction2 && direction3 && direction4 && direction5), LOW);
      analogWrite ((PWM1 && PWM2), 0);
      Serial.println ("STOP ALL");
}

void controlomniwheel()
{
    if (BlueTooth.available())
  {
    BT_input = BlueTooth.read();

    if (BT_input == 'F' )
    {
      forwardomniwheel();
    }

    else if (BT_input == 'B' )
    {
      backwardomniwheel();
    }

    else if (BT_input == 'R' )
    {
      rightomniwheel();
    }

    else if (BT_input == 'L' )
    {
      leftomniwheel();
    }

    else if (BT_input == 'G' )
    {
      spinclockwiseomniwheel();
    }

    else if (BT_input == 'I' )
    {
      spincounterclockwiseomniwheel();
    }
    else if (BT_input == 'H' )
    {
      stop_xomniwheel();
    }

    else if (BT_input == 'J' )
    {
      stop_yomniwheel();
    }
    else if (!(BT_input == 'J') && !(BT_input == 'H') && !(BT_input == 'I') && !(BT_input == 'G') && !(BT_input == 'L') && !(BT_input == 'R') && !(BT_input == 'F') && !(BT_input == 'B'))
    {
      stopall();
    }
  }
 delay(50);
  }

void controlDIYrobot()
{
    if (BlueTooth.available())
  {
    BT_input = BlueTooth.read();

    if (BT_input == 'F' )
    {
      forwardDIYrobot();
    }

    else if (BT_input == 'B' )
    {
      backwardDIYrobot();
    }

    else if (BT_input == 'R' )
    {
      rightDIYrobot();
    }

    else if (BT_input == 'L' )
    {
      leftDIYrobot();
    }

    else if (!(BT_input == 'J') && !(BT_input == 'H') && !(BT_input == 'I') && !(BT_input == 'G') && !(BT_input == 'L') && !(BT_input == 'R') && !(BT_input == 'F') && !(BT_input == 'B'))
    {
      stopall();
    }  
  }
  delay(50);
}

void loop () {

  mpu6050.update();

  if (millis() - timer > 1000) {

    digitalWrite (LEDMPU1, HIGH);
    Serial.println ("=======================================================");
    Serial.print ("temp : "); Serial.println (mpu6050.getTemp());
    Serial.print ("accX : ");  Serial.print (mpu6050.getAccX());
    Serial.print ("  accY : "); Serial.print (mpu6050.getAccY());
    Serial.print ("  accZ : "); Serial.println (mpu6050.getAccZ());

    Serial.print ("gyroX : "); Serial.print (mpu6050.getGyroX());
    Serial.print ("  gyroY : "); Serial.print (mpu6050.getGyroY());
    Serial.print ("  gyroZ : "); Serial.println (mpu6050.getGyroZ());

    Serial.print ("accAngleX : "); Serial.print (mpu6050.getAccAngleX());
    Serial.print ("  accAngleY : "); Serial.print (mpu6050.getAccAngleY());

    Serial.print ("gyroAngleX : "); Serial.print (mpu6050.getGyroAngleX());
    Serial.print ("  gyroAngleY : "); Serial.print (mpu6050.getGyroAngleY());
    Serial.print ("  gyroAngleZ : "); Serial.println (mpu6050.getGyroAngleZ());

    Serial.print ("angleX : "); Serial.print (mpu6050.getAngleX());
    Serial.print ("  angleY : "); Serial.print (mpu6050.getAngleY());
    Serial.print ("  angleZ : "); Serial.println (mpu6050.getAngleZ());
    Serial.println ("=======================================================");

  timer = millis ();
  }

  IMU.readSensor ();
  digitalWrite (LEDMPU2, HIGH);
  Serial.print ("AccelX: "); Serial.print (IMU.getAccelX_mss(), 6); Serial.print ("   ");
  Serial.print ("AccelY: "); Serial.print (IMU.getAccelY_mss(), 6); Serial.print ("   ");  
  Serial.print ("AccelZ: "); Serial.println (IMU.getAccelZ_mss(), 6);

  Serial.print ("GyroX: "); Serial.print (IMU.getGyroX_rads(), 6); Serial.print ("   ");
  Serial.print ("GyroY: "); Serial.print (IMU.getGyroY_rads(), 6); Serial.print ("   ");  
  Serial.print ("GyroZ: "); Serial.println (IMU.getGyroZ_rads(), 6);

  Serial.print ("MagX: "); Serial.print (IMU.getMagX_uT(), 6); Serial.print ("   ");
  Serial.print ("MagY: "); Serial.print (IMU.getMagY_uT(), 6); Serial.print ("   ");  
  Serial.print ("MagZ: "); Serial.println (IMU.getMagZ_uT(), 6);

  Serial.print ("Temperature in C : "); Serial.println (IMU.getTemperature_C(), 6); Serial.println();
  delay(200);

  digitalWrite ((trig_1 && trig_2 && trig_3 && trig_4 && trig_5 && trig_6), HIGH);
  delayMicroseconds (10);
  digitalWrite ((trig_1 && trig_2 && trig_3 && trig_4 && trig_5 && trig_6), LOW);

  time1 = pulseIn (echo_1, HIGH);
  time2 = pulseIn (echo_2, HIGH);
  time3 = pulseIn (echo_3, HIGH);
  time4 = pulseIn (echo_4, HIGH);
  time5 = pulseIn (echo_5, HIGH);
  time6 = pulseIn (echo_6, HIGH);

  distance1 = (time1 * 0.034) /2;
  distance2 = (time2 * 0.034) /2;
  distance3 = (time3 * 0.034) /2;
  distance4 = (time4 * 0.034) /2;
  distance5 = (time5 * 0.034) /2;
  distance6 = (time6 * 0.034) /2;

  Serial.print (distance1);
  Serial.print (distance2);
  Serial.print (distance3);
  Serial.print (distance4);
  Serial.print (distance5);
  Serial.print (distance6);
  Serial.println ("CM");

  delay(10);

  if (!(distance1 >= 36 && distance2 >= 89) && !(distance5 >= 90 && distance6 >= 90 ) && !(distance3 >= 68 && distance4 >= 25))
  {
    digitalWrite (LEDMotor1 && LEDMotor2 && LEDMotor3, HIGH);
    controlomniwheel();
    Serial.println ("Nothing detected");
    Serial.print ("Distance=");
    Serial.println (distance1);
    Serial.println (distance2);
    Serial.println (distance3);
    Serial.println (distance4);
    Serial.println (distance5);
    Serial.println (distance6);
  }

  else if ((distance3 >= 36 && distance4 >= 89) && (distance1>=68 && distance2 >= 25))
  {
    digitalWrite (LEDU3, HIGH);
    digitalWrite (LEDU4, HIGH);

    Serial.println ("DOWNSTAIR DETECTED");
    Serial.print ("Distance=");
    Serial.println (distance3);
    Serial.println (distance4);

    controlDIYrobot();
  }

  else if (( distance5 >= 90 && distance6 >= 90 ))
  {
    digitalWrite (LEDU5, HIGH);
    digitalWrite (LEDU6, HIGH);

    Serial.println ("PIT DETECTED");
    Serial.print ("Distance=");
    Serial.print (distance5);
    Serial.print (distance6);
    stopall();
  }
}
