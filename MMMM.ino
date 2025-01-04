#include <Servo.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>
#define TCAADDR 0x70
MPU9250 IMU0(Wire,0x70);
MPU9250 IMU1(Wire,0x70);
int status;
void TCA9548A(uint8_t bus)
{
Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
Wire.write(1 << bus);          // send byte to select bus
Wire.endTransmission();
}
// PS2 Tank by Igor Fonseca @2019
// Controls a robotic tank using a PS2 joystick, using D-pad buttons
// based on an example using the PS2X library by Bill Porter 2011
// All text above must be included in any redistribution.
// include libraries
#include "PS2X_lib.h"
#define TRIG_PIN A0
#define ECHO_PIN A1
#define enA 13
#define in1 12 
#define in2 11
#define in3 7
#define in4 6
#define enB 5
#define servoPin 2
// These are used to set the direction of the bridge driver.
#define ENA 3      //ENA
#define MOTORA_1 4 //IN3
#define MOTORA_2 5 //IN4
#define MOTORB_1 8 //IN1
#define MOTORB_2 7 //IN2
#define ENB 6      //ENB
PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;
//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller,
//or call config_gamepad(pins) again after connecting the controller.
float motA, motB, motC;
float PWMA, PWMB, PWMC;
float cm;
float inches;
int i;
char userInput;
int cwSpeed = 500;
int ccwSpeed = -500;
int speedIn = 100;
int batLevel = 95;
AccelStepper stepperA(AccelStepper::DRIVER, 3, 2); //defines 3 as the step pin, 2 as direction
AccelStepper stepperB(AccelStepper::DRIVER, 5, 4);
AccelStepper stepperC(AccelStepper::DRIVER, 7, 6);
class Motor{
int cwSpeed = 500;
int ccwSpeed = -500;
int speedIn = 100;
int batLevel = 95;
int enablePin;
int directionPin1;
int directionPin2;
public:
Motor(int ENPin,int dPin1,int dPin2){
enablePin = ENPin;
directionPin1 = dPin1;
directionPin2 = dPin2;
};
void Drive(int speed){
if(speed>=0){
digitalWrite(directionPin1, LOW);
digitalWrite(directionPin2, HIGH);
}
else{
digitalWrite(directionPin1, LOW);
digitalWrite(directionPin2, LOW);
speed = - speed;
}
analogWrite(enablePin, speed);
}
};
Motor leftMotor = Motor(enA, in1, in2);
Motor rightMotor = Motor(enB, in3, in4);
Servo myservo;
void motorInitiation(){
pinMode(enA, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
digitalWrite(enA, LOW);
digitalWrite(enB, LOW);
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
const unsigned int MAX_DIST = 23200;
bool ObsticalAhead = false;
enum Directions { Forward, TurnLeft, TurnRight, TurnAround,Brake};
Directions nextStep = Forward;
unsigned long t1;
unsigned long t2;
unsigned long pulse_width;
void setup()
{
Wire.begin();
Serial.begin(9600);
status = IMU0.begin();
status = IMU1.begin();
Serial.println(F("\nSend any character to begin DMP programming and demo: "));
while (Serial.available() && Serial.read()); // empty buffer
while (!Serial.available());                 // wait for data
while (Serial.available() && Serial.read()); // empty buffer again
Serial.println(F("AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ"));
// Configure output pins
pinMode(ENA, OUTPUT);
pinMode(MOTORA_1, OUTPUT);
pinMode(MOTORA_2, OUTPUT);
pinMode(ENB, OUTPUT);
pinMode(MOTORB_1, OUTPUT);
pinMode(MOTORB_2, OUTPUT);
// Disable both motors
digitalWrite(ENA,0);
digitalWrite(ENB,0);
// Start serial communication
Serial.begin(57600);
error = ps2x.config_gamepad(13,11,10,12, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
// Check for error
if(error == 0){
Serial.println(F("Found Controller, configured successful"));
}
else if(error == 1)
Serial.println(F("No controller found, check wiring or reset the Arduino"));
else if(error == 2)
Serial.println("Controller found but not accepting commands");
else if(error == 3)
Serial.println(F("Controller refusing to enter Pressures mode, may not support it."));   
// Check for the type of controller
type = ps2x.readType();
switch(type) {
case 0:
Serial.println(F("Unknown Controller type"));
break;
case 1:
Serial.println(F("DualShock Controller Found"));
break;
case 2:
Serial.println(F("GuitarHero Controller Found"));
break;
}
pinMode(ENB, OUTPUT);
pinMode(MOTORB_1, OUTPUT);
pinMode(MOTORB_2, OUTPUT);
pinMode(TRIG_PIN, OUTPUT);
digitalWrite(TRIG_PIN, LOW);
Serial.begin(9600);
myservo.attach(servoPin);
motorInitiation();
Directions nextStep = Forward;
stepperA.setMaxSpeed(3000);
stepperA.setAcceleration(1000);
stepperB.setMaxSpeed(3000);
stepperB.setAcceleration(1000);
stepperC.setMaxSpeed(3000);
stepperC.setAcceleration(1000);
Serial.begin(9600);
Serial.println(F("Omni-Bot Control Begin"));
Serial.println(F("enter: f, b, r, l, c, a, s"));
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
Serial.begin(57600);
error = ps2x.config_gamepad(13,11,10,12,true,true);
if(error == 0){
Serial.println(F("Found controller, configured successful"));
Serial.println(F("Try out all the buttons, X will vibrate the controller, faster as you press harder"));
Serial.println(F("holding L1 or R1 will print out the analog stick values."));
Serial.println(F("Go to www.billporter.info for updates and to report bugs."));
}
}
void loop()
{
TCA9548A(7);
IMU0.readSensor();
Serial.print(F("IMU Sensor 1:"));
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getAccelX_mss(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getAccelY_mss(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getAccelZ_mss(),6);
Serial.print(F(","));
Serial.print(F("\t")); 
Serial.print(IMU0.getGyroX_rads(),6);  
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getGyroY_rads(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getGyroZ_rads(),6);  
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU0.getMagX_uT(),6);
Serial.print(F(","));
Serial.print(F("\t"));  Serial.print(IMU0.getMagY_uT(),6);
Serial.print(F(","));  Serial.print(F("\t"));
Serial.print(IMU0.getMagZ_uT(),6);
Serial.print(",");
Serial.println(F("\t"));
delay(100);
TCA9548A(6);
IMU1.readSensor();
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getAccelX_mss(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getAccelY_mss(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getAccelZ_mss(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getGyroX_rads(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getGyroY_rads(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getGyroZ_rads(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getMagX_uT(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getMagY_uT(),6);
Serial.print(F(","));
Serial.print(F("\t"));
Serial.print(IMU1.getMagZ_uT(),6);
Serial.print(F(","));
Serial.println(F("\t"));  
delay(100);
if(error == 1)
return;
else {
ps2x.read_gamepad();
if(ps2x.ButtonPressed(GREEN_FRET))
Serial.println(F("Green Fret Pressed"));
if(ps2x.ButtonPressed(RED_FRET))
Serial.println(F("Red Fret Pressed"));
if(ps2x.ButtonPressed(YELLOW_FRET))
Serial.println(F("Yellow Fret Pressed"));
if(ps2x.ButtonPressed(BLUE_FRET))
Serial.println(F("Blue Fret Pressed"));
if(ps2x.ButtonPressed(ORANGE_FRET))
Serial.println(F("Orange Fret Pressed"));
if(ps2x.ButtonPressed(STAR_POWER))
Serial.println(F("Star Power Command"));
if(ps2x.ButtonPressed(UP_STRUM))
Serial.println(F("Up Strum"));
if(ps2x.ButtonPressed(DOWN_STRUM))
Serial.println(F("Down Strum"));
if(ps2x.ButtonPressed(PSB_START))
Serial.println(F("Start is being held"));
if(ps2x.ButtonPressed(PSB_SELECT))
Serial.println(F("Select is being held"));
if(ps2x.ButtonPressed(ORANGE_FRET))
{
Serial.println(F("Wammy Bar Position:"));
Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
}
else {
ps2x.read_gamepad(false, vibrate);
if(ps2x.Button(PSB_START))
Serial.println(F("Start is being held"));
if(ps2x.Button(PSB_SELECT))
Serial.println(F("Select is being held"));
if(ps2x.Button(PSB_PAD_UP)) {
Serial.print(F("Up held this hard: "));
Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
}
if(ps2x.Button(PSB_PAD_RIGHT)){
Serial.print(F("Right held this hard: "));
Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
}
if(ps2x.Button(PSB_PAD_LEFT)){
Serial.print(F("LEFT held this hard: "));
Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
}
if(ps2x.Button(PSB_PAD_DOWN)){
Serial.print(F("DOWN held this hard: "));
Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
}
vibrate = ps2x.Analog(PSAB_BLUE);
if (ps2x.NewButtonState())
{
if(ps2x.Button(PSB_L3))
Serial.println(F("L3 pressed"));
if(ps2x.Button(PSB_R3))
Serial.println(F("R3 pressed"));
if(ps2x.Button(PSB_L2))
Serial.println(F("L2 pressed"));
if(ps2x.Button(PSB_R2))
Serial.println(F("R2 pressed"));
if(ps2x.Button(PSB_GREEN))
Serial.println(F("Triangle pressed"));
}
if(ps2x.ButtonPressed(PSB_RED))
Serial.println(F("Circle just pressed"));
if(ps2x.ButtonReleased(PSB_PINK))
Serial.println(F("Square just released"));
if(ps2x.NewButtonState(PSB_BLUE))
Serial.println(F("X just changed"));
if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
{
Serial.print(F("Stick Values:"));
Serial.print(ps2x.Analog(PSS_LY), DEC);
Serial.print(F(","));
Serial.print(ps2x.Analog(PSS_LX), DEC);
Serial.print(F(","));
Serial.print(ps2x.Analog(PSS_RY), DEC);
Serial.print(F(","));
Serial.println(ps2x.Analog(PSS_RX), DEC);
}
}
delay(50);
}
int userInput = Serial.read();
int speedPot = analogRead(A0);
int batIn = analogRead(A1);
int speedIn = map(speedPot, 0, 750, 0, 1000);
int cwSpeed = speedIn;
int ccwSpeed = -speedIn;
int batLevel = map(batIn, 0, 1023, 0, 250);
if(batLevel > 94) {
digitalWrite(10, HIGH);
digitalWrite(11, HIGH);
}
else if(batLevel < 95) {
digitalWrite(10, LOW);
digitalWrite(11, LOW);
}
if(userInput =='f') fwd();
else if(userInput =='b') bkw();
else if(userInput =='r') rt();
else if(userInput =='l') lt();
else if(userInput =='c') cw();
else if(userInput =='a') ccw();
else if(userInput =='s') stp();
stepperA.runSpeed();
stepperB.runSpeed();
stepperC.runSpeed();
checkDistance();
checkDirection();
drive();
}
void fwd(){
Serial.println(F("Bot moving forward"));
acwMotor();
bstopMotor();
cccwMotor();
}
void bkw(){
Serial.println(F("Bot moving backwards"));
accwMotor();
bstopMotor();
ccwMotor();
}
void rt(){
Serial.println(F("Bot moving right"));
astopMotor();
bccwMotor();
cstopMotor();
}
void lt(){
Serial.println(F("Bot moving left"));
astopMotor();
bcwMotor();
cstopMotor();
}
void cw() {
Serial.println(F("Bot moving clockwise"));
acwMotor();
bcwMotor();
ccwMotor();
}
void ccw() {
Serial.println(F("Bot moving counter-clockwise"));
accwMotor();
bccwMotor();
cccwMotor();
}
void stp() {
Serial.println(F("Bot stop"));
astopMotor();
bstopMotor();
cstopMotor();
}
void acwMotor(){
Serial.println(F("Motor A clockwise"));
stepperA.setSpeed(cwSpeed);
}
void accwMotor(){
Serial.println(F("Motor A counter-clockwise"));
stepperA.setSpeed(ccwSpeed);
}
void bcwMotor(){
Serial.println(F("Motor B clockwise"));
stepperB.setSpeed(cwSpeed);
}
void bccwMotor(){
Serial.println(F("Motor B counter-clockwise"));
stepperB.setSpeed(ccwSpeed);
}
void ccwMotor(){
Serial.println(F("Motor C clockwise"));
stepperC.setSpeed(cwSpeed);
}
void cccwMotor(){
Serial.println(F("Motor C counter-clockwise"));
stepperC.setSpeed(ccwSpeed);
}
void astopMotor(){
Serial.println(F("Motor A stop"));
stepperA.setSpeed(0);
}
void bstopMotor(){
Serial.println(F("Motor A stop"));
stepperB.setSpeed(0);
}
void cstopMotor(){
Serial.println(F("Motor A stop"));
stepperC.setSpeed(0);
}
void checkDistance(){
digitalWrite(TRIG_PIN, HIGH);
delayMicroseconds(10);
digitalWrite(TRIG_PIN, LOW);
while (digitalRead(ECHO_PIN) == 0 );
t1 = micros();
while(digitalRead(ECHO_PIN) == 1);
t2 = micros();
pulse_width = t2 - t1;
cm = pulse_width / 58.0;
inches = pulse_width / 1480;
if ( pulse_width > MAX_DIST ) {
Serial.println(F("Out of range"));
}
else { 
}
delay(60);
if(cm<=20){
ObsticalAhead = true;
Serial.println(F("problem Ahead"));
}
else{ObsticalAhead = false;}
}
void checkDirection(){
Serial.println(F("checking direction"));
if(ObsticalAhead ==true){
nextStep = Brake;
drive();
myservo.write(180);
delay(400);
checkDistance();
if(ObsticalAhead ==false){
nextStep = TurnLeft;
Serial.println(F("Next step is TurnLeft"));
myservo.write(90);
delay(400);
}
myservo.write(0);
delay(800);
checkDistance();
if(ObsticalAhead ==false){
nextStep = TurnRight;
Serial.println(F("Next step is TurnRight"));
myservo.write(90);
delay(400);
}
else{
nextStep = TurnAround;
myservo.write(90);
delay(300);
Serial.println(F("Next step is TurnAround"));
}
}
else{nextStep = Forward;}
}
void drive(){
switch (nextStep){
case Forward:
leftMotor.Drive(255);
rightMotor.Drive(255);
Serial.println(F("Forward"));
break;
case TurnLeft:
leftMotor.Drive(-255);
rightMotor.Drive(255);
Serial.println(F(" TurnLeft"));
delay(400);
break;
case TurnRight:
leftMotor.Drive(255);
rightMotor.Drive(-255);
Serial.println(F(" TurnRight"));
delay(400);
break;
case TurnAround:
leftMotor.Drive(255);
rightMotor.Drive(-255);
Serial.println(F(" TurnAround"));
delay(600);
break;
case Brake:
leftMotor.Drive(0);
rightMotor.Drive(0);
Serial.println(F(" stopped"));
}
}
