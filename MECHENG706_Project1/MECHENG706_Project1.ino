#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h> // Need for wireless module.
//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

/*
//Wireless Setup
#define INTERNAL_LED 13

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

#define STARTUP_DELAY 10 // Seconds
#define LOOP_DELAY 10 // miliseconds
#define SAMPLE_DELAY 10 // miliseconds


// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
*/

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Motor Control Setup
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;
int speed_val = 300;
int speed_change;
int pos = 0;

//IR Sensor Setup
int IR1 = A5;
int IR2 = A7;
int leftIR = A4;
int rightIR = A6;

//Gyroscope Setup
float sensorPin = A8;               //define the pin that gyro is connected
int T = 400;                        // T is the time of one loop
int sensorValue = 0;                // read out value of sensor
float gyroSupplyVoltage = 5;        // supply voltage for gyro
float gyroZeroVoltage = 0;          // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
float gyroRate = 0;                 // read out value of sensor in voltage
float currentAngle = 0;            // current angle calculated by angular velocity integral on

//Ultrasonic Sensor Setup
const int TRIG_PIN = 49;
const int ECHO_PIN = 48;
const unsigned int MAX_DIST = 23200; // Anything over 400 cm (23200 us pulse) is "out of range"

//Kalman filter setup
double last_var = 999;
double process_noise = 1;
double last_est_rightIR = 0;
double last_est_leftIR = 0;
double last_est_IR1 = 0;
double last_est_IR2 = 0;
//Serial Pointer
HardwareSerial *SerialCom;

void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  //**********Gyroscope Setup Start**********//
  pinMode(sensorPin, INPUT);
  int i;
  float sum = 0;
  for (i = 0; i < 100; i++)       //read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(sensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;   //average the sum as the zero drifting
  pinMode(TRIG_PIN, OUTPUT);     //the Trigger pin will tell the sensor to range find
  digitalWrite(TRIG_PIN, LOW);
  //**********Gyroscope Setup End**********//

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("PROTOTYPE 18/03/2022");
  delay(1000);
  SerialCom->println("Setup....");

  /*
  // Wireless module setup
  BluetoothSerial.begin(115200);

  Serial.print("Ready, waiting for 1000ms");
  */
  delay(1000); //settling time but no really needed
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}

STATE initialising() {
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  // This is my new comment! :)
  float IR1_distance;
  float IR2_distance;
  float leftIR_distance;
  float rightIR_distance;
  
  static unsigned long previous_millis;
  fast_flash_double_LED_builtin();

  /*--------------------------------COURSE START--------------------------------*/
  /*while (1) {
    IR1_distance = IR1_read();
    IR2_distance = IR2_read();
    leftIR_distance = leftIR_read();
    rightIR_distance = rightIR_read();

    Serial.println((String)"IR1: " + IR1_distance + (String)" IR2: " + IR2_distance + " leftIR: " + leftIR_distance + " rightIR: " + rightIR_distance);
  } 
  */
 Serial.println("Started the course.");
 //FindCorner();
 //CCWstraighten();
 strafe_left();
 delay(1000);
 Serial.println("Finished the course.");
 delay(1000);
  //  Serial.println("Turn by angle starting...");
  //  TurnByAngle(90);
  //  Serial.println("Turn by angle finished");
  //  delay(1000);
  //  driveToWall();                                    //move forward until detecting a wall
  //  delay(1000);
  ////  straighten();                                     //straighten up 90 degrees to the wall
  // TurnByAngle(90);                                             //turn 90 degrees to the right
  //  delay(1000);
  //  driveToWall();                                    //move forward until detecting a wall, now robot will be in a corner
  //  //  straighten();                                     //straighten up 90 degrees to the wall
  //  delay(1000);
  // TurnByAngle(90);                                             //turn 90 degrees to the right to find out which orientation robot is in
  //  IR1_distance = IR1_read();                        //measure the distance to the wall to figure out orientation
  //  IR2_distance = IR2_read();
  //  if ((IR1_distance < 140) || (IR2_distance < 140)) {
  //   TurnByAngle(90);                                           //TO BE TUNED!! if the wall is less than x, need to turn again before completing course
  //  }
  //  driveToWall();                                   //continue move forward until detecting a wall
  //  float leftDistance = leftIR_read();              //check distance to the left of the robot to determine CW / CCW direction
  //  if (leftDistance < 25) {
  //  CWcorner();                                    //if wall is on the left of the robot, complete CW course
  //  }
  //  else {
  //    CCWcorner();                                   //else the wall is on the right, so complete CCW course
  //  }
  stop();
  return STOPPED;
  /*--------------------------------COURSE END--------------------------------*/

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    return RUNNING;
  }
}

/*--------------------------------READING IR SENSORS--------------------------------*/
float IR1_read()
{
  double est;
  int signalADC = analogRead(IR1);
  //float distance = 17948 * pow(signalADC, -1.22);
  float distance = 9380 * pow(signalADC, -1.11);
//  est = Kalman(distance, last_est_IR1, 2);
//  if (isnan(est)){
//  last_est_IR1 = 0;
//  }
//  else {
//    last_est_IR1 = est;
//  }
//  Serial.print("Distance reading for front right IR1 (in cm): ");
//  Serial.println(est);
  delay(100); //Delay 0.1 second
  //return est;
  return distance;
}

float IR2_read()
{
  double est;
  int signalADC = analogRead(IR2);
  //float distance = 17948 * pow(signalADC, -1.22);
  float distance = 2551 * pow(signalADC, -0.885);
//  est = Kalman(distance, last_est_IR2, 2); //Kalman filter
//  if (isnan(est)){
//  last_est_IR2 = 0;
//  }
//  else {
//    last_est_IR2 = est;
//  }
//  Serial.print("Distance reading for front left IR2 (in cm): ");
//  Serial.print(est);
//  Serial.print(" ; "
  delay(100); //Delay 0.1 second
 // return est;
 return distance;
}

float leftIR_read()
{
  double est;
  int signalADC = analogRead(leftIR);
  //float distance = 17948 * pow(signalADC, -1.22);
  float distance = 2550 * pow(signalADC, -1.01);
//  est = Kalman(distance, last_est_leftIR, 2); //Kalman filter
//  if (isnan(est)){
//  last_est_leftIR = 0;
//  }
//  else {
//    last_est_leftIR = est;
//  }
//  Serial.print("Distance reading for back left IR (in cm): ");
//  Serial.print(est);
//  Serial.print(" ; ");
  delay(100); //Delay 0.1 second
  return distance;
 // return est;
}

float rightIR_read()
{
  double est;
  int signalADC = analogRead(rightIR);
  //float distance = 17948 * pow(signalADC, -1.22);
  float distance = 1788 * pow(signalADC, -0.924);
//  est = Kalman(distance, last_est_rightIR, 2); //Kalman filter
//  if (isnan(est)){
//  last_est_rightIR = 0;
//  }
//  else {
//    last_est_rightIR = est;
//  }
  //Serial.print("Distance reading for right IR (in cm): ");
  //Serial.println(est);
  delay(100); //Delay 0.1 second
  return distance;
  //return est;
}

double Kalman(double rawdata, double prev_est, double sensor_noise) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var + process_noise;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_est = constrain(a_post_est, 0, 999); // constrain output to prevent infinite values.
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var = a_post_var;
  return a_post_est;
  
}

/*--------------------------------READING GYRO SENSOR--------------------------------*/
float gyro_read()
{
  // convert the 0-1023 signal to 0-5v
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate = (analogRead(sensorPin) - gyroZeroVoltage) * gyroSupplyVoltage / 1023;

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0)
  {
    currentAngle += 360;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }

  //  Serial.print("Potentiometre ");
  //  Serial.println(analogRead(sensorPin));
  //  Serial.print("Angular velocity: ");
  //  Serial.println(angularVelocity);
  //  Serial.print("Current angle: ");
  //  Serial.println(currentAngle);

  // control the time per loop
  delay (T);
  return currentAngle;
}

//Angular Velocity
float gyro_velocity()
{
  // convert the 0-1023 signal to 0-5v
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate = (analogRead(sensorPin) - gyroZeroVoltage) * gyroSupplyVoltage / 1023;

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0)
  {
    currentAngle += 360;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }

  Serial.print("Potentiometre ");
  Serial.println(analogRead(sensorPin));
  Serial.print("Angular velocity: ");
  Serial.println(angularVelocity);
  Serial.print("Current angle: ");
  Serial.println(currentAngle);

  // control the time per loop
  delay (T);
  return angularVelocity;
}

/*--------------------------------MOTOR MOVEMENT FUNCTIONS--------------------------------*/

void FindCorner()
{
  float ultraDist = HC_SR04_range();
  float frontLeft = IR2_read();
  float frontRight = IR1_read();
  float backLeft = leftIR_read();
  float backRight = rightIR_read();
  float iAngle = gyro_read();
  Serial.print("Ultrasond reading is: ");
  Serial.print(ultraDist);
  Serial.print("  Front right IR1: ");
  Serial.print(frontRight);
  Serial.print("  Front left IR2: ");
  Serial.print(frontLeft);
  Serial.print("  Back left IR: ");
  Serial.print(backLeft);
  Serial.print("  Back right IR: ");
  Serial.println(backRight);
 
  //Orientate the robot to face a wall 60cm away
  while (ultraDist > 60) {
    Serial.println("Orientate the robot to face a wall 60cm away");
    cw();
    ultraDist = HC_SR04_range();
    Serial.print("Ultrasond reading is: ");
    Serial.println(ultraDist);
  }
  stop();

  //Drive straight until any sensor sees a wall 15cm away
  while ((ultraDist > 15) && (frontLeft > 15) && (backLeft > 15) && (frontRight > 15) && (backRight > 15)) {
    Serial.println("Drive straight until any sensor sees a wall 15cm away");
    iAngle = gyro_read();
    forward(iAngle);
    ultraDist = HC_SR04_range();
    frontLeft = IR2_read();
    frontRight = IR1_read();
    backLeft = leftIR_read();
    backRight = rightIR_read();
      Serial.print("Ultrasond reading is: ");
  Serial.print(ultraDist);
  Serial.print("  Front right IR1: ");
  Serial.print(frontRight);
  Serial.print("  Front left IR2: ");
  Serial.print(frontLeft);
  Serial.print("  Back left IR: ");
  Serial.print(backLeft);
  Serial.print("  Back right IR: ");
  Serial.println(backRight);
  }
  stop();


  
  if (frontRight < 20) {
    Serial.println("Front right near wall");
    CCWstraighten();
  } else if (frontLeft < 20) {
    CWstraighten();
  } else if ((frontLeft<200) && (frontRight<200)) {
    if (frontLeft > frontRight) {
      while (frontRight > 15.2) { //calibrate later
        strafe_right();
        frontRight = IR1_read();
      }
    } else if (frontRight > frontLeft) {
      while (frontLeft > 15.2) { //calibrate later
        strafe_left();
        frontLeft = IR2_read();
      }
    }
  } else if ((frontLeft>250) && (frontRight>250)) {
    TurnByAngle(90);

  } else if ((frontLeft<250) && (frontRight>250)) {
    while (frontLeft > 15.2) { //calibrate later
      strafe_left();
      frontLeft = IR2_read();
    }
  } else if ((frontRight<250) && (frontLeft>250)) {
    while (frontRight > 15.2) { //calibrate later
      strafe_right();
      frontRight = IR1_read();
    }
  }
  stop();
  //return;

  while (ultraDist > 15) {
    iAngle = gyro_read();
    forward(iAngle);
    ultraDist = HC_SR04_range();
  }
  stop();

  //Now we are in a corner
  if (leftIR_read() < 20) {
    TurnByAngle(90);
    ultraDist = HC_SR04_range();
    if (ultraDist > 130) {
      return;
    } else {
      TurnByAngle(90);
      return;
    }
  } else {
    TurnByAngle(-90);
    ultraDist = HC_SR04_range();
    if (ultraDist > 130) {
      return;
    } else {
      TurnByAngle(-90);
      return;
    }
  }
}


void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void CWstraighten() {
  float frontL = IR2_read();
  float backL = leftIR_read();
  float error = frontL - backL;

  while (abs(error) > 0.2) { //calibrate this later
    cw();
    frontL = IR2_read();
    backL = leftIR_read();
    error = frontL - backL;
  }
  //now the robot is aligned to the wall, check for 15cm distance
  while (abs(15 - frontL) > 0.2) { //calibrate this later
    if (frontL < 15) {
      strafe_right();
      frontL = IR2_read();
    } else if (frontL > 15) {
      strafe_left();
      frontL = IR2_read();
    }
  }
  stop();
}

void CCWstraighten() {
  float frontR = IR1_read();
  float backR = rightIR_read();
  float error = frontR - backR;
    Serial.print("Front right IR1: ");
  Serial.print(frontR);
  Serial.print("  Back right IR: ");
  Serial.print(backR);
  Serial.print("  Error is: ");
  Serial.println(error);

  while (abs(error) > 2) { //calibrate this later
      left_font_motor.writeMicroseconds(1400);
  left_rear_motor.writeMicroseconds(1400);
  right_rear_motor.writeMicroseconds(1400);
  right_font_motor.writeMicroseconds(1400);
    frontR = IR1_read();
    backR = rightIR_read();
    error = frontR - backR;
  }

  //now the robot is aligned to the wall, check for 15cm distance
  while (abs(15 - frontR) > 0.2) { //calibrate this later
    if (frontR < 15) {
      strafe_left();
      frontR = IR1_read();
    } else if (frontR > 15) {
      strafe_right();
      frontR = IR1_read();
    }
  }
  stop();
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}


//Drive straight until hitting a wall
void driveToWall()
{
  Serial.println("Drive to Wall Started");
  float IR1_dist = IR1_read();
  float IR2_dist = IR2_read();
  float initAngle = gyro_read();

  while ((IR1_dist > 15) && (IR2_dist > 15)) {
    Serial.println("While Loop entered");
    forward(initAngle);
    IR1_dist = IR1_read();
    IR2_dist = IR2_read();
  }
  Serial.println("Distance reached");
  stop();
}

void straighten()
{
  float IR1_dist = IR1_read();//right
  float IR2_dist = IR2_read();//left
  float error, u, lastError, integral, derivative, speed = 0;
  float integralLimit = 30;
  //float error = IR1_dist - IR2_dist;
  float Kp = 1;
  float Ki = 1;
  int timer = 500;

  while (timer > 0) {

    error = IR1_dist - IR2_dist; //right minus left

    if (abs(error) < integralLimit) { //check for integrator saturation
      integral = integral + error * 0.1;
    } else {
      integral = 0;
    }

    if (abs(error) < 1) { //calibrate this later
      timer -= 100;
    } else {
      timer = 500;
    }

    u = Kp * error + Ki * integral; //calculate the control effort
    speed = (int)constrain(u, -500, 500);

    left_font_motor.writeMicroseconds(1500 - speed);
    left_rear_motor.writeMicroseconds(1500 - speed);
    right_rear_motor.writeMicroseconds(1500 - speed);
    right_font_motor.writeMicroseconds(1500 - speed);
    delay(100);
    IR1_dist = IR1_read();
    IR2_dist = IR2_read();
  }
  stop();
}
/*-------------------------Motion Function for when the first corner is a CW turn-------------------------*/
void CWcorner()
{
  //Corner number 1
  TurnByAngle(90);
  //Forward 22.5cm
  TurnByAngle(90);
  //2m straight
  driveToWall();
  //Corner number 2
  TurnByAngle(-90);
  //forward 22.5cm
  TurnByAngle(-90);
  //2m straight
  driveToWall();
  //Corner number 3
  TurnByAngle(90);
  //forward 22.5cm
  TurnByAngle(90);
  //2m straight
  driveToWall();
  //Corner number 4
  TurnByAngle(-90);
  //forward 22.5cm
  TurnByAngle(-90);
  //2m straight
  driveToWall();
}

/*---Motion Function for when the first corner is a CCW turn---*/
void CCWcorner()
{
  //Corner number 1
  TurnByAngle(-90);
  //forward 22.5cm
  TurnByAngle(-90);
  //2m straight
  driveToWall();
  //Corner number 2
  TurnByAngle(90);
  //forward 22.5cm
  TurnByAngle(90);
  //2m straight
  driveToWall();
  //Corner number 3
  TurnByAngle(-90);
  //forward 22.5cm
  TurnByAngle(-90);
  //2m straight
  driveToWall();
  //Corner number 4
  TurnByAngle(90);
  //forward 22.5cm
  TurnByAngle(90);
  //2m straight
  driveToWall();
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

/*-------------------------SENSORS-------------------------*/

//Sonar Sensor
#ifndef NO_HC-SR04
float HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  }// else {
   // SerialCom->print("HC-SR04:");
   // SerialCom->print(cm);
   // SerialCom->println("cm");
//  }

  return cm;
}
#endif

/*----------------------BATTERY------------------------*/

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

/*----------------------Motor moments------------------------*/


void disable_motors()
{
  left_font_motor.detach();     // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();     // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();    // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();    // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

// Continuously move platform forward.
void forward(float initialAngle)
{
  float finalAngle = gyro_read();
  float angleMoved = finalAngle - initialAngle;
  if (angleMoved > 90) {
    angleMoved = 360 - angleMoved;
  }
  //Serial.print("Angle moved value is: ");
  //Serial.println(angleMoved);
  int k = 10;
  //+VE IS CW
  left_font_motor.writeMicroseconds(1500 + (speed_val + angleMoved * k));
  left_rear_motor.writeMicroseconds(1500 + (speed_val + angleMoved * k));
  right_rear_motor.writeMicroseconds(1500 - (speed_val - angleMoved * k));
  right_font_motor.writeMicroseconds(1500 - (speed_val - angleMoved * k));
}

// Rotate platform by a specified angle in degrees using PI control (+ve input = clockwise, -ve input = counter-clockwise).
void TurnByAngle(int turnAngle)
{
  //Serial.println("Turning clockwise 90 degrees started...");
  //  currentAngle = 0;
  // Initialise variables
  float error, u, lastError, integral, derivative, currentAng, speed = 0;
  float integralLimit = 30; // Set max error boundary for integral gain to be applied to control system.
  float gyroAngle = gyro_read(); // Fetch and store current angle reading before turning.
  float initialAngle = gyro_read();
  float targetAngle = constrain(turnAngle, -180, 180); // Limit maximum turn angle to +ve or -ve 180 degrees.
  float Kp = 6; // Initialise proportional gain.
  float Ki = 0.05; // Initialise integral gain
  int timer = 500; // Initialise tolerance timer.
  int direction = turnAngle / abs(turnAngle);
  int wrapCheck = 0;
  Serial.println((String)"initial angle is: " + initialAngle + (String)", target angle is: " + targetAngle);
  while (timer > 0) {

    // Check if gyroscope angle reading wrapped to 0 while turning clockwise.
    if (((gyroAngle < initialAngle) && (turnAngle > 0))) {
      currentAng = gyroAngle - initialAngle + 360.0; // +360 degree angle correction to relative current angle reading
      wrapCheck = 0;
    }
    // Otherwise, check if gyroscope angle reading wrapped to 359 while turning counter-clockwise.
    else if (((gyroAngle > initialAngle) && (turnAngle < 0))) {
      currentAng = gyroAngle - initialAngle - 360.0; // -360 degree angle correction to relative current angle reading
      wrapCheck = 1;
    }
    else {
      currentAng = gyroAngle - initialAngle; // No angle correction required
      wrapCheck = 2;
    }

    error = targetAngle - currentAng; // Calculate error for desired angle.
    if (abs(error) > abs(targetAngle)) {
      error = (360 - abs(error)) * direction;
    }

    Serial.println((String)"CurrentAng is: " + currentAng + (String)", Error is: " + error + (String)", gyro reading is: " + gyroAngle + (String)", wrap check = " + wrapCheck);
    // Stop integrating if actuators are saturated.
    if (abs(error) < integralLimit) {
      integral = integral + error * 0.1; // Integrate the error with respect to loop frequency (~10Hz).
    }
    else {
      integral = 0; // Disable integral
    }

    // Calculate derivative of error.
    derivative =  error - lastError;
    lastError = error; // Update last error calculated.

    // Loop exits if error remains in steady state for at least 500ms.
    if ((derivative == 0) && (error < 5)) {
      timer -= 100;
    }
    else {
      timer = 500;
    }

    u = Kp * error + Ki * integral; // Calculate the control effort to reach target distance.
    speed = (int) constrain(u, -500, 500);
    //Note:
    left_font_motor.writeMicroseconds(1500 + speed);
    left_rear_motor.writeMicroseconds(1500 + speed);
    right_rear_motor.writeMicroseconds(1500 + speed);
    right_font_motor.writeMicroseconds(1500 + speed);

    delay(100); // Loop repeats at a frequency of ~10Hz

    gyroAngle = gyro_read(); // Get current angle reading from gyroscope sensor (range 0 to 359).
  }
  stop();
  //Serial.println("Turning clockwise 90 degrees stopped");
}
