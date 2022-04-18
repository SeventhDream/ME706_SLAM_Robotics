//=============================================================
//#pragma region 1. SETUP
//=============================================================
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h> //Need for Wireless module

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.


// Wireless Module Setup
#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin
#define OUTPUTMONITOR 0 // USB Serial Port
#define OUTPUTPLOTTER 0 // USB Serial Plotter
#define OUTPUTBLUETOOTHMONITOR 1 // Bluetooth Serial Port

//Servo motor setup
Servo servo1;
int posn = 90; //facing forward

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

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
int speed_val = 400;
int speed_change;
int pos = 0;

//IR Sensor Setup
int frontR_IR = A5;
int frontL_IR = A7;
int backL_IR = A4;
int backR_IR = A6;

//Gyroscope Setup
float sensorPin = A8;               //define the pin that gyro is connected
float T = 100;                       // T is the time of one loop
long previous_millis = 0;           // previous time stamp to calculate T.
int sensorValue = 0;                // read out value of sensor
float gyroSupplyVoltage = 5;        // supply voltage for gyro
float gyroZeroVoltage = 510;          // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
float gyroRate = 0;                 // read out value of sensor in voltage
float currentAngle = 0;            // current angle calculated by angular velocity integral on
bool turned = 0;


//Ultrasonic Sensor Setup
const int TRIG_PIN = 49;
const int ECHO_PIN = 48;
const unsigned int MAX_DIST = 23200; // Anything over 400 cm (2F3200 us pulse) is "out of range"

//Kalman filter setup
double last_var = 999;
double process_noise = 1;
double last_est_rightIR = 0;
double last_est_leftIR = 0;
double last_est_IR1 = 0;
double last_est_IR2 = 0;

//Setup for middle strafe
float y_distance = 15 + 22.5;

//Serial Pointer
HardwareSerial *SerialCom;

// Global Coordinate Variables
float x = 0;
float y = 0;
bool start_printing = 0; //set to 1 right before the first wall follow begins
bool global_isLeft = 0; // = 1 if the wall is on the left when the robot is in starting corner (needs to be implemented at the end of FindCorner())
bool servo_forward = 0;
int index = 0;

void setup(void)
{

  //turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  //Gyroscope Setup Start
  pinMode(sensorPin, INPUT);


  //Servo setup
  servo1.attach(A9);

  int i;
  float sum = 0;
  for (i = 0; i < 100; i++) //read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(sensorPin);
    sum += sensorValue;
    delay(5);
  }
  previous_millis = gyro_read();
  gyroZeroVoltage = sum / 100;   //average the sum as the zero drifting
  pinMode(TRIG_PIN, OUTPUT);     //the Trigger pin will tell the sensor to range find
  digitalWrite(TRIG_PIN, LOW);

  // Serial Port Setup
  SerialCom = &Serial;  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom->begin(115200);
  SerialCom->println("PROTOTYPE 18/03/2022");
  delay(1000);
  SerialCom->println("Setup....");

  BluetoothSerial.begin(115200);

  delay(1000); //settling time but not really needed
}
//#pragma endregion end

//=============================================================
//#pragma region 2. MAIN SECTION
//=============================================================
void loop(void) // Main Loop
{
  static STATE machine_state = INITIALISING; // Initialise machine state
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: // Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}

STATE initialising() {
  SerialCom->println("INITIALISING....");
  delay(1000); // One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  // Initialise variables
  float frontL[] = {0, 999};
  float frontR[] = {0, 999};
  float backL[] = {0, 999};
  float backR[] = {0, 999};
  float initAngle = gyro_read();
  float ultra = HC_SR04_range();
  static unsigned long previous_millis;
  fast_flash_double_LED_builtin();



  /*--------------------------------COURSE START--------------------------------*/

  BluetoothSerial.println("=============================================================");
  BluetoothSerial.println("Started the course.");
  BluetoothSerial.println("=============================================================");
//  StrafeDistance(15, false, initAngle); //strafe to the right
//  StrafeDistance(15, true, initAngle); //strafe to the right

//  float FL_IR_Data[]= {0,999};
//  while(1){
//    FL_IR(FL_IR_Data); // Front left IR sensor reading Broken sensor
//    FL_IR_Data[0]=constrain(FL_IR_Data[0],10,80); //Broken sensor
//  
//    BluetoothSerial.println((String)"Front left sensor value is: " + FL_IR_Data[0]);
//    delay(500);  // actual sensor reading=sensor reading+2.5cm
//  }

//  float FR_IR_Data[]= {0,999};
//  while(1){
//    FR_IR(FR_IR_Data); // Front left IR sensor reading Broken sensor
//    FR_IR_Data[0]=constrain(FR_IR_Data[0],10,80); //Broken sensor
//  
//    BluetoothSerial.println((String)"Front right sensor value is: " + FR_IR_Data[0]);
//    delay(500);  // actual sensor reading=sensor reading+1cm
//  }
  
    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("altFindCorner() start");
    BluetoothSerial.println("=============================================================");
    altFindCorner();

    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("WallFollowUltra() start");
    BluetoothSerial.println("=============================================================");
    WallFollowUltra();
    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("ServoFaceForward() start");
    BluetoothSerial.println("=============================================================");
    ServoFaceForward();
    delay(1000);
    //
    //
  
    global_isLeft = 1;
    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("altMiddleLogic2() start");
    BluetoothSerial.println("=============================================================");
    altMiddleLogic2();
  
  //  BluetoothSerial.println("=============================================================");
  //  BluetoothSerial.println("altMiddleLogic() start");
  //  BluetoothSerial.println("=============================================================");
  //  altMiddleLogic();
  

    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("WallFollowUltra() start");
    BluetoothSerial.println("=============================================================");
    WallFollowUltra();


  delay(3000);
  BluetoothSerial.println("STOPPED");
  return STOPPED;
  /*--------------------------------------COURSE END----------------------------------------*/

  if (millis() - previous_millis > 500) {  // Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    return RUNNING;
  }
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

//#pragma endregion end

//=============================================================
//#pragma region 3.1 COORDINATES
//=============================================================

void Coord() {
  x = 200 - (HC_SR04_range() + (12 / 2)); // 12 / 2 is distance between sonar and middle of robot (**TUNING NEEDED**)
  //BluetoothSerial.print((String)"X = " + x + (String)" Y = " + y); //Printing x and y.
}

void ISR1() {
  Coord();
  //BluetoothSerial.println("Coordinate Interrupt!");
  //BluetoothSerial.println("Coordinate Interrupt!");
  //BluetoothSerial.print("(x, y) = ");
  //BluetoothSerial.print(x);
  //BluetoothSerial.print(", ");
  //BluetoothSerial.print(y);
}

void CoordUpdate() {
  float FR_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  FR_IR(FR_IR_Data); // Front right IR sensor reading
  BL_IR(BL_IR_Data); // Back right IR sensor reading 
  FL_IR(FL_IR_Data); // Front left IR sensor reading Broken sensor

  FR_IR_Data[0]=constrain(FR_IR_Data[0],10,80);
  BL_IR_Data[0]=constrain(BL_IR_Data[0],4,30);
  FL_IR_Data[0]=constrain(FL_IR_Data[0],10,80); //Broken sensor
  //BluetoothSerial.println((String)"in coordsUpdate");
  
  if (servo_forward) {
    x = 200 - (HC_SR04_range() + (24 / 2)); // 12 / 2 is distance between sonar and middle of robot (**TUNING NEEDED**)
  }

//  //BluetoothSerial.println((String)"IR right is:" + FR_IR_Data[0] + (String)"IR left is:" + FL_IR_Data[0]);
//  if (!global_isLeft) {
//
//    //lol leaving out the right side for now haha.
//    if (FR_IR_Data[0] > 79) {    
//      y = 120 - (7.5 + FL_IR_Data[0]);
//    } else {
//      y = 7.5 + FR_IR_Data[0];
//    }
//  }
//  else {
//   // If front right is out of range, wall is closer on the left    
//   if (FR_IR_Data[0] > 79) {
//      if (BL_IR_Data[0] > 30){
//        // If back left is out of range estimate the y coordinates
//        // Only happens when we are in 30-40cm away from the left wall. 
//        y = 35; 
//      }else{
//        //Otherwise use the short range left IR to calculate the y coordinate
//        y = 120 - (7.5 + BL_IR_Data[0]);
//      }
//    } else { //else if front right is in range, calculate teh y coordinate based on the front right IR
//      y = 7.5 + FR_IR_Data[0];
//    }  
//  }  

   if (!global_isLeft) {
     if (FR_IR_Data[0] > 79) {
      //BluetoothSerial.println((String)"LEFT IR IS BEING USED");
        y = 120 - (7.5 + FL_IR_Data[0]+2.5);//2.5cm hard calibration for front left IR
     }else {
      //BluetoothSerial.println((String)"RIGHT IR IS BEING USED");
        y = 7.5 + FR_IR_Data[0]+1;//1cm hard calibrat5ion for front right IR
      }
  }
  else {
    if (FL_IR_Data[0] > 79) {
      BluetoothSerial.println((String)"RIGHT IR IS BEING USED");
      y = 120 - (7.5 + FR_IR_Data[0]+1);
    }else {
      y = 7.5 + FL_IR_Data[0]+2.5;
      BluetoothSerial.println((String)"LEFT IR IS BEING USED");
    }
  }

  if (start_printing) {
    //print to Putty serial monitor
    //BluetoothSerial.println("COORDINATES UPDATING!");
    BluetoothSerial.println((String)"X = " + x + (String)" Y = " + y);
  }
}
//#pragma endregion end

//=============================================================
//#pragma region 3.2 MOTOR MOVEMENT FUNCTIONS
//=============================================================
// alternative middle logic with hardcoded behaviour logic

void altMiddleLogic2() {
  float FR_IR_Data[] = {0, 999};
  FR_IR(FR_IR_Data);
  float FL_IR_Data[] = {0, 999};
  FL_IR(FL_IR_Data);
  float BR_IR_Data[] = {0, 999};
  BR_IR(BR_IR_Data);
  float BL_IR_Data[] = {0, 999};
  BL_IR(BL_IR_Data);

  currentAngle = 0;
  float iAngle = currentAngle;
  float forwardsmash = 5;
  float backwardsmash = -2.1;
  int del_f = 500;
  int del_b = 500;
  int del_c = 4950;
  float coord_mill = millis();
  float prev_mill = millis();

  float strafeT = 320;
  if (((FR_IR_Data[0] + BR_IR_Data[0]) / 2) > ((FL_IR_Data[0] + BL_IR_Data[0]) / 2)) {
    //wall is on the left
    //StrafeSonar(25, 1);
    StrafeTime(450, false, iAngle, 300);
    CoordUpdate();
    drive_forward(0, 0, 0, 150);
    delay(300);
    CoordUpdate();
    motorstop();
    //delay(200);
    //    iAngle = gyro_read();
    //    delay(200);

    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    CoordUpdate();
    motorstop();
    //delay(200);
    //iAngle = gyro_read();
    index++;


    //gyro_forward(forwardsmash, iAngle, 1);
    CoordUpdate();
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    CoordUpdate();
    motorstop();
    //delay(200);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    CoordUpdate();
    motorstop();
    //delay(200);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(forwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    CoordUpdate();
    motorstop();
    //delay(200);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    CoordUpdate();
    motorstop();
    //delay(200);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(forwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, false, iAngle, 300);
    CoordUpdate();
    //StrafeSonar(85, 1);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    CoordUpdate();
    motorstop();
   // delay(200);
    //iAngle = gyro_read();
    index++;
//
//    //gyro_forward(backwardsmash, iAngle, 1);
//    OpenDriveBackward();
//    delay(del_c);
//    StrafeTime(strafeT, false, iAngle, 300);
//    //StrafeSonar(95, 1);
//    drive_backward(0, 0, 0, 150);
//    delay(del_b);
//    motorstop();
//    //delay(200);
//    //iAngle = gyro_read();
//    index++;
//
//    //gyro_forward(forwardsmash, iAngle, 1);
//    OpenDriveForward();
//    delay(5500);
////    //StrafeTime(strafeT, false, iAngle);
////    drive_forward(0, 0, 0, 150);
////    delay(del_f);
//    motorstop();
////    delay(200);
//    //iAngle = gyro_read();
//    index++;

    CoordUpdate();
    ServoFaceLeft();
    CoordUpdate();
    TurnByAngle(170);
    turned = 1;

    StrafeDistance(15, true, iAngle); //false means right
    drive_backward(0, 0, 0, 150);
    delay(700);
    StrafeDistance(15, true, iAngle); //false means right
    
    //AlignToWall(false);//false is left, true is right

//    StrafeDistance(15, false, iAngle); //false means right
//    AlignToWall(true);//false is left, true is right
  }
  else {
    //wall is on the right
    //StrafeSonar(25, 0);
    StrafeTime(450, true, iAngle, 300);    
    drive_forward(0, 0, 0, 150);
    delay(305);
    //    motorstop();
   // delay(200);
    //    iAngle = gyro_read();

    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(375, true, iAngle, 300);
    //StrafeSonar(35, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    //delay(100);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(forwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, true, iAngle, 300);
    //StrafeSonar(45, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    //delay(100);
    //iAngle = gyro_read();
    index++;

    
    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, true, iAngle, 300);
    //StrafeSonar(55, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
   // delay(100);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(forwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, true, iAngle, 300);
    //StrafeSonar(65, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
   // delay(100);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(backwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveBackward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, true, iAngle, 300);
    //StrafeSonar(75, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
   // delay(100);
    //iAngle = gyro_read();
    index++;

    //gyro_forward(forwardsmash, iAngle, 1);
    prev_mill = millis();
    coord_mill = millis();
    while (millis() - prev_mill < del_c) {
      OpenDriveForward();
      if (millis() - coord_mill > 400) {
        CoordUpdate();
        coord_mill = millis();      
      }
    }
    StrafeTime(strafeT, true, iAngle, 300);
    //StrafeSonar(85, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
   // delay(100);
    //iAngle = gyro_read();
    index++;

//    //gyro_forward(backwardsmash, iAngle, 1);
//    OpenDriveBackward();
//    delay(del_c);
//    StrafeTime(strafeT, true, iAngle, 300);
//    //StrafeSonar(95, 0);
//    drive_backward(0, 0, 0, 150);
//    delay(del_b);
//    motorstop();
//  //  delay(100);
//    //iAngle = gyro_read();
//    index++;
//
//    // gyro_forward(forwardsmash, iAngle, 1);
//    OpenDriveForward();
//    delay(5500);
////    //StrafeTime(strafeT, true, iAngle);
////    drive_forward(0, 0, 0, 150);
////    delay(del_f);
//    motorstop();
////    delay(100);
//    //iAngle = gyro_read();
//    index++;

    CoordUpdate();
    ServoFaceRight();
    CoordUpdate();    

    ServoFaceRight();
    TurnByAngle(170);
    StrafeDistance(15, false, iAngle); //false means right
    drive_backward(0, 0, 0, 150);
    delay(700);
    StrafeDistance(15, false, iAngle); //false means right
    //AlignToWall(true);//false is left, true is right

//    StrafeDistance(15, true, iAngle); //false means right
//    AlignToWall(false);//false is left, true is right
  }
}

void altMiddleLogic() {
  float FR_IR_Data[] = {0, 999};
  FR_IR(FR_IR_Data);
  float FL_IR_Data[] = {0, 999};
  FL_IR(FL_IR_Data);
  float BR_IR_Data[] = {0, 999};
  BR_IR(BR_IR_Data);
  float BL_IR_Data[] = {0, 999};
  BL_IR(BL_IR_Data);

  float iAngle = gyro_read();
  float forwardsmash = 15;
  float backwardsmash = -2.2;
  int del_f = 2000;
  int del_b = 500;

  float strafeT = 400;
  if (((FR_IR_Data[0] + BR_IR_Data[0]) / 2) > ((FL_IR_Data[0] + BL_IR_Data[0]) / 2)) {
    //wall is on the left
    //StrafeSonar(25, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    drive_forward(0, 0, 0, 150);
    delay(700);
    motorstop();
    delay(100);
    iAngle = gyro_read();

    gyro_forward(backwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(35, 1);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;


    gyro_forward(forwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(45, 1);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(55, 1);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(65, 1);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(75, 1);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(85, 1);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    StrafeTime(strafeT, false, iAngle, 300);
    //StrafeSonar(95, 1);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    //StrafeTime(strafeT, false, iAngle);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    TurnByAngle(180);

    StrafeDistance(15, true, iAngle); //false means right
    AlignToWall(false);//false is left, true is right
  }
  else {
    //wall is on the right
    StrafeSonar(25, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();

    gyro_forward(backwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(35, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(45, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(55, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(65, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(75, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(85, 0);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(backwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    StrafeSonar(95, 0);
    drive_backward(0, 0, 0, 150);
    delay(del_b);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    gyro_forward(forwardsmash, iAngle, 1);
    //StrafeTime(strafeT, true, iAngle);
    drive_forward(0, 0, 0, 150);
    delay(del_f);
    motorstop();
    delay(100);
    iAngle = gyro_read();
    index++;

    TurnByAngle(180);

    StrafeDistance(15, false, iAngle); //false means right
    AlignToWall(true);//false is left, true is right
  }
}

void MiddleStrafe(bool Left, float iAngle) {
  int half_second_count = 0;
  float prev_millis = millis();
  int strafe_time = 1; //[seconds]
  //while (half_second_count < strafe_time * 2) { //need to be tuned
  if (Left) {
    //BluetoothSerial.println("Strafing Using Right Sesnors");
    if (y_distance < 60) {
      StrafeDistance(y_distance, false, iAngle);
    } else {
      StrafeDistance(y_distance, true, iAngle);
    }
  }
  else {
    //BluetoothSerial.println("Strafing Using Right Sesnors");
    if (y_distance < 60) {
      StrafeDistance(y_distance, true, iAngle);
    }
    else {
      StrafeDistance(y_distance, false, iAngle);
    }
  }
  //        y = y + (half_second_count * (22.5 / (strafe_time * 2)));
  //        if (millis() - prev_millis > 500) {
  //          prev_millis = millis();
  //          half_second_count++;
  //         }
  //}
  y_distance = y_distance + 22.5;
  motorstop();
}

void MiddleStrafe1(int isLeft, float iAngle) {
  StrafeDistance(22.5, isLeft, iAngle);

}

void altFindCorner() {
  //Initialise variables
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};
  float ultraDist = 0;
  float ultraDistLeft = 0;
  float ultraDistRight = 0;
  float iAngle = gyro_read();

  ultraDist = HC_SR04_range(); // Get initial sonar reading.

  //Orientate the robot to face a wall 60cm away
  //BluetoothSerial.println("Orientate the robot to face a wall 60cm away");
  while (ultraDist > 60) {
    //BluetoothSerial.println((String)"Sonar Reading: " + ultraDist);
    cw();
    ultraDist = HC_SR04_range();
    delay(50);
  }
  motorstop(); // stop rotating.

  delay(1000);

  // Check distance from left/right walls using sonar.
  ServoFaceLeft();
  delay(500);
  ultraDistLeft = HC_SR04_range();
  delay(500);
  ServoFaceRight();
  delay(500);
  ultraDistRight = HC_SR04_range();
  ServoFaceForward();

  iAngle = gyro_read();

  //Check if left wall is closer
  if (ultraDistLeft < ultraDistRight) {
    //BluetoothSerial.println("Strafe into left wall!");
    StrafeTime(3000, true, iAngle, 300);
    iAngle = gyro_read();
    //AlignToWall(false);
    //StrafeTime(150, false, iAngle, 250);
  }
  else {
    BluetoothSerial.println("Strafe into left wall!");
    StrafeTime(3000, false, iAngle, 300);
    iAngle = gyro_read();
    //StrafeTime(150, true, iAngle, 250);
    //AlignToWall(true);
  }
  //delay(500);
//  BluetoothSerial.println("Gyro forward start");
//  OpenDriveForward();
//  delay(1500);
//  motorstop();
  gyro_forward(15, iAngle, 1);
  //BluetoothSerial.println("Gyro forward end");

  // Check distance from left/right walls using sonar.

  ServoFaceLeft();
  delay(300);
  ultraDistLeft = HC_SR04_range();
  delay(300);
  ServoFaceRight();
  delay(300);
  ultraDistRight = HC_SR04_range();
  delay(300);
  ServoFaceForward();


  if (ultraDistRight > ultraDistLeft) {
    TurnByAngle(85);
  }
  else {
    TurnByAngle(-85);
  }

  ultraDist = HC_SR04_range();

  if (ultraDist < 120) {
    if (ultraDistRight > ultraDistLeft) {
      TurnByAngle(85);
    }
    else {
      TurnByAngle(-85);
    }


  }
  BluetoothSerial.println("I THINK I AM IN A CORNER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  ServoFaceLeft();
  delay(500);
  ultraDistLeft = HC_SR04_range();
  delay(500);
  ServoFaceRight();
  delay(500);
  ultraDistRight = HC_SR04_range();
  delay(500);
  ServoFaceForward();

  iAngle = gyro_read();
  if (ultraDistRight > ultraDistLeft) {
    ServoFaceLeft();
    BluetoothSerial.println("strafe to the left!");
    StrafeDistance(15, true, iAngle);
    //    AlignToWall(false);
    drive_backward(0, 0, 0, 150);
    delay(700);
    motorstop();
    StrafeDistance(15, true, iAngle);
  }
  else {
    ServoFaceRight();
    BluetoothSerial.println("strafe to the right!");
    StrafeDistance(15, false, iAngle);
    //    AlignToWall(true);
    drive_backward(0, 0, 0, 150);
    delay(700);
    motorstop();
    StrafeDistance(15, false, iAngle);
  }

  
  BluetoothSerial.println("Alt find corner finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

void SonarFaceWall() {
  float ultraDistRight = 0;
  float ultraDistLeft = 0;

  ServoFaceLeft();
  delay(500);
  ultraDistLeft = HC_SR04_range();
  delay(500);
  ServoFaceRight();
  delay(500);
  ultraDistRight = HC_SR04_range();
  delay(500);
  ServoFaceForward();

  if (ultraDistLeft < ultraDistRight) {
    ServoFaceLeft();
  }
  else {
    ServoFaceRight();
  }
}

void FindCorner()
{
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  float ultraDist = HC_SR04_range();

  float iAngle = gyro_read();


  //Orientate the robot to face a wall 60cm away
  while (ultraDist > 60) {
    //BluetoothSerial.println("Orientate the robot to face a wall 60cm away");
    cw();
    ultraDist = HC_SR04_range();
    delay(50);
  }
  motorstop();

  delay(500);

  //BluetoothSerial.println("Drive straight until 20cm from front-facing wall");
  gyro_forward(25, 0, 0);
  iAngle = gyro_read();
  //SonarDistance(20, iAngle, false); // Drive straight until 15cm from front-facing wall

  motorstop();

  Localise();
  //Rereading sensor values
  BR_IR(BR_IR_Data);
  BL_IR(BL_IR_Data);

  iAngle = gyro_read();

  //BluetoothSerial.println("=============================================================");
  //BluetoothSerial.println((String)"back right is" + BR_IR_Data[0]);
  //BluetoothSerial.println("=============================================================");  // Check which sides of the robot are facing the wall
  if (BR_IR_Data[0] < 20) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Wall is on the right, turning CCW");
    //BluetoothSerial.println("=============================================================");
    TurnByAngle(-80);
    iAngle = gyro_read();
    delay(100);
    ultraDist = HC_SR04_range();
    //BluetoothSerial.println((String)"Ultrasonic reading is:" + ultraDist);
    if (ultraDist > 120) {
      StrafeDistance(15, false, iAngle);
      //AlignDelay(true);
      AlignToWall(true);
    }
    else {
      TurnByAngle(-80);
      iAngle = gyro_read();
      StrafeDistance(15, true, iAngle);
      //AlignDelay(false);
      AlignToWall(false);
    }
  } else {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Wall is on the left, turning CW");
    //BluetoothSerial.println("=============================================================");
    TurnByAngle(80);
    iAngle = gyro_read();
    delay(100);

    ultraDist = HC_SR04_range();
    //BluetoothSerial.println((String)"Ultrasonic reading is:" + ultraDist);
    if (ultraDist > 120) {
      StrafeDistance(15, true, iAngle);
      //AlignDelay(false);
      AlignToWall(false);
    }
    else {
      TurnByAngle(80);
      iAngle = gyro_read();
      StrafeDistance(15, false, iAngle);
      //AlignDelay(true);
      AlignToWall(true);
    }
  }
  //BluetoothSerial.println("Ready to START MAPPING!");
  motorstop();
}

void AlignDelay(bool isWallOnRight) {
  if (isWallOnRight) { //Wall is on the right
    ServoFaceRight();
    delay(5000);
  }
  else { //Wall is on the left
    ServoFaceLeft();
    delay(5000);
  }
}



void WallFollowUltra() {
  float ultraFront = HC_SR04_range();
  float initialAngle = gyro_read();
  float angleMoved = 0;
  float GyroAngle = 0;
  float ultraSide = 0;
  float leftVar = 0;
  float ultraSidePrint = 0;
  float error_top = 0;
  float error_short = 0;
  float speed_top = 0;
  float speed_short = 0;
  float speed_gyro = 0;
  float travel_angle = 0;
  float target = 15 ;
  float short_IR = 0;
  float long_IR = 0;
  float strafe_thresh = 10; //if teh robot is more than 10cm away from the target distance, robot will strafe.
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};
  float top_feedback[] = {0, 500};
  // float long_feedback[] = {0, 500};
  float short_feedback[] = {0, 500};
  float gyro_feedback[] = {0, 500};
  float Ultra_Data[] = {0, 999};
  float motor_speed = 150;

  BR_IR(BR_IR_Data);
  BL_IR(BL_IR_Data);
  ULTRA_DIST(Ultra_Data);

 // BluetoothSerial.println((String)"PLEASE PRINT" + start_printing);
  if (start_printing == 0) {
    start_printing = 1;
    x = 0;
    y = 0;
   // BluetoothSerial.println((String)"in print = 1 loop");
  }

  if (BR_IR_Data[0] < BL_IR_Data[0]) { //indicates whether the wall is on left side or right side
    global_isLeft = 0;
  } else {
    global_isLeft = 1;
  }

  float prev_millis = millis();
  float starting = millis();

  while ((millis() - starting) <  10000) { //ultraFront > 15
    if (millis() - prev_millis > 400) {
      BluetoothSerial.println((String)"X = " + x + (String)" Y = " + y);
      prev_millis = millis();
    }

    //Rereading sensor values
    BR_IR(BR_IR_Data);
    BL_IR(BL_IR_Data);
    ULTRA_DIST(Ultra_Data);

    if (turned) {
      x = 200 - (12 + ((millis() - starting) / 10000) * 170);
      y = 120 - (Ultra_Data[0] + 2);
    }
    else {
      x = 12 + ((millis() - starting) / 10000) * 170;
      y = Ultra_Data[0] + 2;
    }

    travel_angle = gyro_read();
    //wrap travel angle
    if (travel_angle > 90) {
      travel_angle = travel_angle - 360;
    }
    //Setting up interupt to start printing coordinates every 0.5sec

    if (BR_IR_Data[0] < BL_IR_Data[0]) { //indicates whether the wall is on left side or right side
      leftVar = -1;
      ultraSide = Ultra_Data[0];
      short_IR = BR_IR_Data[0];
    }
    else {
      leftVar = 1;
      ultraSide = Ultra_Data[0];
      short_IR = BL_IR_Data[0];
    }

    GyroAngle = gyro_read();
    //wrap angle moved
    if (GyroAngle > 90) {
      angleMoved = (360 - GyroAngle) - initialAngle;
    } else {
      angleMoved = GyroAngle - initialAngle;
    }

    //Calculate errors FOR SPEED 350
    if ( leftVar == -1) {
      ultraSidePrint = ultraSide - 5.89;
      //controllers for speed at 100
      error_top = target - (7.09 + ultraSidePrint); //have to measure this, last time it was 8.76-3.52
      error_short = target - (7.09 + short_IR);
      controller(error_top, 2, 1.7, 0.005, 2, 0.5, top_feedback);
      controller(error_short, 4, 1.7, 0.005, 2, 0.5, short_feedback);

    } else {
      ultraSidePrint = ultraSide - 6.47;
//      ultraSidePrint = ultraSide - 8.47;
      error_top = target - (7.15 + ultraSidePrint); //have to measure this, last time it was 8.76-3.52
      error_short = target - (7.15 + short_IR);
//      controller(error_top, 35,3.75, 0.005, 2, 0.5, top_feedback);
//      controller(error_short,1, 0.5, 0, 2, 0.5, short_feedback);

      controller(error_top, 30.5, 3.4, 0.005, 2, 0.5, top_feedback);
      controller(error_short,1, 0.5, 0.01, 2, 0.5, short_feedback);
      
//      //For speed 100 and battery level about 70%
//      controller(error_top, 30, 3.4, 0.5, 2, 0.5, top_feedback);
//      controller(error_short,1, 0, 0, 2, 0.5, short_feedback);
      
//      //For speed 100 and battery level about 70%
//      controller(error_top, 30.5, 3.4, 0.005, 2, 0.5, top_feedback);
//      controller(error_short,1, 0.5, 0.01, 2, 0.5, short_feedback);
      //      controller(error_top, 100, 0, 0.05, 1, 0.5, top_feedback);
      //      //controller(error_short, 2, 0.05, 0, 1, 0.5, short_feedback);
    }

    //    //Calculate errors FOR SPEED 150
    //    //Error top means the error between ultra and wall
    //    if ( leftVar == -1) {
    //      ultraSidePrint = ultraSide - 5.89;
    //      //controllers for speed at 100
    //      error_top = target - (7.09 + ultraSidePrint); //have to measure this, last time it was 8.76-3.52
    //      error_short = target - (7.09 + short_IR);
    //
    //      controller(error_top, 10, 0.1, 0, 2, 0.5, top_feedback);
    //      controller(error_short, 20, 0.1, 0, 2, 0.5, short_feedback);
    //    } else {
    //      ultraSidePrint = ultraSide - 6.47;
    //      error_top = target - (7.15 + ultraSidePrint); //have to measure this, lawst time it was 8.76-3.52
    //      error_short = target - (7.15 + short_IR);
    //      controller(error_top, 25, 0.1, 0, 2, 0.5, top_feedback);
    //      controller(error_short, 2.5, 0.03, 0, 2, 0.5, short_feedback);
    //    }

    //BluetoothSerial.println((String)"left is " + leftVar + (String)", ultraSide is:  " + ultraSidePrint + (String)", error_top is: " + error_top + (String)", Short IR is: " + short_IR + (String)", error_short is: " + error_short);

    speed_top = constrain(top_feedback[0], -1 * (500 - motor_speed), (500 - motor_speed));
    speed_short = constrain(short_feedback[0], -1 * (500 - motor_speed), (500 - motor_speed));

    ////BluetoothSerial.println((String)" long IR timer is " + long_feedback[0] + (String)" Short IR timer is" + short_feedback[1]);
    //If errors are small enough fluctuating between positive and negative, make the right and left motors same power

    if ( (abs(error_top) < 0.4) && (abs(error_short) < 0.4) ) {
      //BluetoothSerial.println("gyro drive forward!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      drive_forward(5, 0, 0, 150);
    } else {
      if ((leftVar == 1) && (short_IR > ultraSidePrint)) { //Top left
        //BluetoothSerial.println("top left");
        //BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_short + (String)" Left Side = " + speed_top);
        drive_forward(abs(speed_top), speed_short, 0, motor_speed);
      }
      else if ((leftVar == 1) && (short_IR < ultraSidePrint)) { //bottom left
       // BluetoothSerial.println("bottom left");
       // BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_short + (String)" Left Side = " + speed_top);
        drive_forward(speed_top, abs(speed_short), 0, motor_speed);
      }
      else if ((leftVar == -1) && (short_IR > ultraSidePrint)) { //top right
        //BluetoothSerial.println("top right");
        //BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_top + (String)" Left Side = " + speed_short);
        drive_forward(speed_short, abs(speed_top), 0, motor_speed);
      }
      else { //bottom right
        //BluetoothSerial.println("bottom right");
        //BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_top + (String)" Left Side = " + speed_short);
        drive_forward(abs(speed_short), speed_top, 0, motor_speed);
      }
    }
  }
  motorstop();
}

void WallFollow2() {
  float ultra = HC_SR04_range();
  float initialAngle = gyro_read();
  float angleMoved, GyroAngle, ultraSidePrint = 0;
  float Ultra_Data[] = {0, 999};

  float leftVar, error_top, error_short, error, long_IR, short_IR, travel_angle, left, adjustment = 0;
  float target = 7.5 ;

  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  //controller arrays
  float feedback[] = {0, 500};


  // Determining if the wall is on the left or right
  //Serial.println((String)"Initial IR distances are: " + (String)" IR Long Right = " + FR_IR_Data[0] + (String)" IR Long Left = " + FL_IR_Data[0] + (String)" IR Short Right = " + BR_IR_Data[0] + (String) " IR Short Left = " + );
  // Closed loop controls
  // While long and short timers have not settled at same error for more than 500ms and ultra is more than 15cm away.
  while (1) {
    //Rereading sensor values
    FR_IR(FR_IR_Data);
    FL_IR(FL_IR_Data);
    BR_IR(BR_IR_Data);
    BL_IR(BL_IR_Data);
    //ultra = HC_SR04_range();
    travel_angle = gyro_read();
    ULTRA_DIST(Ultra_Data);

    //wrap travel angle
    if (travel_angle > 90) {
      travel_angle = travel_angle - 360;
    }

    //Setting up interupt to start printing coordinates every 0.5sec
    if (start_printing = 0) {
      start_printing = 1;
      x = 0;
      y = 0;
    }

    if (BR_IR_Data[0] < BL_IR_Data[0]) { //indicates whether the wall is on left side or right side
      //Serial.println("Wall is on the right!");
      left = -1;
      long_IR = Ultra_Data[0];
      short_IR = BR_IR_Data[0];
    }
    else {
      //Serial.println("Wall is on the left!");
      left = 1;
      long_IR = Ultra_Data[0];
      short_IR = BL_IR_Data[0];
    }
    //Serial.println((String)"IR distances are: " + (String)" IR Long Right = " + FR_IR_Data[0] + (String)" IR Long Left = " + FL_IR_Data[0] + (String)" IR Short Right = " + BR_IR_Data[0] + (String) " IR Short Left = " + BL_IR_Data[0]);

    GyroAngle = gyro_read();

    //wrap angle moved
    if (GyroAngle > 90) {
      angleMoved = (360 - GyroAngle) - initialAngle;
    } else {
      angleMoved = GyroAngle - initialAngle;
    }
    //Calculate errors

    //Error top means the error between ultra and wall
    if (leftVar == -1) {
      error_top = target + 5.04 - Ultra_Data[0]; //have to measure this, lawst time it was 8.76-3.52
      ultraSidePrint =  Ultra_Data[0] - 5.04;
    } else {
      error_top = target + 5.64 - Ultra_Data[0];
      ultraSidePrint = Ultra_Data[0] - 5.64;
    }

    error_short = target - short_IR;
    //Serial.println((String)"Errors are: " + (String)" Long IR = " + error_long + (String)" Short IR = " + error_short);

    //    if( abs(error_long)<1 && abs(error_short)<1){
    //      controller(error_long, 5, 0, 0, 2, 0.5, long_feedback);
    //      controller(error_short,5, 0, 0, 2, 0.5, short_feedback);
    //    }else{
    //      controller(error_long, 5, 0.1, 0, 2, 0.5, long_feedback);
    //      controller(error_short,5, 0.1, 0, 2, 0.5, short_feedback);
    //      //controller(angleMoved,10,0.015,0.005,1,1,gyro_feedback);
    //      controller(angleMoved, 20, 0, 0, 1, 1, gyro_feedback);
    //    }

    if ( abs(error_top) < 0.5 && abs(error_short) < 0.5) {
      //BluetoothSerial.println("gyro drive!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      drive_forward(0, 0, 0, 150);
    } else {
      error = ((Ultra_Data[0] - short_IR) + sin(angleMoved * 3.1415926 / 180)) / 2;
      //error=((long_IR-short_IR)+sin(angleMoved*3.1415926/180))/2;
      //BluetoothSerial.println((String)"overall error is : " + error + (String)"Sonar distance is: " + ultraSidePrint + (String)", top Error is: " + error_top + (String)"Short IR is: " + short_IR + (String)", Error short is: " + error_short);

      //      controller(error, 10,10, 0.05, 1, 0.5, feedback);
      //      controller(error, 15,8, 0.05, 0.8, 0.5, feedback);
      //controller(error, 20, 1.2, 0.05, 0.8, 0.5, feedback);
      controller(error, 50, 0, 0, 0.8, 0.5, feedback);

      adjustment = constrain(feedback[0], -500, 500);
      drive_forward(0, 0, adjustment, 150);

      ////BluetoothSerial.println((String)" long IR timer is " + long_feedback[0] + (String)" Short IR timer is" + short_feedback[1]);
      ////BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_long + (String)" Left Side = " + speed_short);
    }
  }
}

void WallFollow3() {
  float ultra = HC_SR04_range();
  float initialAngle = gyro_read();
  float angleMoved, GyroAngle = 0;

  float error_long, error_short, error, long_IR, short_IR, travel_angle, left, adjustment = 0;
  float target = 7.5 ;

  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  //controller arrays
  float feedback[] = {0, 500};


  // Determining if the wall is on the left or right
  //Serial.println((String)"Initial IR distances are: " + (String)" IR Long Right = " + FR_IR_Data[0] + (String)" IR Long Left = " + FL_IR_Data[0] + (String)" IR Short Right = " + BR_IR_Data[0] + (String) " IR Short Left = " + );
  // Closed loop controls
  // While long and short timers have not settled at same error for more than 500ms and ultra is more than 15cm away.
  while (ultra > 15) {
    //Rereading sensor values
    FR_IR(FR_IR_Data);
    FL_IR(FL_IR_Data);
    BR_IR(BR_IR_Data);
    BL_IR(BL_IR_Data);
    ultra = HC_SR04_range();
    travel_angle = gyro_read();

    //wrap travel angle
    if (travel_angle > 90) {
      travel_angle = travel_angle - 360;
    }

    //Setting up interupt to start printing coordinates every 0.5sec
    if (start_printing = 0) {
      start_printing = 1;
      x = 0;
      y = 0;
    }

    if ((FR_IR_Data[0] - target) < (FL_IR_Data[0] - target)) { //indicates whether the wall is on left side or right side
      //Serial.println("Wall is on the right!");
      left = -1;
      long_IR = FR_IR_Data[0];
      short_IR = BR_IR_Data[0];
    }
    else {
      //Serial.println("Wall is on the left!");
      left = 1;
      long_IR = FL_IR_Data[0];
      short_IR = BL_IR_Data[0];
    }
    //Serial.println((String)"IR distances are: " + (String)" IR Long Right = " + FR_IR_Data[0] + (String)" IR Long Left = " + FL_IR_Data[0] + (String)" IR Short Right = " + BR_IR_Data[0] + (String) " IR Short Left = " + BL_IR_Data[0]);

    GyroAngle = gyro_read();

    //wrap angle moved
    if (GyroAngle > 90) {
      angleMoved = (360 - GyroAngle) - initialAngle;
    } else {
      angleMoved = GyroAngle - initialAngle;
    }
    //Calculate errors
    error_long = target - long_IR;
    error_short = target - short_IR;
    //Serial.println((String)"Errors are: " + (String)" Long IR = " + error_long + (String)" Short IR = " + error_short);

    //    if( abs(error_long)<1 && abs(error_short)<1){
    //      controller(error_long, 5, 0, 0, 2, 0.5, long_feedback);
    //      controller(error_short,5, 0, 0, 2, 0.5, short_feedback);
    //      controller(angleMoved, 20, 0, 0, 1, 1, gyro_feedback);
    //    }else{
    //      controller(error_long, 5, 0.1, 0, 2, 0.5, long_feedback);
    //      controller(error_short,5, 0.1, 0, 2, 0.5, short_feedback);
    //      //controller(angleMoved,10,0.015,0.005,1,1,gyro_feedback);
    //      controller(angleMoved, 20, 0, 0, 1, 1, gyro_feedback);
    //    }

    if ( abs(error_long) < 0.5 && abs(error_short) < 0.5) {
      //BluetoothSerial.println("gyro drive!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      drive_forward(0, 0, 0, 150);
    } else {
      error = ((error_long + error_short) + sin(angleMoved * 3.1415926 / 180)) / 2;
      //error=((long_IR-short_IR)+sin(angleMoved*3.1415926/180))/2;
      //BluetoothSerial.println((String)"overall error is : " + error + (String)"Current Long IR is: " + long_IR + (String)", Error is: " + error_long + (String)"Current Short IR is: " + short_IR + (String)", Error is: " + error_short);

      //      controller(error, 10,10, 0.05, 1, 0.5, feedback);
      //      controller(error, 15,8, 0.05, 0.8, 0.5, feedback);
      controller(error, 20, 1.2, 0.05, 0.8, 0.5, feedback);

      adjustment = constrain(feedback[0], -500, 500);
      drive_forward(0, 0, -1 * adjustment, 150);

      ////BluetoothSerial.println((String)" long IR timer is " + long_feedback[0] + (String)" Short IR timer is" + short_feedback[1]);
      ////BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_long + (String)" Left Side = " + speed_short);
    }
  }
}


void AlignToWall(boolean isWallOnRight) {
  //BluetoothSerial.println("=============================================================");
  //BluetoothSerial.println("ALIGN TO WALL START");
  //BluetoothSerial.println("=============================================================");

  //FALSE MEANS WALL IS ON THE LEFT
  float error = 0;
  float u = 0;
  float lastError = 0;
  float integralReading = 0;
  float ultraSidePrint = 0;
  float derivative = 0;
  float integralLimit = 10; // Set max error boundary for integral gain to be applied to control system.
  float initialAngle = gyro_read();
  float effort = 0;
  float Kp = 40; // Initialise proportional gain.
  float Ki = 10; // Initialise integral gain
  int timer = 500; // Initialise tolerance timer.
  float F_IR_Data[] = {0, 999};
  float B_IR_Data[] = {0, 999};
  float ultra_Data[] = {0, 999};
  float backL = 0;
  int directionvar = 0;
  integralReading = 0;

  if (isWallOnRight) { //Wall is on the right
    ServoFaceRight();
  }
  else { //Wall is on the left
    ServoFaceLeft();

  }
  delay(5000);

  while ((timer > 0) && ((F_IR_Data[0] < 30) || (B_IR_Data[0] < 30))) {
    if (isWallOnRight) { //Wall is on the right
      ////BluetoothSerial.println("In align to wall, the wall is on the right");
      FR_IR(F_IR_Data); // Front IR sensor reading
      BR_IR(B_IR_Data); // Back IR sensor reading
      ULTRA_DIST(ultra_Data); //Front ultra sensor reading
      directionvar = 1;
      ultraSidePrint = ultra_Data[0] - 4.74;
      error = ultraSidePrint - B_IR_Data[0]; // Error is difference between readings
    }
    else { //Wall is on the left
      ////BluetoothSerial.println("In align to wall, the wall is on the left");
      FL_IR(F_IR_Data); // Front IR sensor reading
      BL_IR(B_IR_Data); // Back IR sensor reading
      ULTRA_DIST(ultra_Data); //Front ultra sensor reading
      directionvar = -1;
      ultraSidePrint = ultra_Data[0] - 5.64;
      error = ultraSidePrint - B_IR_Data[0]; // Error is difference between readings
    }
    /*    //Error top means the error between ultra and wall
      if (leftVar == -1) {//right
      ultraSidePrint = ultraSide - 4.74;
      controller(error_top, 2.4, 2, 0.005, 2, 0.5, top_feedback);
      controller(error_short, 2.4, 2, 0.005, 2, 0.5, short_feedback);
      } else { //left
      ultraSidePrint = ultraSide - 5.64;
      controller(error_top, 2.4, 2.4, 0.005, 2, 0.5, top_feedback);
      controller(error_short, 2.4, 2.4, 0.005, 2, 0.5, short_feedback);
      }*/
    //BluetoothSerial.println((String) "Front ultra: " + ultraSidePrint + (String)" Back IR: " + B_IR_Data[0] + (String)" error: " + error + (String)" timer: " + timer+(String)" integral before changing: " + integralReading);
    // Stop integrating if actuators are saturated.
    if (abs(error) < integralLimit) {
      integralReading = integralReading + error * 0.1; // Integrate the error with respect to loop frequency (~10Hz).
    }
    else {
      integralReading = 0; // Disable integral
    }

    // Calculate derivative of error.
    derivative =  error - lastError;
    lastError = error; // Update last error calculated.

    // Loop exits if error remains in steady state for at least 500ms.
    if ((derivative < 1) && (abs(error) < 0.4)) {
      timer -= 100;
      // //BluetoothSerial.println((String) "TIMER TESTING FRONT IR: " + F_IR_Data[0] + (String)" Back IR: " + B_IR_Data[0] + (String) "error: " + error + (String)" Integral is: " + integral + (String)" d: " + derivative);
    }
    else {
      timer = 300;
    }

    //    if (abs(error) > 20) {
    //      //BluetoothSerial.println("Error is greater than 8");
    ////      Kp = 8.2;
    ////      Ki = 1.2;
    //      Kp = 5;
    //      Ki =1;
    //    } else {
    //      //Kp = 20; // Initialise proportional gain.
    //      //Ki = 3; // Initialise integral gain
    //      Kp = 13.5; // Initialise proportional gain. Ku=30 Pu=2
    //      Ki = 8; // Initialise integral gain
    //    }


    u = Kp * error + Ki * integralReading; // Calculate the control effort to reach target distance.
    effort = constrain(u, -150, 150);



    // //BluetoothSerial.println((String) "error: " + error + (String)" u: " + effort + (String)" integral: " + integralReading);

    left_font_motor.writeMicroseconds(1500 +  directionvar * effort);
    left_rear_motor.writeMicroseconds(1500 + directionvar * effort);
    right_rear_motor.writeMicroseconds(1500 + directionvar * effort);
    right_font_motor.writeMicroseconds(1500 + directionvar * effort);
    delay(100); // Loop repeats at a frequency of ~10Hz
  }

  motorstop();
  ServoFaceForward();
  delay(500);
  //BluetoothSerial.println("=============================================================");
  //BluetoothSerial.println("ALIGN TO WALL END");
  //BluetoothSerial.println("=============================================================");
}

// Strafe left at fixed speed value.
void strafe_left (float u, float speedval)
{
  u = constrain(u, -1 * speedval, speedval);

  left_font_motor.writeMicroseconds(1500 - speedval - u);
  left_rear_motor.writeMicroseconds(1500 + speedval + u);
  right_rear_motor.writeMicroseconds(1500 + speedval + u);
  right_font_motor.writeMicroseconds(1500 - speedval - u);
}

// Strafe right at fixed speed value.
void strafe_right (float u, float speedval)
{
  u = constrain(u, -1 * speedval, speedval);
  left_font_motor.writeMicroseconds(1500 + speedval + u);
  left_rear_motor.writeMicroseconds(1500 - speedval - u - 80);
  right_rear_motor.writeMicroseconds(1500 - speedval - u);
  right_font_motor.writeMicroseconds(1500 + speedval + u);
}

// Pivot counter clockwise at a fixed speed value
void ccw_that_fixes_itself (float u)
{
  left_font_motor.writeMicroseconds(1500 - 80);
  left_rear_motor.writeMicroseconds(1500 - 80);
  right_rear_motor.writeMicroseconds(1500 - 80);
  right_font_motor.writeMicroseconds(1500 - 80);
}

void cw_that_fixes_itself (float u)
{
  left_font_motor.writeMicroseconds(1500 + 80);
  left_rear_motor.writeMicroseconds(1500 + 80);
  right_rear_motor.writeMicroseconds(1500 + 80);
  right_font_motor.writeMicroseconds(1500 + 80);
}


void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void drive_test ()
{
  left_font_motor.writeMicroseconds(1500 + 200);
  left_rear_motor.writeMicroseconds(1500 + 200);
  right_rear_motor.writeMicroseconds(1500 - 100);
  right_font_motor.writeMicroseconds(1500 - 100);
}

void drive_forward(float adjustment1, float adjustment2, float correction, float speedval) {
  adjustment1 = constrain(adjustment1, -1 * speedval, speedval);
  adjustment2 = constrain(adjustment2, -1 * speedval, speedval);
  correction = constrain(correction, -1 * speedval, speedval);
  //BluetoothSerial.println((String)"adjustment 1 is: " + adjustment1 + (String)"adjustment 2 is: " + adjustment2 + (String)"Correction is:" + correction);

  left_font_motor.writeMicroseconds(1500 + (speedval + adjustment1 - correction));
  left_rear_motor.writeMicroseconds(1500 + (speedval + adjustment1 - correction));
  right_rear_motor.writeMicroseconds(1500 - (speedval + adjustment2 + correction));
  right_font_motor.writeMicroseconds(1500 - (speedval + adjustment2 + correction));
}

void drive_backward(float adjustment1, float adjustment2, float correction, float speedval) {
  adjustment1 = constrain(adjustment1, -1 * speedval, speedval);
  adjustment2 = constrain(adjustment2, -1 * speedval, speedval);
  correction = constrain(correction, -1 * speedval, speedval);
  //BluetoothSerial.println((String)"adjustment 1 is: " + adjustment1 + (String)"adjustment 2 is: " + adjustment2 + (String)"Correction is:" + correction);

  left_font_motor.writeMicroseconds(1500 - (speedval + adjustment1 + correction));
  left_rear_motor.writeMicroseconds(1500 - (speedval + adjustment1 + correction));
  right_rear_motor.writeMicroseconds(1500 + (speedval + adjustment2 - correction));
  right_font_motor.writeMicroseconds(1500 + (speedval + adjustment2 - correction));
}

//void drive_forward(float adjustment1, float adjustment2, float correction, float speedval) {
//  float overallspeed1=(speedval + adjustment1 - correction);
//  float overallspeed2=(speedval + adjustment2 + correction);
//  adjustment1 = constrain(overallspeed1, -500, 500);
//  adjustment2 = constrain(overallspeed2, -500, 500);
//  //BluetoothSerial.println((String)"adjustment 1 is: " + adjustment1 + (String)"adjustment 2 is: " + adjustment2 + (String)"Correction is:" + correction);
//
//  left_font_motor.writeMicroseconds(1500 + overallspeed1);
//  left_rear_motor.writeMicroseconds(1500 + overallspeed1);
//  right_rear_motor.writeMicroseconds(1500 - overallspeed2);
//  right_font_motor.writeMicroseconds(1500 - overallspeed2);
//}
//
//void drive_backward(float adjustment1, float adjustment2, float correction, float speedval) {
//  float overallspeed1=(speedval + adjustment1 + correction);
//  float overallspeed2=(speedval + adjustment2 - correction);
//  adjustment1 = constrain(overallspeed1, -500, 500);
//  adjustment2 = constrain(overallspeed2, -500, 500);
//  //BluetoothSerial.println((String)"adjustment 1 is: " + adjustment1 + (String)"adjustment 2 is: " + adjustment2 + (String)"Correction is:" + correction);
//
//  left_font_motor.writeMicroseconds(1500 - overallspeed1);
//  left_rear_motor.writeMicroseconds(1500 - overallspeed1);
//  right_rear_motor.writeMicroseconds(1500 + overallspeed2);
//  right_font_motor.writeMicroseconds(1500 + overallspeed2);
//}


//Drive straight until hitting a wall
void driveToWall()
{
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  Serial.println("Drive to Wall Started");
  FR_IR(FR_IR_Data);
  FL_IR(FL_IR_Data);
  float initAngle = gyro_read();

  while ((FR_IR_Data[0] > 15) && (FL_IR_Data[0] > 15)) {
    Serial.println("While Loop entered");
    drive_forward(0, 0, initAngle, 400);
    FR_IR(FR_IR_Data);
    FL_IR(FL_IR_Data);
  }
  Serial.println("Distance reached");
  motorstop();
}

//Straighten the drone?
void straighten()
{
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  FR_IR(FR_IR_Data);//right
  FL_IR(FL_IR_Data);//left
  float error, u, lastError, integral, derivative, speed = 0;
  float integralLimit = 30;
  //float error = IR1_dist - IR2_dist;
  float Kp = 1;
  float Ki = 1;
  int timer = 500;

  while (timer > 0) {

    error = FR_IR_Data[0] - FL_IR_Data[0]; //right minus left

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
    FR_IR(FR_IR_Data);
    FL_IR(FL_IR_Data);
  }
  motorstop();
}

void disable_motors()
{
  left_font_motor.detach();     // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();     // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();    // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();    // detach the servo on pin right_front to turn Vex Motor Controller 29 Off
  servo1.detach();
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
void motorstop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

// PI controller helper function
void controller(float error, float kp, float ki, float kd, float integral_limit, float error_limit, float controller_feedback[]) {

  float integral = 0;
  float lastError = 0;
  float derivative = 0;

  integral = integral + error * 0.01;

  //to prevent integral windup
  if (abs(error) > integral_limit) {
    integral = 0;
  }

  // Calculate derivative of error.
  derivative =  error - lastError;
  lastError = error; // Update last error calculated.

  // Loop exits if error remains in steady state for at least 500ms.
  if ((derivative == 0) && (error < error_limit)) {
    controller_feedback[1] -= 100;
  }
  else {
    controller_feedback[1] = 500;
  }

  controller_feedback[0] = kp * error + ki * integral + kd * derivative;

  //modifying feedback array to be accessed in the higher level function

}

void gyro_forward(float target, float initialAngle, float isMiddle) {
  //target positive for forward, negative for backward
  delay(500);
  float angleMoved = 0;
  float GyroAngle = 0;
  float motorval = 0;
  float feedback_one[] = {0, 500}; //controller feedback array, where feedback[0] is u and feedback[1] is timer
  float frontL[] = {0, 999};
  float frontR[] = {0, 999};
  bool backwards = false;
  bool init_angle_big = false;

  float ultra = HC_SR04_range();
  //while timer is greater than 0, the error hasn't been settled for more than 5 ms.

  float prev_millis = millis();
  FL_IR(frontL);
  FR_IR(frontR);

  //Wrapping target so positive is forward, negative is backward
  if (target < 0) {
    target = 200 - 27 + target;
    backwards = true;
  }
  currentAngle = 0;
  initialAngle = 0;

 // BluetoothSerial.println((String)("NEW GYRO FORWARD"));

  //BluetoothSerial.println((String)"gyro_forward() target is " + target + " ultra is " + ultra);
  // Terminate once within desired tolerance.
  while (((ultra > target) && !backwards && (isMiddle || ((frontL[0] > 30) && (frontR[0] > 30)))) || ((ultra < target) && backwards && (isMiddle || ((frontL[0] > 30) && (frontR[0] > 30))))) {
    //wrap initial angle
    //    if (initialAngle > 90) {
    //      initialAngle = initialAngle - 360;
    //      init_angle_big=true;
    //    }

    GyroAngle = gyro_read();

    if (GyroAngle > 90 && !init_angle_big) {
      angleMoved = (GyroAngle - 360) - initialAngle;
    } else {
      angleMoved = GyroAngle - initialAngle;
    }

    //Choose controller settings for either forward or backward
    if (backwards) {
      controller(angleMoved, 82, 1, 0, 1, 1,  feedback_one);
      controller(angleMoved, 70, 3, 0, 1, 1,  feedback_one);
    } else {
      controller(angleMoved, 82, 1, 0, 1, 1,  feedback_one);
      controller(angleMoved, 70, 3, 0, 1, 1,  feedback_one);
    }

    //feedback_one[0]=0;

    //To account for fluctuations in gyroscope. If the change in error is small, make error 0. If the change in error is too big (sudden spike of noise) make error 0.
    if (abs(angleMoved) < 0.6 || abs(angleMoved) > 60) {
      motorval = 0;
    } else {
      motorval = feedback_one[0];
    }

    //constrain(motorval, -100,100);
    if (motorval < -150) {
      motorval = -150;
    } else if (motorval > 150) {
      motorval = 150;
    }

    //  sudden jerk that doesn't work.
    //    //Choose controller settings for either forward or backward
    //    if (backwards) {
    //      //controller(angleMoved, 70, 3, 0, 1, 1,  feedback_one);
    //      controller(angleMoved, 30 , 0.1, 0, 1, 1,  feedback_one);
    //    } else {
    //      //controller(angleMoved, 70, 3, 0, 1, 1,  feedback_one);
    //      controller(angleMoved, 30 , 0.1, 0, 1, 1,  feedback_one);
    //    }
    //
    //    //feedback_one[0]=0;
    //
    //    //To account for fluctuations in gyroscope. If the change in error is small, make error 0. If the change in error is too big (sudden spike of noise) make error 0.
    //    if (abs(angleMoved) < 0.6 || abs(angleMoved)>60) {
    //      if (backwards) {
    //        drive_backward(0, 0, 0, 350);
    //      } else {
    //        drive_forward(0, 0, 0, 350);
    //      }
    //    } else {
    //      motorval = feedback_one[0];
    //      //constrain(motorval, -100,100);
    //      if (motorval<-500){
    //        motorval=-500;
    //      }else if (motorval>500){
    //        motorval=500;
    //      }
    //      ccw_that_fixes_itself(motorval);
    //    }
    //BluetoothSerial.println((String)("initial angle is : ") + initialAngle + (String)"current angle is:" + currentAngle  + (String)("angle reading: ") +  GyroAngle + (String)("error: ") + angleMoved + (String)", adjustment: " + motorval);

    if (backwards) {
      drive_backward(0, 0, motorval, 350);
    } else {
      drive_forward(0, 0, motorval, 350);
    }

    ultra = HC_SR04_range();
    //    FL_IR(frontL);
    //    FR_IR(frontR);

    if (millis() - prev_millis > 450) {
      CoordUpdate();
      prev_millis = millis();
    }

  }

  motorstop();
  delay(500);//VERY IMPORTANTTT!!! FOR GYRO TO GO STRAIGHT need enough time for the motors to settle down.
}

// Determine effort signal to output for PID control based on input error, PID gains, and integral limit.
void PID_Control(float error[], float gains[], float * derivative, float * integral, float * integralLimit, float * u, float effortLimit[]) {

  // Check if error exceeds integral error limit
  if (abs(error[1]) < *integralLimit) {
    *integral = *integral + error[1] * 0.1; // Integrate the error with respect to loop frequency (~10Hz).
  }
  else {
    *integral = 0; // reset integral to avoid windup
  }

  *derivative =  (error[1] - error[0]) / 0.1; // Calculate derivative of error.
  error[0] = error[1]; // Update last error calculated.

  *u = gains[0] * error[1] + gains[1] * *integral + gains[2] * *derivative; // Calculate the control effort to reach target distance.

  if (*u < effortLimit[0]) {
    *u = effortLimit[0];
  }
  else if (*u > effortLimit[1]) {
    *u = effortLimit[1];
  }
}

void testStrafe() {
  // Initialise variables
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};
  float iAngle = 0;
  iAngle = gyro_read();

  FL_IR(FL_IR_Data);
  FR_IR(FR_IR_Data);
  BL_IR(BL_IR_Data);
  BR_IR(BR_IR_Data);
  StrafeDistance(15, true, iAngle);
  delay(100);
  StrafeDistance(32.5, true, iAngle);
  delay(1000);
  //      StrafeDistance(60,true,iAngle);
  //      delay(1000);
  //      StrafeDistance(32.5,false,iAngle);
  //      delay(1000);
  //      StrafeDistance(15,false,iAngle);
}

void Localise() {
  // Initialise variables
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};
  float iAngle = 0;
  iAngle = gyro_read();

  FL_IR(FL_IR_Data);
  FR_IR(FR_IR_Data);
  BL_IR(BL_IR_Data);
  BR_IR(BR_IR_Data);

  //BluetoothSerial.println((String)"frontL: " + FL_IR_Data[0] + (String)"FrontR:" + FR_IR_Data[0]);

  if (FR_IR_Data[0] < 20) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Front right near wall... Aligning");
    //BluetoothSerial.println("=============================================================");
    //AlignDelay(true);
    AlignToWall(true);//false is left, true is right
  } else if (FL_IR_Data[0] < 20) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Front Left Near Wall... Aligining");
    //BluetoothSerial.println("=============================================================");
    //AlignDelay(false);
    AlignToWall(false);
  } else if ((FL_IR_Data[0] < 20) && (FR_IR_Data[0] < 20)) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Facing diagonal corner");
    //BluetoothSerial.println("=============================================================");
    if (FL_IR_Data[0] > FR_IR_Data[0]) {
      //BluetoothSerial.println("=============================================================");
      //BluetoothSerial.println("Strafing right");
      //BluetoothSerial.println("=============================================================");
      StrafeDistance(15, false, iAngle);
      delay(100);
      //BluetoothSerial.println("=============================================================");
      //BluetoothSerial.println("Aligning right");
      //BluetoothSerial.println("=============================================================");
      //AlignDelay(true);
      AlignToWall(true);
    } else if (FR_IR_Data[0] > FL_IR_Data[0]) {
      //BluetoothSerial.println("=============================================================");
      //BluetoothSerial.println("Strafing left");
      //BluetoothSerial.println("=============================================================");
      StrafeDistance(15, true, iAngle);
      delay(100);
      //BluetoothSerial.println("=============================================================");
      //BluetoothSerial.println("Aligning left");
      //BluetoothSerial.println("=============================================================");
      //AlignDelay(false);
      AlignToWall(false);
    }
  } else if ((FL_IR_Data[0] > 79) && (FR_IR_Data[0] > 79)) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("LR sensors out of range (middle of wall)");
    //BluetoothSerial.println("Turning 90 degrees clockwise");
    //BluetoothSerial.println("=============================================================");
    TurnByAngle(80);

  } else if ((FL_IR_Data[0] < 79) && (FR_IR_Data[0] > 79)) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Strafing Left 2");
    //BluetoothSerial.println("=============================================================");
    StrafeDistance(15, true, iAngle);
  } else if ((FR_IR_Data[0] < 79) && (FL_IR_Data[0] > 79)) {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Strafing right 2");
    //BluetoothSerial.println("=============================================================");
    StrafeDistance(15, false, iAngle);
  }
  else {
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println("Random 90 degree CW turn");
    //BluetoothSerial.println("=============================================================");
    //BluetoothSerial.println((String)"Front Left: " + FL_IR_Data[0] + (String)"Front Right: " + FR_IR_Data[0]);
    TurnByAngle(80);
  }
  delay(100);
  //BluetoothSerial.println("Driving straight to corner (hopefully)");
  iAngle = gyro_read();
  //SonarDistance(15, iAngle, false);
  gyro_forward(15, iAngle, 1);
  motorstop();
  delay(100);
  //BluetoothSerial.println("OMFG WE ARE IN A CORNER!!!!");
}

// Rotate platform by a specified angle in degrees using PI control (+ve input = clockwise, -ve input = counter-clockwise).
void TurnByAngle(int turnAngle)
{
  //BluetoothSerial.println("=========================================================================================");
  //BluetoothSerial.println("STARTING TURN BY ANGLE");
  //BluetoothSerial.println("=======================================================================================");
  currentAngle = 0;
  previous_millis = millis();
  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -500, 500};
  float gyroError[] = {0, 0};
  float gyroGains[] = {5, 4, 0.001};
  float gyroIntegral = 0;
  float gyroDerivative = 0;

  float gyroAngle = currentAngle;
  float integralLimit = 30; // Set max error boundary for integral gain to be applied to control system
  int timer = 500; // Initialise tolerance timer.
  float targetAngle = constrain(turnAngle, -180, 180); // Limit maximum turn angle to +ve or -ve 180 degrees.
  //int direction = turnAngle / abs(turnAngle);
  int wrapCheck = 0;

  while (timer > 0) {

    gyroError[1] = targetAngle - gyroAngle; // Calculate error for desired angle.
    if (gyroError[1] > 90) {
      gyroError[1] = gyroError[1] - 360;
    }

    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &integralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.

    // Loop exits if error remains in steady state for at least 500ms.
    if ((gyroDerivative < 5) && (abs(gyroError[1]) < 5)) {
      timer -= 100;
    }
    else {
      timer = 300;
    }

    //Note:
    left_font_motor.writeMicroseconds(1500 + angleEffort);
    left_rear_motor.writeMicroseconds(1500 + angleEffort);
    right_rear_motor.writeMicroseconds(1500 + angleEffort);
    right_font_motor.writeMicroseconds(1500 + angleEffort);

    delay(100); // Loop repeats at a frequency of ~10Hz

    gyroAngle = gyro_read(); // Get current angle reading from gyroscope sensor (range 0 to 359).
    if (currentAngle > 180 && turnAngle > 0) {
      gyroAngle = (gyroAngle - 360);
    }
    else if ((currentAngle > 180) && (turnAngle < 0)) {
      gyroAngle = currentAngle - 360;
    }

    //BluetoothSerial.println((String)"error: " + gyroError[1] + (String)", u: " + angleEffort + (String)" Angle: " + gyroAngle + (String)" wrap: " + wrapCheck);
  }
  motorstop();
  //BluetoothSerial.println("==============================================================================");
  //BluetoothSerial.println("TURN BY ANGLE IS COMPLETE!");
  //BluetoothSerial.println("==============================================================================");
}

// Drive straight and stop a certain distance in cm away from an object detected in front of the robot.

void SonarDistance(float target, float initialAngle, boolean isMiddle) {

  //BluetoothSerial.println((String)"Driving to " + target + (String)"cm");
  // Initialise variables
  float frontL[] = {0, 999};
  float frontR[] = {0, 999};
  float sonar[] = {0, 999};

  float u = 0;
  float uLimit[] = { -250, 250}; //Limit maximuim effort signal for sonar.
  float sonarError[] = {0, 0};
  float sonarGains[] = {10, 0.5, 0.01}; // Kp, Ki, and Kd gains for sonar
  float sonarIntegral = 0;
  float sonarDerivative = 0;

  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -250, 250};
  float gyroError[] = {0, 0};
  float gyroGains[] = {50, 2, 0};
  float gyroIntegral = 0;
  float gyroDerivative = 0;
  float sonarReading = 0;
  //float initialAngle = gyro_read();
  float current_Angle = initialAngle;
  float integralLimit = (uLimit[1] / sonarGains[0]) / 2; // Set max error boundary for integral gain to be applied to control system
  int timer = 500; // Initialise tolerance timer.


  //wrap initial angle
  if (initialAngle > 90) {
    initialAngle = 360 - initialAngle;
  }


  // PI control loop with additional straighten correction using gyro.
  do {
    FL_IR(frontL);
    FR_IR(frontR);
    sonar[0] = HC_SR04_range(); // Determine distance from object using sonar sensors (in cm) and convert to mm.

    sonarError[1] = sonar[0] - target; // Update the error for distance from target distance.

    current_Angle = gyro_read();
    //wrap current angle
    if (current_Angle > 90) {
      current_Angle = current_Angle - 360;
    }

    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).


    PID_Control(sonarError, sonarGains, &sonarDerivative, &sonarIntegral, &integralLimit, &u, uLimit); // Calculate control effort for driving straight using PID control.
    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &integralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.

    //+VE IS CW
    left_font_motor.writeMicroseconds(1500 + (u - angleEffort));
    left_rear_motor.writeMicroseconds(1500 + (u - angleEffort));
    right_rear_motor.writeMicroseconds(1500 - (u + angleEffort));
    right_font_motor.writeMicroseconds(1500 - (u + angleEffort));

    ////BluetoothSerial.println((String)"Error: " + sonarError[1] + (String)(" sonar: ") + sonar[0] + (String)", current angle: " + current_Angle + (String)" initial angle " + initialAngle + (String)" Adjustment: " + angleEffort + (String)" timer: " + timer);
    //BluetoothSerial.println((String)" u is: " + u + (String)" sonar error: " + sonar[0] + (String)" gyro error: " + gyroError[1] + (String)" integral: " + gyroIntegral + (String)", effort: " + angleEffort + (String)" derivative: " + gyroDerivative + (String)" current Angle " + current_Angle + (String)" timer: " + timer);

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(sonarDerivative) < 10) && (abs(sonarError[1]) < 10) && (abs(gyroError[1]) < 1)) {
      timer -= 100;
    }
    else {
      timer = 500;
    }
    delay(100); // ~10Hz

  } while (((sonar != -1) && (timer > 0)) && (isMiddle || ((frontL[0] > 20) && (frontR[0] > 20)))); // Terminate once within desired tolerance.
  motorstop();
  //BluetoothSerial.println((String)"frontL[0]" + frontL[0] + (String)"cm" + (String)"frontR[0]" + frontR[0] + (String)"cm");

  if (sonar == -1) {
    //BluetoothSerial.println("ERROR: INVALID SONAR READING!!!");
  }
  delay(100);
}

void GyroDistance( float initialAngle) {

  //BluetoothSerial.println((String)"Straightening ");

  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -150, 150};
  float gyroError[] = {0, 0};
  float gyroGains[] = {80, 2, 0};
  float gyroIntegral = 0;
  float gyroDerivative = 0;
  float sonarReading = 0;
  //float initialAngle = gyro_read();
  float current_Angle = initialAngle;
  float gyroIntegralLimit = (angleEffortLimit[1] / gyroGains[0]) / 2; // Set max error boundary for integral gain to be applied to control system
  int timer = 500; // Initialise tolerance timer.



  //wrap initial angle
  if (initialAngle > 90) {
    initialAngle = 360 - initialAngle;
  }


  // PI control loop with additional straighten correction using gyro.
  do {


    current_Angle = gyro_read();
    //wrap current angle
    if (current_Angle > 90) {
      current_Angle = current_Angle - 360;
    }

    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).


    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &gyroIntegralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.

    //+VE IS CW
    left_font_motor.writeMicroseconds(1500 + (0 - angleEffort));
    left_rear_motor.writeMicroseconds(1500 + (0 - angleEffort));
    right_rear_motor.writeMicroseconds(1500 + (0 - angleEffort));
    right_font_motor.writeMicroseconds(1500 + (0 - angleEffort));

    //BluetoothSerial.println( (String)" gyro error: " + gyroError[1] + (String)" integral: " + gyroIntegral + (String)", effort: " + angleEffort + (String)" derivative: " + gyroDerivative + (String)" current Angle " + current_Angle + (String)" timer: " + timer);

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(gyroDerivative) < 10) && (abs(gyroError[1]) < 0.5)) {
      timer -= 100;
    }
    else {
      timer = 500;
    }
    delay(100); // ~10Hz

  } while (timer > 0); // Terminate once within desired tolerance.
  motorstop();
  delay(100);
}
// Strafe to a specified distance from a wall using average reading from IR sensors.
void StrafeDistance(float target, boolean isLeft, float initialAngle) {
  float actualTarget = target - 7.5 - 1; //-1 for hard calibration
  //BluetoothSerial.println((String)"Strafing to " + actualTarget + (String)"cm");
  // Initialise variables
  float u = 0;
  float uLimit[] = { -120, 120}; //Limit maximuim effort signal for sonar. -250,250
  float irError[] = {0, 0};
  //float irGains[] = {13.5,5.2,0}; // Kp, Ki, and Kd gains for sonar U=30, T=3
  // float irGains[] = {27, 18, 2.5}; // Kp, Ki, and Kd gains for sonar
  float irGains[] = {26, 21, 5}; // Kp, Ki, and Kd gains for sonar
  float irIntegral = 0;
  float irDerivative = 0.2;
  float irFront[] = {0, 999};
  float irBack[] = {0, 999};

  float angle = 0;
  float angleEffort = 0;
  //float angleEffortLimit[] = {0,0};
  float angleEffortLimit[] = { 0, 0};
  float gyroError[] = {0, 0}; // [lastError,error] for gyro
  float gyroGains[] = {12, 0.1, 0}; // Kp, Ki, and Kd gains for gyro
  float gyroIntegral = 0;
  float gyroDerivative = 0;
  float angleIntegralLimit = angleEffortLimit[1] / gyroGains[0] / 2;
  float ultra = HC_SR04_range();


  //float initialAngle = gyro_read();
  float current_Angle = initialAngle;
  float integralLimit = (uLimit[1] / irGains[0]) / 2; // Set max error boundary for integral gain to be applied to control system
  float gyroIntegralLimit = angleEffortLimit[1] / gyroGains[0]; // Set max error boundary for integral gain to be applied to control system
  int timer = 300; // Initialise tolerance timer.

  //    if (isLeft){
  //      ServoFaceLeft();
  //      delay(500);
  //    }else{
  //      ServoFaceRight();
  //      delay(500);
  //    }

  //Wrap initial angle
  if (initialAngle > 90) {
    initialAngle = 360 - initialAngle;
  }

  if (current_Angle > 90) {
    current_Angle = 360 - current_Angle;
  }


  // PI control loop with additional straighten correction using gyro.
  do {
    // Check which sensors to read based on input parameter.
    if (isLeft) {
      FL_IR(irFront); // Front left IR sensor reading
      BL_IR(irBack); // Back left IR sensor reading
      ultra = HC_SR04_range(); //Ultrasonic reading
      irError[1] = (irBack[0] - (target - 7)); // Error is average difference between IR sensors and target distance.
    } else {
      FR_IR(irFront); // Front right IR sensor reading
      BR_IR(irBack); // Back right IR sensor reading
      ultra = HC_SR04_range(); //Ultrasonic reading
      irError[1] = ( irBack[0] - (target - 7)); // Error is average difference between IR sensors and target distance.
    }
    current_Angle = gyro_read();
    //wrap current angle
    if (current_Angle > 90) {
      current_Angle = current_Angle - 360;
    }

    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).
    PID_Control(irError, irGains, &irDerivative, &irIntegral, &integralLimit, &u, uLimit); // Calculate control effort for driving straight using PID control.
    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &gyroIntegralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(irDerivative) < 10) && abs(irError[1]) < 0.6) {
      timer -= 100;
    }
    else {
      timer = 300;
    }

    //+VE IS CW
    //BluetoothSerial.println( (String)" IR is: " + irBack[0] + (String)" Error: " + irError[1] + (String)", uStrafe: " + u + (String)" derivative: " + irDerivative + (String)" gyroerror: " + gyroError[1] + (String)" timer: " + timer);

    // Check which sensors to read based on input parameter.
    if (!isLeft) {
      left_font_motor.writeMicroseconds(1500 + (+ u - angleEffort));
      left_rear_motor.writeMicroseconds(1500 + ( - u - angleEffort));
      right_rear_motor.writeMicroseconds(1500 + ( - u - angleEffort));
      right_font_motor.writeMicroseconds(1500 + (+ u - angleEffort));
    } else {
      left_font_motor.writeMicroseconds(1500 + (- u - angleEffort));
      left_rear_motor.writeMicroseconds(1500 + ( + u - angleEffort));
      right_rear_motor.writeMicroseconds(1500 + ( + u - angleEffort));
      right_font_motor.writeMicroseconds(1500 + ( - u - angleEffort));
    }

    delay(100); // ~10Hz
  } while (timer > 0); // Terminate once within desired tolerance.
  //BluetoothSerial.println("Strafing Complete");
  motorstop();
}

// Strafe to a specified distance from a wall using average reading from IR sensors.
void StrafeSonar(float target, boolean isLeft) {
  // Initialise variables
  float u = 0;
  float ultra = 0;
  float uLimit[] = { -120, 120}; //Limit maximuim effort signal for sonar. -250,250
  float Error[] = {0, 0};
  //float irGains[] = {13.5,5.2,0}; // Kp, Ki, and Kd gains for sonar U=30, T=3
  //float Gains[] = {17,4,0.005}; // Kp, Ki, and Kd gains for sonar
  float Gains[] = {0.1, 0, 0}; // Kp, Ki, and Kd gains for sonar
  float Integral = 0;
  float integralLimit = 2;
  float Derivative = 0.2;
  float Front[] = {0, 999};
  float Back[] = {0, 999};
  float Ultra_Data[] = {0, 999};
  float prev_mil = millis();

  float ultraSidePrint = 0;
  ULTRA_DIST(Ultra_Data);

  int timer = 300; // Initialise tolerance timer.

  // If we started on the left
  if (global_isLeft) {
    ServoFaceLeft();
    delay(500);
  } else {
    ServoFaceRight();
    delay(500);
  }

  // PI control loop with additional straighten correction using gyro.
  do {
    // Check which sensors to read based on input parameter.
    ULTRA_DIST(Ultra_Data);
    ultra = HC_SR04_range();

    if (!global_isLeft) {
      //ultraSidePrint = Ultra_Data[0] - 4.74;//-3cm is the wheel distance.
    } else {
      //ultraSidePrint = Ultra_Data[0] - 5.64;
    }

    ultraSidePrint = Ultra_Data[0];

    Error[1] = ultraSidePrint - target; // Error is average difference between IR sensors and target distance.

    PID_Control(Error, Gains, &Derivative, &Integral, &integralLimit, &u, uLimit); // Calculate control effort for driving straight using PID control.

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(Derivative) < 10) && abs(Error[1]) < 2) {
      timer -= 100;
    }
    else {
      timer = 300;
    }

    //+VE IS CW
    //BluetoothSerial.println((String)"Raw sonar value is:"+ ultra+(String)" Error: " + Error[1] + (String)", uStrafe: " + u + (String)" derivative: " + Derivative + (String)" timer: " + timer);


    if (!global_isLeft) {
      if (u > 0) {
        strafe_right(u, 100);
      } else {
        strafe_left(u, 100);
      }
    } else {
      if (u > 0) {
        strafe_left(u, 100);
      } else {
        strafe_right(u, 100);
      }
    }

    if (millis() - prev_mil > 400) {
      CoordUpdate();
      prev_mil = millis();
    }

    delay(100); // ~10Hz
  } while (timer > 0); // Terminate once within desired tolerance.
  //BluetoothSerial.println("Strafing Complete");
  motorstop();
  delay(100);

  ServoFaceForward();
  delay(500);
}

void OpenDriveBackward() {
//  left_font_motor.writeMicroseconds(1500 - 419);
//  left_rear_motor.writeMicroseconds(1500 - 419);
//  right_rear_motor.writeMicroseconds(1500 + 400);
//  right_font_motor.writeMicroseconds(1500 + 400);
  left_font_motor.writeMicroseconds(1500 - 500);
  left_rear_motor.writeMicroseconds(1500 - 500);
  right_rear_motor.writeMicroseconds(1500 + 450);
  right_font_motor.writeMicroseconds(1500 + 450);
}

void OpenDriveForward() {
  left_font_motor.writeMicroseconds(1500 + 500);
  left_rear_motor.writeMicroseconds(1500 + 500);
  right_rear_motor.writeMicroseconds(1500 - 450);
  right_font_motor.writeMicroseconds(1500 - 450);
}


// Strafe to a specified distance from a wall using average reading from IR sensors.
void StrafeTime(float timeToStrafe, boolean isLeft, float initialAngle, float strafeSpeed) {
  delay(500);
  //BluetoothSerial.println((String)"Strafing for " + timeToStrafe + (String)" seconds");
  // Initialise variables
  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -150, 150};
  float gyroError[] = {0, 0}; // [lastError,error] for gyro
  float gyroGains[] = {12, 0.1, 0}; // Kp, Ki, and Kd gains for gyro
  float gyroIntegral = 0;
  float gyroDerivative = 0;
  //float initialAngle = gyro_read();
  float current_Angle = initialAngle;
  //float integralLimit = (uLimit[1]/irGains[0])/2; // Set max error boundary for integral gain to be applied to control system
  float gyroIntegralLimit = angleEffortLimit[1] / gyroGains[0]; // Set max error boundary for integral gain to be applied to control system
  int timer = 300; // Initialise tolerance timer.
  float prev_millis = millis(); //for coords

  //Wrap initial angle
  if (initialAngle > 90) {
    initialAngle = 360 - initialAngle;
  }
  if (current_Angle > 90) {
    current_Angle = 360 - current_Angle;
  }

  // PI control loop with additional straighten correction using gyro.
  do {
    //wrap current angle
    if (current_Angle > 90) {
      current_Angle = current_Angle - 360;
    }

    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).
    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &gyroIntegralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.
    timeToStrafe = timeToStrafe - 100;

    if (abs(gyroError[1]) < 1) {
      timer = timer - 100;
    } else {
      timer = 300;
    }

    angleEffort = constrain(0, -1 * (500 - strafeSpeed), (500 - strafeSpeed));

    //+VE IS CW
    //BluetoothSerial.println( (String)" STRAFING initial angle is:" + initialAngle + "Gyro is: " + current_Angle + (String)" Error: " + gyroError[1] + (String)", angle effort: " + angleEffort + (String)" timer: " + timer);

    // Check which sensors to read based on input parameter.
    if (!isLeft) {
      left_font_motor.writeMicroseconds(1500 + (strafeSpeed - angleEffort));
      left_rear_motor.writeMicroseconds(1500 - (strafeSpeed + angleEffort));
      right_rear_motor.writeMicroseconds(1500 - (strafeSpeed + angleEffort));
      right_font_motor.writeMicroseconds(1500 + (strafeSpeed - angleEffort));
    } else {
      left_font_motor.writeMicroseconds(1500 - (strafeSpeed + angleEffort));
      left_rear_motor.writeMicroseconds(1500 + (strafeSpeed - angleEffort));
      right_rear_motor.writeMicroseconds(1500 + ( strafeSpeed - angleEffort));
      right_font_motor.writeMicroseconds(1500 - ( strafeSpeed + angleEffort));
    }
    CoordUpdate();
    delay(100); // ~10Hz
  } while (timeToStrafe > 0); // Terminate once within desired tolerance.

  //  // This corrects angle if the motor stops running
  //  while (timer > 0) {
  //    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).
  //
  //    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &gyroIntegralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.
  //    if (abs(gyroError[1]) < 1) {
  //      timer = timer - 100;
  //    } else {
  //      timer = 300;
  //    }
  //
  //    //+VE IS CW
  //    ////BluetoothSerial.println( (String)" STRAFING initial angle is:" + initialAngle+ "Gyro is: " + current_Angle+(String)" Error: " + gyroError[1] + (String)", angle effort: " + angleEffort + (String)" timer: " + timer);
  //
  //    // Check which sensors to read based on input parameter.
  //    if (!isLeft) {
  //      left_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
  //      left_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
  //      right_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
  //      right_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
  //    } else {
  //      left_font_motor.writeMicroseconds(1500 - (150 + angleEffort));
  //      left_rear_motor.writeMicroseconds(1500 + (150 - angleEffort));
  //      right_rear_motor.writeMicroseconds(1500 + ( 150 - angleEffort));
  //      right_font_motor.writeMicroseconds(1500 - ( 150 + angleEffort));
  //    }
  //
  //    if (millis() - prev_millis > 400) {
  //      CoordUpdate();
  //      prev_millis = millis();
  //    }

  //    delay(100); // ~10Hz
  //  } while (timeToStrafe > 0); // Terminate once within desired tolerance.

  //  timeToStrafe = timeToStrafe - 100;
  //
  //  if (abs(gyroError[1]) < 1) {
  //    timer = timer - 100;
  //  } else {
  //    timer = 300;
  //  }
  //  if (!isLeft) {
  //    left_font_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
  //    left_rear_motor.writeMicroseconds(1500 - (0 - angleEffort * 10));
  //    right_rear_motor.writeMicroseconds(1500 - (0 - angleEffort * 10));
  //    right_font_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
  //  } else {
  //    left_font_motor.writeMicroseconds(1500 - ( 0 - angleEffort * 10));
  //    left_rear_motor.writeMicroseconds(1500 + ( 0 + angleEffort * 10));
  //    right_rear_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
  //    right_font_motor.writeMicroseconds(1500 - ( 0 - angleEffort * 10));
  //  }
  ////BluetoothSerial.println("Strafing Complete");
  motorstop();
  delay(500);//VERY IMPORTANTTT!!! FOR GYRO TO GO STRAIGHT need enough time for the motors to settle down.
}

void ServoFaceForward() {
  servo_forward = 1;
  servo1.write(90);
}

void ServoFaceLeft() {
  servo_forward = 0;
  servo1.write(180);
}

void ServoFaceRight() {
  servo_forward = 0;
  servo1.write(0);
}
//#pragma endregion end

//=============================================================
//#pragma region 4.1 IR SENSOR FUNCTIONS
//=============================================================
void ULTRA_DIST(float ultra_output[])
{
  float distance = HC_SR04_range();
  Kalman(distance, ultra_output, 5);
}

void FR_IR(float FR_IRoutput[])
{
  int signalADC = analogRead(frontR_IR);
  float distance = 15832 * pow(signalADC, -1.21);
  distance = constrain(distance, 1, 80);
  Kalman(distance, FR_IRoutput, 5);
}

void FL_IR(float FL_IRoutput[])
{
  int signalADC = analogRead(frontL_IR);
  float distance = 16479 * pow(signalADC, -1.21);
  distance = constrain(distance, 1, 80);
  Kalman(distance, FL_IRoutput, 5);

}

void BL_IR(float BL_IR_output[])
{
  int signalADC = analogRead(backL_IR);
  float distance = 7275 * pow(signalADC, -1.19);
  distance = constrain(distance, 1, 30);
  Kalman(distance, BL_IR_output, 5);
}

void BR_IR(float BR_IR_output[])
{
  int signalADC = analogRead(backR_IR);
  float distance = 4921 * pow(signalADC, -1.13);
  distance = constrain(distance, 1, 30);
  Kalman(distance, BR_IR_output, 5);
}

void Kalman(float rawdata, float kalman_output[], float sensor_noise) {  // Kalman Filter
  float a_priori_est = 0;
  float a_post_est = 0;
  float a_priori_var = 0;
  float a_post_var = 0;
  float kalman_gain = 0;
  a_priori_est = kalman_output[0];
  a_priori_var = kalman_output[1] + process_noise;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_est = constrain(a_post_est, 0, 999); // constrain output to prevent infinite values.
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var = a_post_var;
  kalman_output[0] = a_post_est;
  kalman_output[1] = a_post_var;
}
//#pragma endregion end

//=============================================================
// #pragma region 4.2 GYRO SENSOR FUNCTIONS
//=============================================================
float gyro_read()
{
  if (previous_millis == 0) {
    previous_millis = millis();
  }

  T = millis() - previous_millis;
  previous_millis = millis();

  //If functions changed, reset T.
  if (T > 300) {
    T = 1;
  }

  // convert the 0-1023 signal to 0-5v
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate = (analogRead(sensorPin) - gyroZeroVoltage) * gyroSupplyVoltage / 1023;

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T). 1000 converts milliseconds to seconds
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

  //   Serial.print("previous millis is: ");
  //  Serial.println(previous_millis);

  //  Serial.print("Time now: ");
  //  Serial.println(millis());
  //BluetoothSerial.print((String)"Time taken for one loop is: "+T);
  //Serial.print((String)"Current angle: " + analogRead(sensorPin));

  // control the time per loop
  delay (100);
  return currentAngle;
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}
//#pragma endregion end

//=============================================================
// #pragma region 4.3 ULTRASONIC SENSOR FUNCTIONS
//=============================================================

// Sonar Sensor
#ifndef NO_HC-SR04
float HC_SR04_range()
{
  unsigned long t1=0;
  unsigned long t2=0;
  unsigned long pulse_width=0;
  float cm=0;
  float inches=0;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    ////BluetoothSerial.println((String)"Pulse width is: " + pulse_width);
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
    ////BluetoothSerial.println((String)"Pulse width is: " + pulse_width);
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
//#pragma endregion end

//=============================================================
// #pragma region 5. LIPO BATTERY
//=============================================================

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

// Slow flash LED to indicate stopped state.
void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
//#pragma endregion end


//=============================================================
//#pragma region 6. WIRELESS MODULE FUNCTIONS
//=============================================================
void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void BluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";

  //BluetoothSerial.print(Value1, DEC);
  //BluetoothSerial.print(Delimiter);
  //BluetoothSerial.print(Value2, DEC);
  //BluetoothSerial.print(Delimiter);
  //BluetoothSerial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3)
{
  if (OUTPUTMONITOR)
  {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER)
  {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR)
  {
    //BluetoothSerialOutputMonitor(Value1, Value2, Value3);;
  }
}
//#pragma endregion end
