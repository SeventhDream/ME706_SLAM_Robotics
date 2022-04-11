//=============================================================
//#pragma region 1. SETUP
//=============================================================
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h> //Need for Wireless module


//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

const byte interruptPin = 5;

// Wireless Module Setup
#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin
#define OUTPUTMONITOR 0 // USB Serial Port
#define OUTPUTPLOTTER 0 // USB Serial Plotter
#define OUTPUTBLUETOOTHMONITOR 1 // Bluetooth Serial Port

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
int speed_val = 150;
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
float gyroZeroVoltage = 505;          // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
float gyroRate = 0;                 // read out value of sensor in voltage
float currentAngle = 0;            // current angle calculated by angular velocity integral on


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

void setup(void)
{
  //Coordinate Interrupt
  pinMode(interruptPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR1, CHANGE);

  //turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  //Gyroscope Setup Start
  pinMode(sensorPin, INPUT);
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
  static unsigned long previous_millis;
  fast_flash_double_LED_builtin();

  /*--------------------------------COURSE START--------------------------------*/
  //  String Delimiter = ", ";
  //
  //  BluetoothSerial.println((String)"FrontL: " + frontL[0] + " FrontR: " + frontR[0] + " BackL: " + backL[0]+ " backR: " + backR[0]);


  BluetoothSerial.println("=============================================================");
  BluetoothSerial.println("Started the course.");
  BluetoothSerial.println("=============================================================");
  while (1){
    gyro_forward(15,0);
  }
//  while(1){
//    gyro_read();
//  }
 //FindCorner();
 //WallFollow();
 //altMiddleLogic();
 //WallFollow();
 //AlignToWall(true);
 // delay(5000);
 //AlignToWall(true);
  BluetoothSerial.println("STOPPED");
  //Serial.println("Finished the course.");
  //TurnByAngle(80);

  stop();
  delay(5000);
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
  BluetoothSerial.print((String)"(x, y) = (" + x +  ", " + y + ")"); //Printing x and y
}

void ISR1() {
  Coord();
  BluetoothSerial.println("Coordinate Interrupt!");
  BluetoothSerial.println("Coordinate Interrupt!");
  BluetoothSerial.print("(x, y) = ");
  BluetoothSerial.print(x);
  BluetoothSerial.print(", ");
  BluetoothSerial.print(y);
}

void CoordUpdate() {
  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  FR_IR(FR_IR_Data); // Front right IR sensor reading
  FL_IR(FL_IR_Data); // Front left IR sensor reading
  //!isLeft means the wallstartedon the left
  x = 200 - (HC_SR04_range() + (24 / 2)); // 12 / 2 is distance between sonar and middle of robot (**TUNING NEEDED**)
  BluetoothSerial.println((String)"IR right is:"+FR_IR_Data[0]+(String)"IR left is:"+FL_IR_Data[0]);
  if (!global_isLeft) {
    if (FR_IR_Data[0] > 79) {
      y = 120 - (7 + FL_IR_Data[0]);
    } else {
      y = 7 + FR_IR_Data[0];
    }
  } 
  else {
    if (FL_IR_Data[0] > 79) {
      y = 120 - (7 + FR_IR_Data[0]);
    } else {
      y = 7 + FL_IR_Data[0];
    }
    
  }
  BluetoothSerial.println("COORDINATES UPDATING!");
  BluetoothSerial.print("(x, y) = ");
    BluetoothSerial.print(x);
    BluetoothSerial.print(", ");
    BluetoothSerial.println(y);
  //print to textfile/serial output here
  }
//#pragma endregion end

//=============================================================
//#pragma region 3.2 MOTOR MOVEMENT FUNCTIONS
//=============================================================
// alternative middle logic with hardcoded behaviour logic
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

  if (((FR_IR_Data[0] + BR_IR_Data[0]) / 2) > ((FL_IR_Data[0] + BL_IR_Data[0]) / 2)) {
    StrafeTime(1000, false, iAngle);

    gyro_forward(-15, iAngle);
    StrafeTime(1000, false, iAngle);
      gyro_forward(15,iAngle);
      StrafeTime(1000,false,iAngle);
      //delay(3000);
      
      gyro_forward(-15,iAngle);
      StrafeTime(1000,false,iAngle);

      gyro_forward(15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,false,iAngle);
       
      gyro_forward(-15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,false,iAngle);
            //SonarDistance(160,iAngle,true);
      
    }
    else{
      StrafeTime(1000,true,iAngle);
      gyro_forward(-15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,true,iAngle);
      gyro_forward(15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,true,iAngle);
      gyro_forward(-15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,true,iAngle);
      gyro_forward(15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,true,iAngle);
      gyro_forward(-15,iAngle);
      //SonarDistance(160,iAngle,true);
      StrafeTime(1000,true,iAngle);
      //SonarDistance(160,iAngle,true);
       
    }
  }
  void MiddleLogic() {
    
    //Reading sensor values
    float FR_IR_Data[] = {0,999};
    FR_IR(FR_IR_Data);
    float FL_IR_Data[] = {0,999};
    FL_IR(FL_IR_Data);
    float BR_IR_Data[] = {0,999};
    BR_IR(BR_IR_Data);
    float BL_IR_Data[] = {0,999};
    BL_IR(BL_IR_Data);
    
    unsigned long prev_millis = millis();
    float half_second_count = 0;
    float strafe_time = 1; ///**NEED TO TUNE
    bool Forward = 0;
    bool isLeft = 0;
    float iAngle = gyro_read();
    float init_angle=0;
    x = 85; //TEMP
    y = 15; //TEMP

    y_distance=15+8;

    if (((FR_IR_Data[0] + BR_IR_Data[0]) / 2) > ((FL_IR_Data[0] + BL_IR_Data[0]) / 2)) {
        //wall is on the left of the robot
        isLeft = 0;
        BluetoothSerial.println("Wall is on the left!");
        MiddleStrafe(isLeft,iAngle);
        
        
        /*
        //OLD CODE (for reference)
        while (half_second_count < strafe_time * 2) {//need to be tuned
          strafe_right();
          prev_millis = millis();
          y = y + (half_second_count * (22.5 / (strafe_time * 2)));
          if (millis() - prev_millis > 500){
            prev_millis = millis();
            half_second_count++;
          }
          }
          half_second_count = 0;
          stop();
          */

        SonarDistance(200 - 15 - 24,iAngle, true); //12 / 2 should be dist from mid of robot to sonar **NEED TO TUNE**
        stop();
        
        /*
        //OLD CODE (for reference)
        ultra = HC_SR04_range();
        while (ultra < (200 - 15 - 13)) { //calibrate later
        if (millis() - prev_millis > 500){
          prev_millis = millis();
          }
      }
      stop();*/
        
        
        MiddleStrafe(isLeft,iAngle);

    gyro_forward(15, iAngle);
    StrafeTime(1000, false, iAngle);
    delay(3000);

    gyro_forward(-15, iAngle);
    StrafeTime(1000, false, iAngle);

    gyro_forward(15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, false, iAngle);

    gyro_forward(-15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, false, iAngle);
    //SonarDistance(160,iAngle,true);

  }
  else {
    StrafeTime(1000, true, iAngle);
    gyro_forward(-15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, true, iAngle);
    gyro_forward(15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, true, iAngle);
    gyro_forward(-15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, true, iAngle);
    gyro_forward(15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, true, iAngle);
    gyro_forward(-15, iAngle);
    //SonarDistance(160,iAngle,true);
    StrafeTime(1000, true, iAngle);
    //SonarDistance(160,iAngle,true);

  }
}

void MiddleStrafe(bool Left, float iAngle) {
    int half_second_count = 0;
    float prev_millis = millis();
    int strafe_time = 1; //[seconds]
    //while (half_second_count < strafe_time * 2) { //need to be tuned
        if (Left) {
              BluetoothSerial.println("Strafing Using Right Sesnors");
              if (y_distance < 60){
               StrafeDistance(y_distance,false,iAngle);
              } else{
          StrafeDistance(y_distance,true,iAngle);
        }
        }
        else {
          BluetoothSerial.println("Strafing Using Right Sesnors");
         if (y_distance < 60){
               StrafeDistance(y_distance,true,iAngle);
              }
              else{
          StrafeDistance(y_distance,false,iAngle);
        }
        }
//        y = y + (half_second_count * (22.5 / (strafe_time * 2)));
//        if (millis() - prev_millis > 500) {
//          prev_millis = millis();
//          half_second_count++;
//         }
        //}
        y_distance=y_distance+22.5;
        stop();
  }
  
void MiddleStrafe1(int isLeft, float iAngle) {
  StrafeDistance(22.5, isLeft, iAngle);

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
    BluetoothSerial.println("Orientate the robot to face a wall 60cm away");
    cw();
    ultraDist = HC_SR04_range();
    delay(50);
  }
  stop();

  delay(1000);

  BluetoothSerial.println("Drive straight until 20cm from front-facing wall");
  gyro_forward(20,0);
  //SonarDistance(20, iAngle, false); // Drive straight until 15cm from front-facing wall

  stop();

  Localise();
  //Rereading sensor values
  BR_IR(BR_IR_Data);

    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println((String)"back right is"+BR_IR_Data[0]);
     BluetoothSerial.println("=============================================================");  // Check which sides of the robot are facing the wall
  if (BR_IR_Data[0] < 20) {
    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Wall is on the right, turning CCW");
     BluetoothSerial.println("=============================================================");
    TurnByAngle(-80);
    delay(100);
    ultraDist = HC_SR04_range();
    BluetoothSerial.println((String)"Ultrasonic reading is:"+ultraDist);
    if (ultraDist > 120) {
      AlignToWall(true);
      }
    else {
      TurnByAngle(-80);
      AlignToWall(false);
    }
  } else {
    BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Wall is on the left, turning CW");
     BluetoothSerial.println("=============================================================");
    TurnByAngle(80);
    delay(100);

    ultraDist = HC_SR04_range();
    BluetoothSerial.println((String)"Ultrasonic reading is:"+ultraDist);
    if (ultraDist > 120) {
      AlignToWall(false);
    }
    else {
      TurnByAngle(80);
      AlignToWall(true);
    }
  }
  BluetoothSerial.println("Ready to START MAPPING!");
  stop();
}
void WallFollow() {
  float ultra = HC_SR04_range();
  float initialAngle = 0;
  float angleMoved, GyroAngle = 0;
  
  float error_long, error_short, long_IR, short_IR, left, integral_long, integral_short, travel_angle, speed_long, speed_short, u_long, u_short = 0;
  float target = 7;
  float strafe_thresh = 10; //if teh robot is more than 10cm away from the target distance, robot will strafe.

  float FR_IR_Data[] = {0, 999};
  float FL_IR_Data[] = {0, 999};
  float BL_IR_Data[] = {0, 999};
  float BR_IR_Data[] = {0, 999};

  //controller arrays
  float long_feedback[] = {0, 500};
  float short_feedback[] = {0, 500};
  float gyro_feedback[] = {0, 500};

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
    BluetoothSerial.println((String)"Current Long IR is: " + long_IR + (String)", Error is: " + error_long + (String)"Current Short IR is: " + short_IR + (String)", Error is: " + error_short);

  //    //If wall is too far, strafe left or right.
  //    if ((((error_long + error_short) / 2) < (-1 * strafe_thresh) && left == 1) || ((((error_long + error_short) / 2) > strafe_thresh) && left == 0)) {
  //      StrafeDistance(abs(error_long + error_short) / 2, true, initialAngle);
  //    } else if ((((error_long + error_short) / 2) < (-1 * strafe_thresh) && left == 0) || ((((error_long + error_short) / 2) > strafe_thresh) && left == 1)) {
  //      StrafeDistance(abs(error_long + error_short) / 2, false, initialAngle);
  //    }

    controller(error_long, 5.5, 0.9, 0.05, 1, 0.5, long_feedback);
    controller(error_short, 5.5, 0.9, 0.05, 1, 0.5, short_feedback);
    controller(angleMoved, 5, 50, 0, 5, 1, gyro_feedback);

    speed_long = constrain(long_feedback[0], -500, 500);
    speed_short = constrain(short_feedback[0], -500, 500);

    //BluetoothSerial.println((String)" long IR timer is " + long_feedback[0] + (String)" Short IR timer is" + short_feedback[1]);
    //BluetoothSerial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_long + (String)" Left Side = " + speed_short);
    //If errors are small enough fluctuating between positive and negative, make the right and left motors same power
    if( abs(error_long)<0.4 && abs(error_short)<0.4){
      drive_forward(0,0,0);
    }else if (left==1 && short_IR>long_IR){//Top left
      BluetoothSerial.println("top left");
      drive_forward(abs(long_feedback[0]),short_feedback[0],0);
    }else if (left==1 && short_IR<long_IR){//bottom left
      BluetoothSerial.println("bottom left");
      drive_forward(long_feedback[0],abs(short_feedback[0]),0);
    }else if(left==-1 && short_IR>long_IR){//top right
      BluetoothSerial.println("top right");
      drive_forward(short_feedback[0],abs(long_feedback[0]),0);
    }else{//bottom right
       BluetoothSerial.println("bottom right");
      drive_forward(abs(short_feedback[0]),long_feedback[0],0);
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

void AlignToWall(boolean isLeft) {
  BluetoothSerial.println("=============================================================");
  BluetoothSerial.println("ALIGN TO WALL START");
  BluetoothSerial.println("=============================================================");
  
  //FALSE MEANS WALL IS ON THE LEFT
  float error, u, lastError, integral, derivative=0;
  float integralLimit = 10; // Set max error boundary for integral gain to be applied to control system.
  float initialAngle = gyro_read();
  float effort = 0;
  float Kp = 15; // Initialise proportional gain.
  float Ki = 5; // Initialise integral gain
  int timer = 500; // Initialise tolerance timer.
  float F_IR_Data[] = {0, 999};
  float B_IR_Data[] = {0, 999};
  float backL = 0;
  int directionvar = 0;

  while (timer > 0) {
    if (isLeft) { //Wall is on the right
      FR_IR(F_IR_Data); // Front IR sensor reading
      BR_IR(B_IR_Data); // Back IR sensor reading
      directionvar = 1;
    }
    else { //Wall is on the left
      FL_IR(F_IR_Data); // Front IR sensor reading
      BL_IR(B_IR_Data); // Back IR sensor reading
      directionvar = -1;
    }

    error = F_IR_Data[0] - B_IR_Data[0]; // Error is difference between readings
    BluetoothSerial.println((String) "Front IR: " + F_IR_Data[0]+ (String)" Back IR: " + B_IR_Data[0] );
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
    if ((derivative < 1) && (abs(error) < 0.5)) {
      timer -= 100;
      BluetoothSerial.println((String) "TIMER TESTING FRONT IR: " + F_IR_Data[0]+ (String)" Back IR: " + B_IR_Data[0]+(String) "error: " + error +(String)" Integral is: " + integral+(String)" d: " + derivative);
    }
    else {
      timer = 300;
    }

    if (abs(error)>8){
      BluetoothSerial.println("Error is greater than 8");
      Kp=8.2;
      Ki=1.2;
    }else{
      Kp = 13; // Initialise proportional gain.
      Ki = 3.8; // Initialise integral gain      
    }
    u = Kp * error + Ki * integral; // Calculate the control effort to reach target distance.
    effort = constrain(u, -500, 500);
    BluetoothSerial.println((String) "error: " + error + (String)" u: " + effort + (String)" d: " + derivative);

      left_font_motor.writeMicroseconds(1500 + directionvar *effort);
      left_rear_motor.writeMicroseconds(1500 + directionvar *effort);
      right_rear_motor.writeMicroseconds(1500 + directionvar *effort);
      right_font_motor.writeMicroseconds(1500 + directionvar *effort);
      delay(100); // Loop repeats at a frequency of ~10Hz
  }



  
  stop();
  BluetoothSerial.println("=============================================================");
  BluetoothSerial.println("ALIGN TO WALL END");
  BluetoothSerial.println("=============================================================");
}

// Strafe left at fixed speed value.
void strafe_left (float u)
{
  u = constrain(u, -1 * speed_val, speed_val);
  left_font_motor.writeMicroseconds(1500 - speed_val - u);
  left_rear_motor.writeMicroseconds(1500 + speed_val + u);
  right_rear_motor.writeMicroseconds(1500 + speed_val + u);
  right_font_motor.writeMicroseconds(1500 - speed_val - u);
}

// Strafe right at fixed speed value.
void strafe_right (float u)
{
  u = constrain(u, -1 * speed_val, speed_val);
  left_font_motor.writeMicroseconds(1500 + speed_val + u);
  left_rear_motor.writeMicroseconds(1500 - speed_val - u);
  right_rear_motor.writeMicroseconds(1500 - speed_val - u);
  right_font_motor.writeMicroseconds(1500 + speed_val + u);
}

// Pivot counter clockwise at a fixed speed value
void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void drive_test ()
{
  left_font_motor.writeMicroseconds(1500 + 200);
  left_rear_motor.writeMicroseconds(1500 + 200);
  right_rear_motor.writeMicroseconds(1500 - 100);
  right_font_motor.writeMicroseconds(1500 - 100);
}

void drive_forward(float adjustment1, float adjustment2, float correction) {

  adjustment1 = constrain(adjustment1, -150, 150);
  adjustment2 = constrain(adjustment2, -150, 150);
  correction = constrain(correction, -150, 150);
  BluetoothSerial.println((String)"adjustment 1 is: " + adjustment1+(String)"adjustment 2 is: " + adjustment2 +(String)"Correction is:"+correction);

  left_font_motor.writeMicroseconds(1500 + (speed_val + adjustment1 - correction));
  left_rear_motor.writeMicroseconds(1500 + (speed_val + adjustment1 - correction));
  right_rear_motor.writeMicroseconds(1500 - (speed_val + adjustment2+ correction));
  right_font_motor.writeMicroseconds(1500 - (speed_val + adjustment2+ correction));
}

void drive_backward(float adjustment, float correction) {

  adjustment = constrain(adjustment, -150, 150);
  correction = constrain(correction, -150, 150);
  BluetoothSerial.println((String)"adjustment is: " + correction);

  left_font_motor.writeMicroseconds(1500 - (speed_val + adjustment + correction));
  left_rear_motor.writeMicroseconds(1500 - (speed_val + adjustment + correction));
  right_rear_motor.writeMicroseconds(1500 + (speed_val + adjustment - correction));
  right_font_motor.writeMicroseconds(1500 + (speed_val + adjustment - correction));
}


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
    drive_forward(0, 0, initAngle);
    FR_IR(FR_IR_Data);
    FL_IR(FL_IR_Data);
  }
  Serial.println("Distance reached");
  stop();
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
  stop();
}

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

// PI controller helper function
void controller(float error, float kp, float ki, float kd, float integral_limit, float error_limit, float feedback[]) {

  float integral, lastError, derivative = 0;

  integral = integral + error * 0.01;

  //to prevent integral windup
  if (error > integral_limit) {
    integral = 0;
  }

  // Calculate derivative of error.
  derivative =  error - lastError;
  lastError = error; // Update last error calculated.

  // Loop exits if error remains in steady state for at least 500ms.
  if ((derivative == 0) && (error < error_limit) && (error != 0)) {
    feedback[1] -= 100;
  }
  else {
    feedback[1] = 500;
  }

  feedback[0] = kp * error + ki * integral + kd * derivative;

  //modifying feedback array to be accessed in the higher level function

}

void gyro_forward(float target, float initialAngle){
    //target positive for forward, negative for backward
      float angleMoved,GyroAngle=0;
      float feedback[]={0,500};//controller feedback array, where feedback[0] is u and feedback[1] is timer
      bool backwards=false;

      float ultra = HC_SR04_range();
      //while timer is greater than 0, the error hasn't been settled for more than 5 ms.

      float prev_millis = millis();

      //Wrapping target so positive is forward, negative is backward
      if (target<0){
        target=200-24+target;
        backwards=true;
      }
      
      BluetoothSerial.println((String)"gyro_forward() target is "+target+" ultra is "+ultra);
      while(((ultra>target) && !backwards) || ((ultra<target) && backwards)){
        //wrap initial angle
        if (initialAngle > 90) {
          initialAngle = initialAngle-360;
        }
        GyroAngle=gyro_read();
        if (GyroAngle> 90) {
          angleMoved = (GyroAngle-360)-initialAngle;
        }else{
          angleMoved= GyroAngle-initialAngle; 
        }
        BluetoothSerial.println((String)("initial angle is : ") + initialAngle+(String)("angle reading: ") +  GyroAngle+(String)("error: ") + angleMoved + (String)", adjustment: " + feedback[0]);

        //controller(angleMoved, 10, 50, 0, 5,1,feedback);        
        controller(angleMoved,24,2,0.005,10,1,feedback);

//        //To account for fluctuations in gyroscope. If the change in error is small, make error 0.
//        if (abs(angleMoved)<1){
//          feedback[0]=0;
//        }

        if (backwards){
          drive_backward(0,feedback[0]);
        }else{
          drive_forward(0,0,feedback[0]);
        }


        ultra = HC_SR04_range();

  //        int wait = 1000;
  //        int wait_millis = millis();
  //        while (millis() - wait_millis < 1000) {
  //          if (millis() - prev_millis > 450) {
  //            CoordUpdate();
  //            prev_millis = millis();
  //          }
  //        }

        if (millis() - prev_millis > 450) {
          CoordUpdate();
          prev_millis = millis();
        }
        
      }
      
      stop();
      delay(1000);//VERY IMPORTANTTT!!! FOR GYRO TO GO STRAIGHT need enough time for the motors to settle down.
  }
// Determine effort signal to output for PID control based on input error, PID gains, and integral limit.
void PID_Control(float error[], float gains[], float *derivative, float *integral, float *integralLimit, float *u, float effortLimit[]) {

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

  BluetoothSerial.println((String)"frontL: " + FL_IR_Data[0] + (String)"FrontR:" + FR_IR_Data[0]);

  if (FR_IR_Data[0] < 20) {
  BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Front right near wall... Aligning");
     BluetoothSerial.println("=============================================================");
    AlignToWall(true);//false is left, true is right
  } else if (FL_IR_Data[0] < 20) {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Front Left Near Wall... Aligining");
     BluetoothSerial.println("=============================================================");
    AlignToWall(false);
  } else if ((FL_IR_Data[0] < 60) && (FR_IR_Data[0] < 60)) {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Facing diagonal corner");
     BluetoothSerial.println("=============================================================");
    if (FL_IR_Data[0] > FR_IR_Data[0]) {
       BluetoothSerial.println("=============================================================");
      BluetoothSerial.println("Strafing right");
       BluetoothSerial.println("=============================================================");
      StrafeDistance(15, false, iAngle);
      delay(100);
       BluetoothSerial.println("=============================================================");
      BluetoothSerial.println("Aligning right");
       BluetoothSerial.println("=============================================================");
      AlignToWall(true);
    } else if (FR_IR_Data[0] > FL_IR_Data[0]) {
       BluetoothSerial.println("=============================================================");
      BluetoothSerial.println("Strafing left");
       BluetoothSerial.println("=============================================================");
      StrafeDistance(15, true, iAngle);
      delay(100);
       BluetoothSerial.println("=============================================================");
      BluetoothSerial.println("Aligning left");
       BluetoothSerial.println("=============================================================");
      AlignToWall(false);
    }
  } else if ((FL_IR_Data[0] > 79) && (FR_IR_Data[0] > 79)) {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("LR sensors out of range (middle of wall)");
    BluetoothSerial.println("Turning 90 degrees clockwise");
     BluetoothSerial.println("=============================================================");
    TurnByAngle(80);

  } else if ((FL_IR_Data[0] < 79) && (FR_IR_Data[0] > 79)) {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Strafing Left 2");
     BluetoothSerial.println("=============================================================");
    StrafeDistance(15, true, iAngle);
  } else if ((FR_IR_Data[0] < 79) && (FL_IR_Data[0] > 79)) {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Strafing right 2");
     BluetoothSerial.println("=============================================================");
    StrafeDistance(15, false, iAngle);
  } 
  else {
     BluetoothSerial.println("=============================================================");
    BluetoothSerial.println("Random 90 degree CW turn");
     BluetoothSerial.println("=============================================================");
    TurnByAngle(80);
    BluetoothSerial.println((String)"Front Left: " + FL_IR_Data[1] + (String)"Front Right: " + FR_IR_Data[1]);
  }
  delay(100);
  BluetoothSerial.println("Driving straight to corner (hopefully)");
  iAngle = gyro_read();
  //SonarDistance(15, iAngle, false);
  gyro_forward(15,iAngle);
  stop();
  delay(100);
  BluetoothSerial.println("OMFG WE ARE IN A CORNER!!!!");
}

// Rotate platform by a specified angle in degrees using PI control (+ve input = clockwise, -ve input = counter-clockwise).
void TurnByAngle(int turnAngle)
{
  BluetoothSerial.println("=========================================================================================");
  BluetoothSerial.println("STARTING TURN BY ANGLE");
  BluetoothSerial.println("=======================================================================================");
  currentAngle = 0;
  previous_millis = millis();
  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -500, 500};
  float gyroError[] = {0, 0};
  float gyroGains[] = {4, 4, 0.001};
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
    if ((gyroDerivative < 0.5) && (abs(gyroError[1]) < 2)) {
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

    BluetoothSerial.println((String)"error: " + gyroError[1] + (String)", u: " + angleEffort + (String)" Angle: " + gyroAngle + (String)" wrap: " + wrapCheck);
  }
  stop();
  BluetoothSerial.println("==============================================================================");
  BluetoothSerial.println("TURN BY ANGLE IS COMPLETE!");
  BluetoothSerial.println("==============================================================================");
}

// Drive straight and stop a certain distance in cm away from an object detected in front of the robot.

void SonarDistance(float target, float initialAngle, boolean isMiddle) {

  BluetoothSerial.println((String)"Driving to " + target + (String)"cm");
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

    //BluetoothSerial.println((String)"Error: " + sonarError[1] + (String)(" sonar: ") + sonar[0] + (String)", current angle: " + current_Angle + (String)" initial angle " + initialAngle + (String)" Adjustment: " + angleEffort + (String)" timer: " + timer);
    BluetoothSerial.println((String)" u is: " + u + (String)" sonar error: " + sonar[0] + (String)" gyro error: " + gyroError[1] + (String)" integral: " + gyroIntegral + (String)", effort: " + angleEffort + (String)" derivative: " + gyroDerivative + (String)" current Angle " + current_Angle + (String)" timer: " + timer);

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(sonarDerivative) < 10) && (abs(sonarError[1]) < 10) && (abs(gyroError[1]) < 1)) {
      timer -= 100;
    }
    else {
      timer = 500;
    }
    delay(100); // ~10Hz

  } while (((sonar != -1) && (timer > 0)) && (isMiddle || ((frontL[0] > 20) && (frontR[0] > 20)))); // Terminate once within desired tolerance.
  stop();
  BluetoothSerial.println((String)"frontL[0]" + frontL[0] + (String)"cm" + (String)"frontR[0]" + frontR[0] + (String)"cm");

  if (sonar == -1) {
    BluetoothSerial.println("ERROR: INVALID SONAR READING!!!");
  }
  delay(100);
}

void GyroDistance( float initialAngle) {

  BluetoothSerial.println((String)"Straightening ");

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

    BluetoothSerial.println( (String)" gyro error: " + gyroError[1] + (String)" integral: " + gyroIntegral + (String)", effort: " + angleEffort + (String)" derivative: " + gyroDerivative + (String)" current Angle " + current_Angle + (String)" timer: " + timer);

    // Loop exits if error remains in steady state for at least 500ms.
    if ((abs(gyroDerivative) < 10) && (abs(gyroError[1]) < 0.5)) {
      timer -= 100;
    }
    else {
      timer = 500;
    }
    delay(100); // ~10Hz

  } while (timer > 0); // Terminate once within desired tolerance.
  stop();
  delay(100);
}
// Strafe to a specified distance from a wall using average reading from IR sensors.
void StrafeDistance(float target, boolean isLeft, float initialAngle) {
  float actualTarget = target - 7;
  BluetoothSerial.println((String)"Strafing to " + actualTarget + (String)"cm");
  // Initialise variables
  float u = 0;
  float uLimit[] = { -120, 120}; //Limit maximuim effort signal for sonar. -250,250
  float irError[] = {0, 0};
  float irGains[] = {20, 18, 2.5}; // Kp, Ki, and Kd gains for sonar
  float irIntegral = 0;
  float irDerivative = 0.2;
  float irFront[] = {0, 999};
  float irBack[] = {0, 999};

  float angle = 0;
  float angleEffort = 0;
  float angleEffortLimit[] = { -150, 150};
  float gyroError[] = {0, 0}; // [lastError,error] for gyro
  float gyroGains[] = {12, 0.1, 0}; // Kp, Ki, and Kd gains for gyro
  float gyroIntegral = 0;
  float gyroDerivative = 0;
  float angleIntegralLimit = angleEffortLimit[1] / gyroGains[0] / 2;

  //float initialAngle = gyro_read();
  float current_Angle = initialAngle;
  float integralLimit = (uLimit[1] / irGains[0]) / 2; // Set max error boundary for integral gain to be applied to control system
  float gyroIntegralLimit = angleEffortLimit[1] / gyroGains[0]; // Set max error boundary for integral gain to be applied to control system
  int timer = 300; // Initialise tolerance timer.


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
      irError[1] = (irFront[0] - (target - 7)); // Error is average difference between IR sensors and target distance.
    } else {
      FR_IR(irFront); // Front right IR sensor reading
      BR_IR(irBack); // Back right IR sensor reading
      irError[1] = ( irFront[0] - (target - 7)); // Error is average difference between IR sensors and target distance.
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
    BluetoothSerial.println( (String)" IR is: " + irFront[0] + (String)" Error: " + irError[1] + (String)", uStrafe: " + u + (String)" derivative: " + irDerivative + (String)" gyroerror: " + gyroError[1] + (String)" timer: " + timer);

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
  BluetoothSerial.println("Strafing Complete");
  stop();
  delay(100);
}

// Strafe to a specified distance from a wall using average reading from IR sensors.
void StrafeTime(float timeToStrafe, boolean isLeft, float initialAngle) {

  BluetoothSerial.println((String)"Strafing for " + timeToStrafe + (String)" seconds");
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

    //+VE IS CW
    BluetoothSerial.println( (String)" STRAFING initial angle is:" + initialAngle + "Gyro is: " + current_Angle + (String)" Error: " + gyroError[1] + (String)", angle effort: " + angleEffort + (String)" timer: " + timer);

    // Check which sensors to read based on input parameter.
    if (!isLeft) {
      left_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
      left_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
      right_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
      right_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
    } else {
      left_font_motor.writeMicroseconds(1500 - (150 + angleEffort));
      left_rear_motor.writeMicroseconds(1500 + (150 - angleEffort));
      right_rear_motor.writeMicroseconds(1500 + ( 150 - angleEffort));
      right_font_motor.writeMicroseconds(1500 - ( 150 + angleEffort));
    }
    delay(100); // ~10Hz
  } while (timeToStrafe > 0); // Terminate once within desired tolerance.

  // This corrects angle if the motor stops running
  while (timer > 0) {
    gyroError[1] =  current_Angle - initialAngle; // Calculate angle error (relative to starting angle).

    PID_Control(gyroError, gyroGains, &gyroDerivative, &gyroIntegral, &gyroIntegralLimit, &angleEffort, angleEffortLimit); // Calculate control effort for angle correction using PID control.
      if(abs(gyroError[1]) < 1){
        timer = timer - 100;
      }else{
        timer = 300;
      }
      
      //+VE IS CW
      //BluetoothSerial.println( (String)" STRAFING initial angle is:" + initialAngle+ "Gyro is: " + current_Angle+(String)" Error: " + gyroError[1] + (String)", angle effort: " + angleEffort + (String)" timer: " + timer);

      // Check which sensors to read based on input parameter.
      if(!isLeft){
        left_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
        left_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
        right_rear_motor.writeMicroseconds(1500 - (150 + angleEffort));
        right_font_motor.writeMicroseconds(1500 + (150 - angleEffort));
      } else{
        left_font_motor.writeMicroseconds(1500 - (150 + angleEffort));
        left_rear_motor.writeMicroseconds(1500 + (150 - angleEffort));
        right_rear_motor.writeMicroseconds(1500 + ( 150 - angleEffort));
        right_font_motor.writeMicroseconds(1500 - ( 150 + angleEffort));
      }
      
      if (millis() - prev_millis > 400) {
        CoordUpdate();
        prev_millis = millis();
      }
      
      delay(100); // ~10Hz
    } while (timeToStrafe> 0); // Terminate once within desired tolerance.

    timeToStrafe = timeToStrafe - 100;

    if (abs(gyroError[1]) < 1) {
      timer = timer - 100;
    } else {
      timer = 300;
    }

    if (!isLeft) {
      left_font_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
      left_rear_motor.writeMicroseconds(1500 - (0 - angleEffort * 10));
      right_rear_motor.writeMicroseconds(1500 - (0 - angleEffort * 10));
      right_font_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
    } else {
      left_font_motor.writeMicroseconds(1500 - ( 0 - angleEffort * 10));
      left_rear_motor.writeMicroseconds(1500 + ( 0 + angleEffort * 10));
      right_rear_motor.writeMicroseconds(1500 + (0 + angleEffort * 10));
      right_font_motor.writeMicroseconds(1500 - ( 0 - angleEffort * 10));
    }
    //BluetoothSerial.println("Strafing Complete");
    stop();
    delay(1000);//VERY IMPORTANTTT!!! FOR GYRO TO GO STRAIGHT need enough time for the motors to settle down.
}
//#pragma endregion end

//=============================================================
//#pragma region 4.1 IR SENSOR FUNCTIONS
//=============================================================
void FR_IR(float output[])
{
  int signalADC = analogRead(frontR_IR);
  float distance = 15832 * pow(signalADC, -1.21);
  distance = constrain(distance, 1, 80);
  Kalman(distance, output, 5);
}

void FL_IR(float output[])
{
  int signalADC = analogRead(frontL_IR);
  float distance = 16479 * pow(signalADC, -1.21);
  distance = constrain(distance, 1, 80);
  Kalman(distance, output, 5);

}

void BL_IR(float output[])
{
  int signalADC = analogRead(backL_IR);
  float distance = 7275 * pow(signalADC, -1.19);
  distance = constrain(distance, 1, 30);
  Kalman(distance, output, 5);
}

void BR_IR(float output[])
{
  int signalADC = analogRead(backR_IR);
  float distance = 4921 * pow(signalADC, -1.13);
  distance = constrain(distance, 1, 30);
  Kalman(distance, output, 5);
}

void Kalman(double rawdata, float output[], double sensor_noise) {  // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;
  a_priori_est = output[0];
  a_priori_var = output[1] + process_noise;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_est = constrain(a_post_est, 0, 999); // constrain output to prevent infinite values.
  a_post_var = (1 - kalman_gain) * a_priori_var;
  last_var = a_post_var;
  output[0] = a_post_est;
  output[1] = a_post_var;
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
  //  Serial.print("Time taken for one loop is: ");
  //  Serial.println(T);
  //Serial.print((String)"Current angle: " + currentAngle);

  // control the time per loop
  delay (100);
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

  Serial.print("Potentiometer ");
  Serial.println(analogRead(sensorPin));
  Serial.print("Angular velocity: ");
  Serial.println(angularVelocity);
  Serial.print("Current angle: ");
  Serial.println(currentAngle);

  // control the time per loop
  delay (T);
  return angularVelocity;
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

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";

  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
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
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);;
  }
}
//#pragma endregion end
