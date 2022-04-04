
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

  //Coordinates
  float x = 0;
  float y = 0;
  bool start_printing = 0; //set to 1 right before the first wall follow begins

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
    float frontL[] = {0,999};
    float frontR[] = {0,999};
    float backL[] = {0,999};
    float backR[] = {0,999};

    static unsigned long previous_millis;
    fast_flash_double_LED_builtin();

    /*--------------------------------COURSE START--------------------------------*/
    //  String Delimiter = ", ";
    //    
    //    BluetoothSerial.print((String)"FrontL: " + frontL[0] + " FrontR: " + frontR[0] + " BackL: " + backL[0]+ " backR: " + backR[0]);
    //    BluetoothSerial.print(Delimiter);
    //    BluetoothSerial.print("Hello World!");
    //    BluetoothSerial.print(Delimiter);
    //    BluetoothSerial.println("Hello WORLD" + 99);
    //    delay(100);
    //    }
    
    
    Serial.println("Started the course.");
    //      while (1) {
    //        
    //        FR_IR(frontR);
    //        FL_IR(frontL);
    //        BL_IR(backL);
    //        BR_IR(backR);
    //        
    //        Serial.println((String)"FrontL: " + frontL[0] + (String)" FrontR: " + frontR[0] + " BackL: " + backL[0]+ " backR: " + backR[0]);
    //        delay(100);
    //     } 
    //StrafeDistance(15,false);  
    BluetoothSerial.println("WHERE AM I?");
      Localise();
      //TurnByAngle(90);
      //AlignToWall(true);
      //FindCorner();
    //SonarDistance(15);
    //WallFollow();
    //delay(10000);

    Serial.println("Finished the course.");
    
    stop();
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
  }

//#pragma endregion end

//=============================================================
//#pragma region 3.2 MOTOR MOVEMENT FUNCTION
//=============================================================

  
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
    bool forward = 0;

    if (((FR_IR_Data[0] + BR_IR_Data[0]) / 2) > ((FL_IR_Data[0] + BL_IR_Data[0]) / 2)) {
        //wall is on the left of the robot

        MiddleStrafe();
        
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

        SonarDistance(200 - 15 - (12 / 2)); //12 / 2 should be dist from mid of robot to sonar **NEED TO TUNE**
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
        
        
        MiddleStrafe();

        SonarDistance(15);
        stop();
        /*
        //OLD CODE (for reference)
        ultra = HC_SR04_range();
        while (ultra > 15) {
          initAngle = gyro_read();
          gyro_forward(initAngle);
          if (millis() - prev_millis > 500){
            prev_millis = millis();
            }
        }
        stop();*/

        MiddleStrafe();

        SonarDistance(200 - 15 - (12 / 2)); //12 / 2 should be dist from mid of robot to sonar **NEED TO TUNE**
        stop();

        MiddleStrafe();
        
        //logic to strafe right into the right wall, then wall follow again
    } else {
        //wall is on the right of the robot

        for (int i = 0; i < 3; i++) {
          MiddleStrafe();
          if (forward) {
            SonarDistance(15);
          }
          else {
            SonarDistance(200 - 15 - (12 / 2)); //12 / 2 should be dist from mid of robot to sonar **NEED TO TUNE**
          }
          forward = ~forward;
        }
        
        MiddleStrafe();
        
        //logic to strafe left into the wall, then wall follow again
    }
  }

  void MiddleStrafe() {
    int half_second_count = 0;
    float prev_millis = millis();
    int strafe_time = 1; //[seconds]
    
    while (half_second_count < strafe_time * 2) { //need to be tuned
        strafe_right();
        prev_millis = millis();
        y = y + (half_second_count * (22.5 / (strafe_time * 2)));
        if (millis() - prev_millis > 500) {
          prev_millis = millis();
          half_second_count++;
         }
        }
        stop();
  }

  void FindCorner()
  {
    float FR_IR_Data[]={0,999};
    float FL_IR_Data[]={0,999};
    float BL_IR_Data[]={0,999};
    float BR_IR_Data[]={0,999};

    float ultraDist = HC_SR04_range();

    float iAngle = gyro_read();
    // Serial.print("Initial Ultrasond reading is: ");
    // Serial.print(ultraDist);
    // Serial.print("  Front right IR1: ");
    // Serial.print(FR_IR_Data[0]);
    // Serial.print("  Front left IR2: ");
    // Serial.print(FL_IR_Data[0]);
    // Serial.print("  Back left IR: ");
    // Serial.print(BL_IR_Data[0]);
    // Serial.print("  Back right IR: ");
    // Serial.println(BR_IR_Data[0]);

    //Orientate the robot to face a wall 60cm away
    while (ultraDist > 60) {
      BluetoothSerial.println("Orientate the robot to face a wall 60cm away");
      cw();
      ultraDist = HC_SR04_range();
      // Serial.print("Ultrasond reading is: ");
      // Serial.println(ultraDist);
      delay(50);
    }
    stop();
    delay(1000);
    // Serial.println("Ultrasond reading is less than 60cm !");
    // Serial.println("Driving Straight!");
    SonarDistance(15); // Drive straight until 15cm from front-facing wall

    // Serial.print("Wall Found!");
    stop();
    //delay(5000);

    FL_IR(FL_IR_Data);
    FR_IR(FR_IR_Data);
    BL_IR(BL_IR_Data);
    BR_IR(BR_IR_Data); 

    if (FR_IR_Data[0] < 20) {
      BluetoothSerial.println("Front right near wall");
      AlignToWall(false);
    } else if (FL_IR_Data[0] < 20) {
      BluetoothSerial.println("Front Left Near Wall");
      AlignToWall(true);
    } else if ((FL_IR_Data[0] < 200) && (FR_IR_Data[0] < 200)) {
        BluetoothSerial.println("Facing diagonal corner");
        if (FL_IR_Data[0] > FR_IR_Data[0]) {
          BluetoothSerial.println("Strafing right");
          StrafeDistance(15,false);
  //          while (FR_IR_Data[0] > 15.2) { //calibrate later
  //            strafe_right();
  //            FR_IR(FR_IR_Data);
          // }
          } else if (FR_IR_Data[0] > FL_IR_Data[0]) {
            BluetoothSerial.println("strafing left");
            StrafeDistance(15,true);
  //          while (FL_IR_Data[0] > 15.2) { //calibrate later
  //            strafe_left();
  //            FL_IR(FL_IR_Data);
          // }
          }
      } else if ((FL_IR_Data[0] > 250) && (FR_IR_Data[0] > 250)) {
        BluetoothSerial.println("Turning 90 degrees clockwise");
        TurnByAngle(90);

      } else if ((FL_IR_Data[0] < 250) && (FR_IR_Data[0] > 250)) {
        BluetoothSerial.println("Strafing Left 2");
        StrafeDistance(15,true);
  //      while (FL_IR_Data[0] > 15.2) { //calibrate later
  //        strafe_left();
  //        FL_IR(FL_IR_Data);

        //}
      } else if ((FR_IR_Data[0] < 250) && (FL_IR_Data[0] > 250)) {
        BluetoothSerial.println("Strafing right 2");
        StrafeDistance(15,false);
  //      while (FR_IR_Data[0] > 15.2) { //calibrate later
  //        strafe_right();
  //        FR_IR(FR_IR_Data);
        //}
      }
      else {
        BluetoothSerial.println("Random 90 degree CW turn");
        TurnByAngle(90);
      }

      BluetoothSerial.println("Second Wall Found!");
      //return;
      BluetoothSerial.println("Driving Forward");
      SonarDistance(15); // Drive straight until 15cm from front-facing wall

      BluetoothSerial.println("We are in a CORNER!");
      //Now we are in a corner
      if (BL_IR[0] < 20) {
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
      BluetoothSerial.println("Ready to START MAPPING!");
    }

    void WallFollow() {
      float ultra = HC_SR04_range();
      float error_long, error_short, long_IR, short_IR, left, integral_long, integral_short, travel_angle, speed_long, speed_short, u_long, u_short = 0;
      float target = 8;

      float FR_IR_Data[]={0,999};
      float FL_IR_Data[]={0,999};
      float BL_IR_Data[]={0,999};
      float BR_IR_Data[]={0,999};

      //controller arrays
      float long_feedback[]={0,500};
      float short_feedback[]={0,500};

      // Determining if the wall is on the left or right
      //Serial.println((String)"Initial IR distances are: " + (String)" IR Long Right = " + FR_IR_Data[0] + (String)" IR Long Left = " + FL_IR_Data[0] + (String)" IR Short Right = " + BR_IR_Data[0] + (String) " IR Short Left = " + );

      // Closed loop controls
      // While long and short timers have not settled at same error for more than 500ms and ultra is more than 15cm away.
      while (long_feedback[1] > 0 && short_feedback[1]>0 && ultra > 15) {
        //Rereading sensor values
        FR_IR(FR_IR_Data);
        FL_IR(FL_IR_Data);
        BR_IR(BR_IR_Data);
        BL_IR(BL_IR_Data);
        ultra = HC_SR04_range();
        travel_angle = gyro_read();

        //wrap travel angle
        if(travel_angle>90){
          travel_angle=360-travel_angle;
        }

        //Setting up interupt to start printing coordinates every 0.5sec
        if (start_printing = 0) {
          start_printing = 1;
          x = 0;
          y = 0;
        }

        if ((FR_IR_Data[0] - target) < (FL_IR_Data[0] - target)) { //indicates whether the wall is on left side or right side
          //Serial.println("Wall is on the right!");
          left = 0;
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

        //Calculate errors
        error_long = target - long_IR;
        error_short = target - short_IR;
        //Serial.println((String)"Errors are: " + (String)" Long IR = " + error_long + (String)" Short IR = " + error_short);
      
        controller(error_long, 10, 50, 0.05, 5,0.5,long_feedback);
        controller(error_short, 10, 50, 0.05, 5,0.5,short_feedback);
        
        //  Serial.println((String)"Current Long IR is: " + long_IR + (String)", Error is: " + error_long + (String));
        //  Serial.println((String)"Current Short IR is: " + short_IR + (String)", Error is: " + error_short + (String));

        //speed_long = constrain(long_feedback[0], -500, 500);
        //For some reason this constrain function isn't working, but the one below is :'D

        if (long_feedback[0] < -500) {
          speed_long = -500;
        } else if (long_feedback[0]> 500) {
          speed_long = 500;
        } else {
          speed_long = long_feedback[0];
        }

        speed_short = constrain(short_feedback[0], -500, 500);
        forward(long_feedback[0],short_feedback[0]); //??????????????????? would kind of average out

        //Serial.println((String)" Control Actions are: " + (String)" Long IR = " + u_long + (String)" Short IR = " + u_short);
        //Serial.println((String)" Speed Adjustments are: " + (String)" Right Side = " + speed_long + (String)" Left Side = " + speed_short);

        // if travel angle is big, adjust with gyro
        while (travel_angle > 5) {
          Serial.println("drive straight");
          gyro_forward(travel_angle);
          travel_angle = gyro_read();
        }
        
      }
    }

    void AlignToWall(boolean isLeft) {
      float error, u, lastError, integral, derivative, speed = 0;
      float integralLimit = 10; // Set max error boundary for integral gain to be applied to control system.
      float initialAngle = gyro_read();
      float effort = 0;
      float Kp = 20; // Initialise proportional gain.
      float Ki = 0.05; // Initialise integral gain
      int timer = 500; // Initialise tolerance timer.
      float F_IR_Data[] = {0,999};
      float B_IR_Data[] = {0,999};
      float backL = 0;
      int direction = 0;

      while (timer > 0) {
        if(isLeft){
          FL_IR(F_IR_Data); // Front left IR sensor reading
          BL_IR(B_IR_Data); // Back left IR sensor reading
          direction = 1;
        }
        else {
          FR_IR(F_IR_Data); // Front right IR sensor reading
          BR_IR(B_IR_Data); // Back right IR sensor reading
          direction = -1;
        }
        error = F_IR_Data[0] - B_IR_Data[0]; // Error is difference between readings
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
        if ((derivative < 1) && (error < 5)) {
          
          timer -= 100;
        }
        else {
          timer = 500;
        }

        u = Kp * error + Ki * integral; // Calculate the control effort to reach target distance.
        effort = constrain(u, -500, 500);
        BluetoothSerial.println((String) "error: " + error + (String)" u: " + effort + (String)" d: " + derivative);
        
        if(isLeft){
          left_font_motor.writeMicroseconds(1500 - direction*effort);
          left_rear_motor.writeMicroseconds(1500 - direction*effort);
          right_rear_motor.writeMicroseconds(1500 - direction*effort);
          right_font_motor.writeMicroseconds(1500 - direction*effort);
        }

        delay(100); // Loop repeats at a frequency of ~10Hz

      }
      stop();
    }

    // Strafe left at fixed speed value.
    void strafe_left ()
    {
      left_font_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_font_motor.writeMicroseconds(1500 - speed_val);
    }

    // Strafe right at fixed speed value.
    void strafe_right ()
    {
      left_font_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_font_motor.writeMicroseconds(1500 + speed_val);
    }

    void cw ()
    {
      left_font_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_font_motor.writeMicroseconds(1500 + speed_val);
    }

    // Pivot counter clockwise at a fixed speed value
    void ccw ()
    {
      left_font_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_font_motor.writeMicroseconds(1500 - speed_val);
    }

    void forward(float adjustment){
        //+VE adjustment IS CW
        left_font_motor.writeMicroseconds(1500 + (adjustment-correction));
        left_rear_motor.writeMicroseconds(1500 + (adjustment-correction));
        right_rear_motor.writeMicroseconds(1500 - (adjustment+correction));
        right_font_motor.writeMicroseconds(1500 - (adjustment+correction));
    }

    //Drive straight until hitting a wall
    void driveToWall()
    {
      float FR_IR_Data[]={0,999};
      float FL_IR_Data[]={0,999};
      float BL_IR_Data[]={0,999};
      float BR_IR_Data[]={0,999};
      
      Serial.println("Drive to Wall Started");
      FR_IR(FR_IR_Data);
      FL_IR(FL_IR_Data);
      float initAngle = gyro_read();

      while ((FR_IR_Data[0]> 15) && (FL_IR_Data[0] > 15)) {
        Serial.println("While Loop entered");
        gyro_forward(initAngle);
        FR_IR(FR_IR_Data);
        FL_IR(FL_IR_Data);
      }
      Serial.println("Distance reached");
      stop();
    }

    //Straighten the drone?
    void straighten()
    {
      float FR_IR_Data[]={0,999};
      float FL_IR_Data[]={0,999};
      float BL_IR_Data[]={0,999};
      float BR_IR_Data[]={0,999};

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

    //Motion Function for when the first corner is a CW turn
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
    void gyro_forward(float initialAngle)
    {
      float angleMoved=0;
      float feedback[]={0,500};//controller feedback array, where feedback[0] is u and feedback[1] is timer

      //while timer is greater than 0, the error hasn't been settled for more than 5 ms.
      while(feedback[1]>0){
        //wrap initial angle
        if (initialAngle > 90) {
          initialAngle = 360 - initialAngle;
        }
    
        //wrap angle moved
        if (gyro_read() > 90) {
          angleMoved = (360-gyro_read())-initialAngle;
        }else{
          angleMoved = gyro_read()-initialAngle;
        }
  
        controller(angleMoved, 10, 0.1, 0, 10,1,feedback);
        forward(feedback[0],0);
        Serial.println((String)("error: ") + angleMoved + (String)", adjustment: " + feedback[0]);
      }
    }

    // PI controller helper function
    void controller(float error, float kp, float ki, float kd, float integral_limit, float error_limit, float feedback[]) {
      
      float integral, lastError, derivative= 0;

      integral = integral + error*0.01;

      //to prevent integral windup
      if (error > integral_limit) {
        integral = 0;
      }

      // Calculate derivative of error.
      derivative =  error - lastError;
      lastError = error; // Update last error calculated.

      // Loop exits if error remains in steady state for at least 500ms.
      if ((derivative == 0) && (error < error_limit)) {
        feedback[1]-= 100;
      }
      else {
        feedback[1] = 500;
      }

      feedback[0] = kp * error + ki * integral+ kd*derivative;

      //modifying feedback array to be accessed in the higher level function
      
    }

    void Localise(){
      // Initialise variables
      float FR_IR_Data[] = {0,999};
      float FL_IR_Data[] = {0,999};
      float BL_IR_Data[] = {0,999};
      float BR_IR_Data[] = {0,999};
      
      FL_IR(FR_IR_Data);
      FR_IR(FL_IR_Data);
      BL_IR(BL_IR_Data);
      BR_IR(BR_IR_Data); 

      if (FR_IR_Data[0] < 20) {
        BluetoothSerial.println("Front right near wall... Aligning");
        AlignToWall(false);
      } else if (FL_IR_Data[0] < 20) {
        BluetoothSerial.println("Front Left Near Wall... Aligining");
        AlignToWall(true);
      } else if ((FL_IR_Data[0] < 60) && (FR_IR_Data[0] < 60)) {
          BluetoothSerial.println("Facing diagonal corner");
          if (FL_IR_Data[0] > FR_IR_Data[0]) {
            BluetoothSerial.println("Strafing right");
            StrafeDistance(15,false);
            delay(100);
            BluetoothSerial.println("Aligning right");
            AlignToWall(false);
  //          while (FR_IR_Data[0] > 15.2) { //calibrate later
  //            strafe_right();
  //            FR_IR(FR_IR_Data);
          // }
          } else if (FR_IR_Data[0] > FL_IR_Data[0]) {
            BluetoothSerial.println("Strafing left");
            StrafeDistance(15,true);
            delay(100);
            BluetoothSerial.println("Aligning left");
            AlignToWall(true);
  //          while (FL_IR_Data[0] > 15.2) { //calibrate later
  //            strafe_left();
  //            FL_IR(FL_IR_Data);
          // }
          }
      } else if ((FL_IR_Data[0] > 79) && (FR_IR_Data[0] > 79)) {
        BluetoothSerial.println("LR sensors out of range (middle of wall)");
        BluetoothSerial.println("Turning 90 degrees clockwise");
        TurnByAngle(90);

      } else if ((FL_IR_Data[0] < 79) && (FR_IR_Data[0] > 79)) {
        BluetoothSerial.println("Strafing Left 2");
        StrafeDistance(15,true);
  //      while (FL_IR_Data[0] > 15.2) { //calibrate later
  //        strafe_left();
  //        FL_IR(FL_IR_Data);

        //}
      } else if ((FR_IR_Data[0] < 79) && (FL_IR_Data[0] > 79)) {
        BluetoothSerial.println("Strafing right 2");
        StrafeDistance(15,false);
  //      while (FR_IR_Data[0] > 15.2) { //calibrate later
  //        strafe_right();
  //        FR_IR(FR_IR_Data);
        //}
      }
      else {
        BluetoothSerial.println("Random 90 degree CW turn");
        TurnByAngle(90);
      }
      delay(100);
      BluetoothSerial.println("Driving straight to corner (hopefully)");
      SonarDistance(15);
      stop();
      BluetoothSerial.println("OMFG WE ARE IN A CORNER!!!!");
    }

    // Rotate platform by a specified angle in degrees using PI control (+ve input = clockwise, -ve input = counter-clockwise).
    void TurnByAngle(int turnAngle)
    {
      //Serial.println("Turning clockwise 90 degrees started...");
      //  currentAngle = 0;
      // Initialise variables
      float error, u, lastError, integral, derivative, currentAng, turnSpeed = 0;
      float integralLimit = 5; // Set max error boundary for integral gain to be applied to control system.
      float gyroAngle = gyro_read(); // Fetch and store current angle reading before turning.
      float initialAngle = gyro_read();
      float targetAngle = constrain(turnAngle, -180, 180); // Limit maximum turn angle to +ve or -ve 180 degrees.
      float Kp = 6; // Initialise proportional gain.
      float Ki = 50; // Initialise integral gain
      float Kd = 0.05; // Initialise integral gain
      int timer = 500; // Initialise tolerance timer.
      int direction = turnAngle / abs(turnAngle);
      int wrapCheck = 0;
      BluetoothSerial.println((String)"initial angle is: " + initialAngle + (String)", target angle is: " + targetAngle);
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

        //Serial.println((String)"CurrentAng is: " + currentAng + (String)", Error is: " + error + (String)", gyro reading is: " + gyroAngle + (String)", wrap check = " + wrapCheck);

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
        turnSpeed = (int) constrain(u, -500, 500);
        //Note:
        left_font_motor.writeMicroseconds(1500 + turnSpeed);
        left_rear_motor.writeMicroseconds(1500 + turnSpeed);
        right_rear_motor.writeMicroseconds(1500 + turnSpeed);
        right_font_motor.writeMicroseconds(1500 + turnSpeed);

        delay(100); // Loop repeats at a frequency of ~10Hz

        gyroAngle = gyro_read(); // Get current angle reading from gyroscope sensor (range 0 to 359).
        BluetoothSerial.println((String)"error: " + error + (String)", u: " + turnSpeed + (String)" Angle: " + gyroAngle);
      }
      stop();
    }

    // Drive straight and stop a certain distance in cm away from an object detected in front of the robot.
    void SonarDistance(float target) {
      // Initialise variables
      float error, u, sonar, lastError, integral, derivative, speed = 0;
      float angleMoved = 0;
      float integralLimit = 5; // Set max error boundary for integral gain to be applied to control system.
      float initialAngle = gyro_read();
      float Kp = 10; // Initialise proportional gain.
      float Ki = 50; // Initialise integral gain
      float Kd = 0.05;
      int timer = 500; // Initialise tolerance timer.
      int encError, deltaU;
      float effort = 0;
      float correction;
      float adjustment = 0;
      //wrap initial angle
        if (initialAngle > 90) {
          initialAngle = 360 - initialAngle;
        }
      // PI control loop with additional straighten correction using gyro.
      do {
        sonar = HC_SR04_range(); // Determine distance from object using sonar sensors (in cm) and convert to mm.
        error = sonar - target; // Update the error for distance from target distance.
        // Stop integrating if actuators are saturated.
        if (abs(error) < integralLimit) {
          integral = integral + error*0.01; // Integrate the error with respect to loop frequency (~10Hz).
        }
        else {
          integral = 0; // Disable integral
        }

        // Calculate derivative of error.
        derivative =  error - lastError;
        lastError = error; // Update last error calculated.

        // Loop exits if error remains in steady state for at least 500ms.
        if ((abs(derivative) < 0.5) && abs(error) < 0.05 * abs(target)) {
          timer -= 100;
        }
        else {
          timer = 500;
        }
        
        u = Kp * error + Ki * integral; // Calculate the control effort to reach target distance.
        effort = constrain(u, -450,450);
        angleMoved =  gyro_read() - initialAngle;

        //wrap angle moved
        if (angleMoved > 90) {
          angleMoved = angleMoved - 360;
        }
        adjustment = ki_controller(angleMoved, 10, 0.1);
        correction = constrain(adjustment,-50,50);
        
        //+VE IS CW
        left_font_motor.writeMicroseconds(1500 + (effort - correction));
        left_rear_motor.writeMicroseconds(1500 + (effort - correction));
        right_rear_motor.writeMicroseconds(1500 - (effort + correction));
        right_font_motor.writeMicroseconds(1500 - (effort + correction));
        BluetoothSerial.println((String)"Error: " + error + (String)(" sonar: ") + sonar + (String)", u: " + effort + (String)" anglemoved: " + angleMoved + (String)" Adjustment: " + correction);


        delay(100); // ~10Hz
      } while (timer > 0 || sonar == -1); // Terminate once within desired tolerance.
    }

    // Strafe to a specified distance from a wall using average reading from IR sensors.
    void StrafeDistance(float target, boolean isLeft) {
      // Initialise variables
      float error = 0;
      float u = 0; 
      float sonar = 0;
      float lastError = 0; 
      float integral = 0; 
      float derivative = 0; 
      float angleMoved = 0;
      float effort = 0;
      float correction;
      float adjustment = 0;
      float irFront[] = {0,999};
      float irBack[] = {0,999};
      
      float integralLimit = 5; // Set max error boundary for integral gain to be applied to control system.
      float initialAngle = gyro_read();

      float Kp = 10; // Initialise proportional gain.
      float Ki = 50; // Initialise integral gain
      float Kd = 0.05;
      int timer = 500; // Initialise tolerance timer.
    

      //Wrap initial angle
      if (initialAngle > 90) {
        initialAngle = 360 - initialAngle;
      }
      FL_IR(irFront);
      
    float initialIR = irFront[0];
      // PI control loop with additional straighten correction using gyro.
      do {
        // Check which sensors to read based on input parameter.
        if(isLeft){
          FL_IR(irFront); // Front left IR sensor reading
          BL_IR(irBack); // Back left IR sensor reading
          error = (irBack[0] + irFront[0])/2 - (target -7); // Error is average difference between IR sensors and target distance.
        } else{
          FR_IR(irFront); // Front right IR sensor reading
          BR_IR(irBack); // Back right IR sensor reading
          error = (irBack[0] + irFront[0])/2 - (target-7); // Error is average difference between IR sensors and target distance.
        }

        // Stop integrating if actuators are saturated.
        if (abs(error) < integralLimit) {
          integral = integral + error*0.1; // Integrate the error with respect to loop frequency (~10Hz).
        }
        else {
          integral = 0; // Disable integral
        }

        // Calculate derivative of error.
        derivative =  error - lastError;
        lastError = error; // Update last error calculated.

        // Loop exits if error remains in steady state for at least 500ms.
        if ((abs(derivative) < 0.5) && abs(error) < 0.05 * abs(target - 7)) {
          timer -= 100;
        }
        else {
          timer = 500;
        }
        
        u = Kp * error + Ki * integral + Kd * derivative; // Calculate the control effort to reach target distance.
        effort = constrain(u, -450,450);
        angleMoved =  gyro_read() - initialAngle;

        //wrap angle moved
        if (angleMoved > 90) {
          angleMoved = angleMoved - 360;
        }
        adjustment = ki_controller(angleMoved, 10, 0.1);
        correction = constrain(adjustment,-50,50);
        correction = 0;
        //+VE IS CW
        float test = (irBack[0] + irFront[0])/2;
        float plote = initialIR - error;
        BluetoothSerial.println((String)"Error: " + error + (String)(", measured distance: ") + test + (String)", u: " + effort);
  //      Serial.print("Error:");
  //      Serial.print(target - 7 - error);
  //      Serial.print(",");
  //      Serial.print("reference:");
  //      Serial.println(target - 7);
        // Check which sensors to read based on input parameter.
        if(!isLeft){
          left_font_motor.writeMicroseconds(1500 + (effort - correction));
          left_rear_motor.writeMicroseconds(1500 - (effort - correction));
          right_rear_motor.writeMicroseconds(1500 - (effort + correction));
          right_font_motor.writeMicroseconds(1500 + (effort + correction));
        } else{
          left_font_motor.writeMicroseconds(1500 - (effort - correction));
          left_rear_motor.writeMicroseconds(1500 + (effort - correction));
          right_rear_motor.writeMicroseconds(1500 + (effort + correction));
          right_font_motor.writeMicroseconds(1500 - (effort + correction));
        }

        delay(100); // ~10Hz
      } while (timer >= 0); // Terminate once within desired tolerance.
      stop();
    }

    float ki_controller(float error, float kp, float ki) {
      float integral, u = 0;
      integral = integral + error*0.01;

      //to prevent integral windup
      if (error > 10) {
        integral=0;
      }
      u = kp * error + ki * integral;
      return u;
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
    Kalman(distance, output, 1);
  }

  void FL_IR(float output[])
  {
    int signalADC = analogRead(frontL_IR);
    float distance = 16479 * pow(signalADC, -1.21);
    distance = constrain(distance, 1, 80);
    Kalman(distance, output, 1);

  }

  void BL_IR(float output[])
  {
    int signalADC = analogRead(backL_IR);
    float distance = 7279 * pow(signalADC, -1.19);
    distance = constrain(distance, 1, 30);
    Kalman(distance, output, 1);
   }

  void BR_IR(float output[])
  {
    int signalADC = analogRead(backR_IR);
    float distance = 4921 * pow(signalADC, -1.13);
    distance = constrain(distance, 1, 30);
    Kalman(distance, output, 1);
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
    T = millis() - previous_millis;
    previous_millis = millis();

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
