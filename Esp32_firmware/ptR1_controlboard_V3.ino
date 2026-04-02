// include
  #include <Arduino.h>
  #include <PCF8575.h>
  #include <ESP32Servo.h>
  #include <ESP32Encoder.h>  
  #include <math.h>                          
  #include <PID_v1.h>
  #include <stdlib.h>             // IMU normalized
  #include <Wire.h>               // ไลบรารีสำหรับ I2C communication
  #include <MPU6050_light.h>           // Library MPU6050

  #include <ros.h>                            // ไลบรารีสำหรับ ROS communication
  #include <tf/transform_broadcaster.h> 
  #include <nav_msgs/Odometry.h>
  #include <geometry_msgs/Twist.h>            // ใช้สำหรับรับคำสั่งความเร็ว (cmd_vel) จาก ROS
  #include <sensor_msgs/Imu.h>                // ใช้สำหรับส่งข้อมูลจาก IMU ไปยัง ROS
  #include <sensor_msgs/MagneticField.h>      // เพิ่ม Header Mag
  #include <geometry_msgs/Vector3.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <std_msgs/String.h>                // ใช้สำหรับส่งข้อมูลเป็นข้อความ
  #include <std_msgs/UInt32.h>                // ใช้สำหรับส่งค่าประเภท UInt32 ใน ROS
  #include <std_msgs/UInt16.h>                // ใช้สำหรับส่งค่าประเภท UInt16 ใน ROS
  #include <std_msgs/UInt8.h>                 // ใช้สำหรับส่งค่าประเภท UInt8 ใน ROS
  #include <std_msgs/Int16.h>

  #include <WiFi.h>
  #include <ArduinoOTA.h>

  #include <FastLED.h>

#define LED_PIN     48
#define NUM_LEDS    1
CRGB leds[NUM_LEDS];

const char* ssid = "Jeanne";
const char* password = "none";

#define I2C_SDA 8
#define I2C_SCL 9

bool debug_mode = false;
bool manual_mode = false;
bool enable_lpf = true;
double lpf_alpha = 0.25;
byte mpu_status = 255;

//---พินของมอเตอร์ไดรเวอร์ (Motor Driver)---
  const uint8_t md1_PWMA  = 6; // to esp32 pin
  const uint8_t md1_AIN2  = 12;   // PCF8575 pin 
  const uint8_t md1_AIN1  = 11;   // PCF8575 pin 
  const uint8_t md1_STBY  = 2;    // PCF8575 pin
  const uint8_t md1_BIN1  = 3;    // PCF8575 pin
  const uint8_t md1_BIN2  = 4;    // PCF8575 pin
  const uint8_t md1_PWMB  = 7; // to esp32 pin

                      
  const uint8_t md2_PWMA = 10; //esp32 pin
  const uint8_t md2_AIN2 = 5;   // PCF8575 pin
  const uint8_t md2_AIN1 = 6;   // PCF8575 pin
  const uint8_t md2_STBY = 7;   // PCF8575 pin
  const uint8_t md2_BIN1 = 8;   // PCF8575 pin
  const uint8_t md2_BIN2 = 9;   //PCF8575 pin
  const uint8_t md2_PWMB = 21; //esp32 pin

//---PCF8575 Pins (0-15)---
  const uint8_t FL_IN1_PCF = md1_AIN1; const uint8_t FL_IN2_PCF = md1_AIN2;
  const uint8_t FR_IN1_PCF = md1_BIN1;  const uint8_t FR_IN2_PCF = md1_BIN2;
  const uint8_t RL_IN1_PCF = md2_AIN1;  const uint8_t RL_IN2_PCF = md2_AIN2;
  const uint8_t RR_IN1_PCF = md2_BIN1;  const uint8_t RR_IN2_PCF = md2_BIN2;
  const uint8_t STBY_PCF_1 = md1_STBY;  const uint8_t STBY_PCF_2 = md2_STBY;

  const uint8_t FL_PWM_PIN = md1_PWMA;
  const uint8_t FR_PWM_PIN = md1_PWMB;
  const uint8_t RL_PWM_PIN = md2_PWMA;
  const uint8_t RR_PWM_PIN = md2_PWMB;

//---Encoder pin---
  // Motor 1: Front-Left (FL)
    const uint8_t ENCODER_FL_A = 13;
    const uint8_t ENCODER_FL_B = 14;

    // Motor 3: Front-Right (FR)
    const uint8_t ENCODER_FR_A = 12;
    const uint8_t ENCODER_FR_B = 11;

    // Motor 4: Rear-Left (RL)
    const uint8_t ENCODER_RL_A = 17;
    const uint8_t ENCODER_RL_B = 18;

    // Motor 2: Rear-Right (RR)
    const uint8_t ENCODER_RR_A = 16;
    const uint8_t ENCODER_RR_B = 15;

// --- Robot Parameters ---
  const float WHEEL_RADIUS = 0.04; 
  const float LX = 0.105; // L1 (Half length) m
  const float LY = 0.0825; // L2 (Half width) m
  const float ROBOT_GEOMETRY = LX + LY;
  const float TICKS_PER_REV = 1320.0;
  const float RADS_PER_TICK = (2.0 * PI) / TICKS_PER_REV;

  const int MIN_PWM = 13;

// --- Power variable ---
  const uint8_t CURRENT_SENSOR_PIN = 4; // GPIO ที่ต่อ Current sensor
  const uint8_t VOLTAGE_SENSOR_PIN = 5; // GPIO ที่ต่อ Voltage sensor
  // --- Power sensor calibration Values and variable ---
    // ESP32 ADC 12-bit = 4095
    // V_REF ของ ESP32 ประมาณ 3.3V
    const float ADC_VREF = 3.2; 
    const float ADC_RES = 4095.0;
    const float VOLTAGE_MULTIPLIER = 10.253;
    int current_zero_point = 0;
    float voltage = 0;
    float current = 0;
  //Exponential Moving Average (EMA) 
  float filter_volt = 0; //ของ current sensor
  float filter_amp = 0;  //ของ current sensor
  const float alpha_volt = 0.05; // กรองเยอะขึ้น แกว่งน้อยลง
  const float alpha_curr = 0.1;  // current ต้องตอบสนองเร็วกว่า
  float filter_amp_adc = 0; // เก็บค่า ADC ที่กรองแล้ว ของ current sensor

// --- PID object and variables ---
  double Kp = 5.5, Ki = 30, Kd = 0.0;
  // PID Objects  setpoint(ROS) input(Encoder) output(PID)
  double sp_FL=0, in_FL=0, out_FL=0;
  double sp_FR=0, in_FR=0, out_FR=0;
  double sp_RL=0, in_RL=0, out_RL=0;
  double sp_RR=0, in_RR=0, out_RR=0;

  PID pidFL(&in_FL, &out_FL, &sp_FL, Kp, Ki, Kd, DIRECT);
  PID pidFR(&in_FR, &out_FR, &sp_FR, Kp, Ki, Kd, DIRECT);
  PID pidRL(&in_RL, &out_RL, &sp_RL, Kp, Ki, Kd, DIRECT);
  PID pidRR(&in_RR, &out_RR, &sp_RR, Kp, Ki, Kd, DIRECT);
  // --- Timing(PID Cycle) ---
  unsigned long prevPIDTime = 0;
  //Ramp Filter
  double target_FL = 0, target_FR = 0, target_RL = 0, target_RR = 0;
  const double RAMP_STEP = 10.0; // ค่าความชันการเร่ง


//---Encoder variable---
  ESP32Encoder encoderFL;
  ESP32Encoder encoderFR;
  ESP32Encoder encoderRL;
  ESP32Encoder encoderRR;
//---Servo object and variables---
  Servo servoPan;
  Servo servoTilt;
  const uint8_t SERVO_PAN_PIN  = 2;    //X
  const uint8_t SERVO_TILT_PIN = 1;    //Y

  int pos_pan = 1500;  // 1500us = 90 degrees (Center)
  int pos_tilt = 1000;
  //int step_servo_x = 50; // Step การขยับ
  //int step_servo_y = 50;
  const int SERVO_MIN = 500;
  const int SERVO_MAX = 2500;
//---Relay variable---
  const uint8_t RELAY2_PCF = 13;

// --- Global Variables for Odometry ---
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;
  double linear_x = 0;
  double linear_y = 0;
  double gyro_z = 0;
  geometry_msgs::Quaternion odom_quat;

//---ROS Globals and Publisher---
  ros::NodeHandle nh;
  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::TransformStamped t;
  std_msgs::String status_msg;
  std_msgs::Float32MultiArray wheels_msg;
  tf::TransformBroadcaster broadcaster;
  std_msgs::Float32MultiArray setpoint_msg;


  char base_link[] = "base_link";
  char odom_frame[] = "odom";


  ros::Publisher odom_pub("/odom", &odom_msg);
  ros::Publisher pub_imu("/imu/data_raw", &imu_msg);
  ros::Publisher status_pub("/robot/status", &status_msg);
  ros::Publisher pub_wheels("/wheel_velocities", &wheels_msg);
  ros::Publisher pub_setpoints("/wheel_setpoints", &setpoint_msg);

  char status_buffer[150]; //ใช้สำหรับ Topic /robot/status

//---Sensor Object---
  PCF8575 pcf(0x20);
  uint16_t pcf_buffer = 0xFFFF;

  MPU6050 mpu(Wire);
//---Heading Hold variable---
  bool enable_heading_hold = true; // เปิด/ปิด ระบบนี้
  float heading_kp = 2.0;          // ค่า Kp: ยิ่งมาก ยิ่งสู้แรงไถล
  double target_heading = 0.0;     // มุมที่เราต้องการล็อกไว้
  bool is_turning = false;         // เช็คว่าตอนนี้กำลังตั้งใจหมุนอยู่ไหม

//---Function declare---
  void setPCFBit(uint8_t pin, bool state);
  void setMotorPWM(float pwm, int pin1, int pin2, int pwmPin);
  void configPWMPin(uint8_t pin);
  void updateOdometryAndIMU(float dt, double v_fl, double v_fr, double v_rl, double v_rr);
  void panCallback(const std_msgs::Int16& msg);
  void tiltCallback(const std_msgs::Int16& msg);
  void pidCallback(const geometry_msgs::Vector3& msg);
  void sysCommandCallback(const std_msgs::String& msg);
  void cmdVelCallbackAuto(const geometry_msgs::Twist& msg);
  void cmdVelCallbackManual(const geometry_msgs::Twist& msg);
  
//---Ros Subscriber---
  ros::Subscriber<std_msgs::Int16> subPan("/camera/pan", panCallback);
  ros::Subscriber<std_msgs::Int16> subTilt("/camera/tilt", tiltCallback);
  ros::Subscriber<geometry_msgs::Vector3> subPID("/config/pid", pidCallback);
  ros::Subscriber<std_msgs::String> subSys("/robot/cmd", sysCommandCallback);
  ros::Subscriber<geometry_msgs::Twist> subCmdVelManual("/robot/cmdvel_manual", cmdVelCallbackManual);
  ros::Subscriber<geometry_msgs::Twist> subCmdVelAuto("/cmd_vel", cmdVelCallbackAuto);
  

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  WiFi.begin(ssid, password);


  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(30);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
      delay(500);
  }
    // ตั้งค่า OTA
  ArduinoOTA.setHostname("ptR1_ESP32S3");
  ArduinoOTA.begin();
  //Setup PCF8575
  pcf.begin();
  pcf_buffer = 0x0000;
  setPCFBit(STBY_PCF_1, HIGH);
  setPCFBit(STBY_PCF_2, HIGH);
  //Setup Relays (PCF8575)
  setPCFBit(RELAY2_PCF, 1);
  pcf.write16(pcf_buffer);

  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  servoPan.attach(SERVO_PAN_PIN, SERVO_MIN, SERVO_MAX);
  servoTilt.attach(SERVO_TILT_PIN, SERVO_MIN, SERVO_MAX);

  //สั่งให้ไปที่จุดกึ่งกลาง 
  servoPan.writeMicroseconds(pos_pan);
  servoTilt.writeMicroseconds(pos_tilt);

  //Setup IMU (MPU6050)
  leds[0] = CRGB::Red;     
  FastLED.show();

  mpu_status = mpu.begin();
  if (mpu_status == 0) {
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(2000);
    mpu.calcGyroOffsets();
  }
  leds[0] = CRGB::Green;
  FastLED.show();

  setupSensors(); 
  //Setup Motors & Encoders
  configPWMPin(FL_PWM_PIN);
  configPWMPin(FR_PWM_PIN);
  configPWMPin(RL_PWM_PIN);
  configPWMPin(RR_PWM_PIN);

  pinMode(ENCODER_FL_A, INPUT_PULLUP); pinMode(ENCODER_FL_B, INPUT_PULLUP);
  pinMode(ENCODER_FR_A, INPUT_PULLUP); pinMode(ENCODER_FR_B, INPUT_PULLUP);
  pinMode(ENCODER_RL_A, INPUT_PULLUP); pinMode(ENCODER_RL_B, INPUT_PULLUP);
  pinMode(ENCODER_RR_A, INPUT_PULLUP); pinMode(ENCODER_RR_B, INPUT_PULLUP);


  // Setup Encoder FL
  encoderFL.attachFullQuad(ENCODER_FL_A, ENCODER_FL_B); // โหมด Quadrature (x4) ละเอียดสุด
  encoderFL.setCount(0); // รีเซ็ตค่าเริ่มต้น
  // Setup Encoder FR
  encoderFR.attachFullQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoderFR.setCount(0);
  // Setup Encoder RL
  encoderRL.attachFullQuad(ENCODER_RL_A, ENCODER_RL_B);
  encoderRL.setCount(0);
  // Setup Encoder RR
  encoderRR.attachFullQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoderRR.setCount(0);

  //Setup PID
  pidFL.SetMode(AUTOMATIC); pidFL.SetOutputLimits(-255, 255);
  pidFR.SetMode(AUTOMATIC); pidFR.SetOutputLimits(-255, 255);
  pidRL.SetMode(AUTOMATIC); pidRL.SetOutputLimits(-255, 255);
  pidRR.SetMode(AUTOMATIC); pidRR.SetOutputLimits(-255, 255);

  //จองพื้นที่สำหรับความเร็วของทั้ง 4 ล้อ
  wheels_msg.data_length = 4;
  wheels_msg.data = (float *)malloc(sizeof(float) * 4);

  nh.getHardware()->setBaud(921600);
  nh.initNode();
  
  broadcaster.init(nh);
  nh.subscribe(subCmdVelManual);
  nh.subscribe(subCmdVelAuto);
  nh.subscribe(subPan);
  nh.subscribe(subTilt);
  nh.subscribe(subPID);
  nh.subscribe(subSys);

  nh.advertise(pub_imu);
  nh.advertise(odom_pub);
  nh.advertise(status_pub);
  nh.advertise(pub_wheels);
  nh.advertise(pub_setpoints);

  imu_msg.header.frame_id = "imu_link";
  setpoint_msg.data_length = 4;
  setpoint_msg.data = (float *)malloc(sizeof(float) * 4);
}

// Global variables หรือ static ภายใน loop
  unsigned long prevPIDMicros = 0;
  const unsigned long PID_INTERVAL_US = 5000; // 5ms = 200Hz

  unsigned long prevPubMillis = 0;
  const unsigned long PUB_INTERVAL_MS = 50; // 50ms = 20Hz

  unsigned long prevHeartbeat = 0;
  const unsigned long HEARTBEAT_INTERVAL = 1000;

  unsigned long lastCmdTime = 0;
  const unsigned long CMD_TIMEOUT = 300; // watchdog

  unsigned long wifi_previousMillis = 0;
  const long wifi_interval = 30000;

  unsigned long prevVoltRead = 0;

  int compassCounter = 0;

  unsigned long last_sync_time = 0;

// --- CPU Load Variables ---
  unsigned long active_micros_acc = 0;
  unsigned long last_cpu_measure_time = 0;
  float cpu_usage_percent = 0.0;

void loop() {
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();

  if (currentMillis - last_sync_time >= 10000) {
    nh.requestSyncTime();
    last_sync_time = currentMillis;
  }

  // ถ้าผ่านไป 30 วิ และ เน็ตหลุดอยู่ (WL_CONNECTED คือต่อติด)
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifi_previousMillis >= wifi_interval)) {
    WiFi.disconnect(); // ตัดการเชื่อมต่อเก่าที่ค้าง
    WiFi.reconnect();  // สั่งให้ต่อใหม่
    wifi_previousMillis = currentMillis; // รีเซ็ตเวลาเพื่อนับใหม่
  }

  bool is_timeout = (currentMillis - lastCmdTime > CMD_TIMEOUT);
  bool is_disconnected = !nh.connected();

  if (is_disconnected) {
    setMotorPWM(0, FL_IN1_PCF, FL_IN2_PCF, FL_PWM_PIN);
    setMotorPWM(0, FR_IN1_PCF, FR_IN2_PCF, FR_PWM_PIN);
    setMotorPWM(0, RL_IN1_PCF, RL_IN2_PCF, RL_PWM_PIN);
    setMotorPWM(0, RR_IN1_PCF, RR_IN2_PCF, RR_PWM_PIN);
    
    setPCFBit(STBY_PCF_1, LOW);
    setPCFBit(STBY_PCF_2, LOW);
    pcf.write16(pcf_buffer); 

    out_FL = 0; out_FR = 0; out_RL = 0; out_RR = 0;
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;

    pidFL.SetMode(MANUAL); pidFR.SetMode(MANUAL); 
    pidRL.SetMode(MANUAL); pidRR.SetMode(MANUAL);

    nh.spinOnce();
    delay(10);
    return; 
  }
  else {
    if(pidFL.GetMode() == MANUAL) { 
        pidFL.SetMode(AUTOMATIC); pidFR.SetMode(AUTOMATIC);
        pidRL.SetMode(AUTOMATIC); pidRR.SetMode(AUTOMATIC);
    }
  }

  if (is_timeout) {
    // แค่ตั้งเป้าหมายความเร็วให้เป็น 0 เพื่อให้รถหยุดอย่างนุ่มนวลผ่าน PID
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    out_FL = 0; out_FR = 0; out_RL = 0; out_RR = 0;
    pidFL.SetMode(MANUAL); pidFR.SetMode(MANUAL); pidRL.SetMode(MANUAL); pidRR.SetMode(MANUAL);// ปิด PID ชั่วคราว
  }else {
    if (pidFL.GetMode() == MANUAL) {
        pidFL.SetMode(AUTOMATIC); pidFR.SetMode(AUTOMATIC);
        pidRL.SetMode(AUTOMATIC); pidRR.SetMode(AUTOMATIC);
    }
  }


  //PID Loop 
  unsigned long currentMicros = micros();
  if (currentMicros - prevPIDMicros >= PID_INTERVAL_US) {
    unsigned long work_start = micros();
    float dt = (currentMicros - prevPIDMicros) / 1000000.0; // แปลง us เป็น seconds
    prevPIDMicros = currentMicros;
    //delayMicroseconds(1000);   --------------------------------------------------->  20% CPU Injection Test

    // --- 1. อ่าน Encoder ---
    long curFL = encoderFL.getCount();
    long curFR = encoderFR.getCount();
    long curRL = encoderRL.getCount();
    long curRR = encoderRR.getCount();

    static long oldFL=0, oldFR=0, oldRL=0, oldRR=0;
    
    double ticks_to_rads = RADS_PER_TICK / dt;

    // คำนวณความเร็วดิบก่อน (Raw Velocity)
    double raw_FL = (curFL - oldFL) * ticks_to_rads;
    double raw_FR = (curFR - oldFR) * ticks_to_rads;
    double raw_RL = (curRL - oldRL) * ticks_to_rads;
    double raw_RR = (curRR - oldRR) * ticks_to_rads;

    // เลือกใช้แบบผ่าน Filter หรือไม่ผ่าน
    if (enable_lpf) {
        in_FL = (in_FL * (1.0 - lpf_alpha)) + (raw_FL * lpf_alpha);
        in_FR = (in_FR * (1.0 - lpf_alpha)) + (raw_FR * lpf_alpha);
        in_RL = (in_RL * (1.0 - lpf_alpha)) + (raw_RL * lpf_alpha);
        in_RR = (in_RR * (1.0 - lpf_alpha)) + (raw_RR * lpf_alpha);
    } else {
        in_FL = raw_FL; in_FR = raw_FR; in_RL = raw_RL; in_RR = raw_RR;
    }

    oldFL = curFL; oldFR = curFR; 
    oldRL = curRL; oldRR = curRR;

    //Odometry calculate and update
    calculateOdometry(dt, in_FL, in_FR, in_RL, in_RR);
    applyRampFilter(); 
    //PID Compute
    pidFL.Compute(); pidFR.Compute(); pidRL.Compute(); pidRR.Compute();

    //Motor Output
    setMotorPWM(out_FL, FL_IN1_PCF, FL_IN2_PCF, FL_PWM_PIN);
    setMotorPWM(out_FR, FR_IN1_PCF, FR_IN2_PCF, FR_PWM_PIN);
    setMotorPWM(out_RL, RL_IN1_PCF, RL_IN2_PCF, RL_PWM_PIN);
    setMotorPWM(out_RR, RR_IN1_PCF, RR_IN2_PCF, RR_PWM_PIN);

    wheels_msg.data[0] = in_FL * WHEEL_RADIUS;  // หน้าซ้าย (m/s)
    wheels_msg.data[1] = in_FR * WHEEL_RADIUS;  // หน้าขวา (m/s)
    wheels_msg.data[2] = in_RL * WHEEL_RADIUS;  // หลังซ้าย (m/s)
    wheels_msg.data[3] = in_RR * WHEEL_RADIUS;  // หลังขวา (m/s)

    // Send I2C Batch
    setPCFBit(STBY_PCF_1, HIGH);
    setPCFBit(STBY_PCF_2, HIGH);
    pcf.write16(pcf_buffer);
    active_micros_acc += (micros() - work_start);
  }
  if (currentMillis - prevPubMillis >= PUB_INTERVAL_MS) {
    unsigned long work_start = micros();
    prevPubMillis = currentMillis;
    publishOdometryAndTF();
    pub_wheels.publish(&wheels_msg);
    active_micros_acc += (micros() - work_start); 
  }
  if (currentMillis - prevHeartbeat >= HEARTBEAT_INTERVAL) {
      prevHeartbeat = currentMillis;
      publishRobotStatus();
    }
  if (currentMillis - prevVoltRead >= 100) { 
    prevVoltRead = currentMillis;
    voltage = readVoltageEMA();
    current = readCurrentEMA();
    }
  
  unsigned long work_start = micros();
  nh.spinOnce();

  //สะสมเวลาที่ CPU ทำงานจริงในลูปรอบนี้
  active_micros_acc += (micros() - work_start);

  //คำนวณเปอร์เซ็นต์ทุกๆ 1 วินาที (1,000 ms)
  if (currentMillis - last_cpu_measure_time >= 1000) {
      cpu_usage_percent = min((active_micros_acc / 10000.0f), 100.0f);
      active_micros_acc = 0; 
      last_cpu_measure_time = currentMillis;
  }
}

// ฟังก์ชันช่วยคำนวณ Inverse Kinematics (ใช้ร่วมกันทั้ง Auto และ Manual)
void computeWheelSpeeds(float vx, float vy, float w) {
  lastCmdTime = millis(); // อัปเดตเวลาล่าสุดที่ได้รับคำสั่ง (ป้องกัน Watchdog ตัด)

  double w_final = w;

  if (enable_heading_hold && manual_mode) {
    // กรณีที่ 1: เราสั่งให้หยุดหมุน (w = 0) -> เข้าโหมด "ล็อกทิศ"
    if (abs(w) < 0.005) {
      if (is_turning) {
        // เพิ่งหยุดหมุนเมื่อกี้ -> ให้จำมุมปัจจุบันเป็นเป้าหมายใหม่ทันที
        target_heading = theta; 
        is_turning = false;
      }

      // คำนวณ Error (เป้าหมาย - มุมจริง)
      double heading_error = target_heading - theta;

      // แก้ปัญหา Wrap Around (เช่น เป้า 3.14 แต่มุมจริง -3.14 -> error ควรเป็นนิดเดียว)
      if (heading_error > PI)  heading_error -= TWO_PI;
      if (heading_error < -PI) heading_error += TWO_PI;

      // คำนวณค่าชดเชย (P-Controller)
      // ถ้า Error เป็นบวก (หุ่นหันซ้ายเกิน) -> ต้องสั่งลบ (หมุนขวา)
      double w_correction = heading_error * heading_kp;
      
      // เอาค่าชดเชยไปรวมกับ w (ซึ่งตอนนี้เป็น 0)
      w_final = w_correction;

    } 
    // กรณีที่ 2: เราสั่งหมุน (w != 0) -> ปล่อยให้หมุน
    else {
      is_turning = true;
      target_heading = theta; // อัปเดตเป้าหมายตามตัวหุ่นไปเรื่อยๆ
      w_final = w;
    }
  }

  // Inverse Kinematics Mecanum X-Config
  float v_fl = vx - vy - (ROBOT_GEOMETRY * w_final);
  float v_fr = vx + vy + (ROBOT_GEOMETRY * w_final);
  float v_rl = vx + vy - (ROBOT_GEOMETRY * w_final);
  float v_rr = vx - vy + (ROBOT_GEOMETRY * w_final);

  target_FL = v_fl / WHEEL_RADIUS;
  target_FR = v_fr / WHEEL_RADIUS; 
  target_RL = v_rl / WHEEL_RADIUS;
  target_RR = v_rr / WHEEL_RADIUS;
}

void cmdVelCallbackAuto(const geometry_msgs::Twist& msg) {
  if (manual_mode == true) return; 
  computeWheelSpeeds(msg.linear.x, msg.linear.y, msg.angular.z);
}

void cmdVelCallbackManual(const geometry_msgs::Twist& msg) {
  if (manual_mode == false) return;
  computeWheelSpeeds(msg.linear.x, msg.linear.y, msg.angular.z);
  setpoint_msg.data[0] = target_FL* WHEEL_RADIUS;
  setpoint_msg.data[1] = target_FR * WHEEL_RADIUS;
  setpoint_msg.data[2] = target_RL * WHEEL_RADIUS;
  setpoint_msg.data[3] = target_RR * WHEEL_RADIUS;
  pub_setpoints.publish(&setpoint_msg);

}

void panCallback(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servoPan.writeMicroseconds(us);
}

void tiltCallback(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servoTilt.writeMicroseconds(us);
}

void pidCallback(const geometry_msgs::Vector3& msg) {
  Kp = msg.x;
  Ki = msg.y;
  Kd = msg.z;
  updatePIDTunings(); // อัปเดตทันที
  
  // (Optional) ส่ง Log กลับไปบอกว่าเปลี่ยนแล้ว
  nh.loginfo("PID Updated");
}

void sysCommandCallback(const std_msgs::String& msg) {
  String cmd = msg.data;
  bool status_changed = false;
  cmd.trim();
  char buf[50];
  snprintf(buf, sizeof(buf), "GOT:[%s] len=%d", cmd.c_str(), cmd.length());
  nh.loginfo(buf);
  if (cmd == "manual_on") {
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    manual_mode = true; status_changed = true;
  } 
  else if (cmd == "manual_off" || cmd == "auto_on") { 
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    manual_mode = false; status_changed = true;
  }
  else if (cmd == "r2_on")  { setPCFBit(RELAY2_PCF, LOW); pcf.write16(pcf_buffer); status_changed = true; }
  else if (cmd == "r2_off") { setPCFBit(RELAY2_PCF, HIGH); pcf.write16(pcf_buffer); status_changed = true; }
  else if (cmd == "debug_on") { debug_mode = true; status_changed = true; }
  else if (cmd == "debug_off") { debug_mode = false; status_changed = true; }
  else if (cmd == "lpf_on") { enable_lpf = true; status_changed = true; }
  else if (cmd == "lpf_off") { enable_lpf = false; status_changed = true; }
  else if (cmd.startsWith("set_lpf:")) {
    float a = cmd.substring(8).toFloat();
    if (a > 0.0 && a <= 1.0) {
      lpf_alpha = a;
      status_changed = true;
    }
  }
  else if (cmd.startsWith("set_pid:")) {
    String values = cmd.substring(8); 
    float p, i, d;
    if (sscanf(values.c_str(), "%f,%f,%f", &p, &i, &d) == 3) {
       Kp = p;
       Ki = i;
       Kd = d;
       updatePIDTunings();
       status_changed = true;
    }   
  }
  else if (cmd == "report" || cmd == "status") {
      status_changed = true;
  }
  if (status_changed) {
    publishRobotStatus();
  }
}

// --- Helper: Control Motor via PCF8575 & PWM ---
void setMotorPWM(float pwm, int pin1, int pin2, int pwmPin) {
  int speed = abs((int)pwm);

  
  if (speed > 0 && speed < MIN_PWM) {
    speed = MIN_PWM;
  }
  
  if (speed > 255) speed = 255;

  // Update แค่ใน Buffer (ยังไม่ส่ง I2C)
  if (pwm > 0) {
    setPCFBit(pin1, HIGH);
    setPCFBit(pin2, LOW);
  } else if (pwm < 0) {
    setPCFBit(pin1, LOW);
    setPCFBit(pin2, HIGH);
  } else {
    setPCFBit(pin1, LOW);
    setPCFBit(pin2, LOW);
    speed = 0;
  }
  
  // เขียน PWM Direct to ESP32 Pin
  analogWrite(pwmPin, speed);
}

void updatePIDTunings() {
  pidFL.SetTunings(Kp, Ki, Kd);
  pidFR.SetTunings(Kp, Ki, Kd);
  pidRL.SetTunings(Kp, Ki, Kd);
  pidRR.SetTunings(Kp, Ki, Kd);
}

// ฟังก์ชันสำหรับแก้ค่า Bit ในตัวแปร buffer
void setPCFBit(uint8_t pin, bool state) {
  if (state) {
    pcf_buffer |= (1 << pin);  // Set Bit (ให้เป็น 1)
  } else {
    pcf_buffer &= ~(1 << pin); // Clear Bit (ให้เป็น 0)
  }
}

// ฟังก์ชันตั้งค่า PWM สำหรับ ESP32 Core 3.0+
void configPWMPin(uint8_t pin) {
  // ตั้งค่า Resolution และ Frequency ต่อขา
  analogWriteResolution(pin, 8);   // 8-bit (0-255)
  analogWriteFrequency(pin, 20000); // 20kHz
}

void publishOdometryAndTF() {
    ros::Time now = nh.now();

    //Odometry Message
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_link;
    
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.angular.z = gyro_z;

    odom_msg.pose.covariance[0] = 0.01; 
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.1; 

    odom_pub.publish(&odom_msg);

    // TF Broadcast
    t.header.stamp = now;
    t.header.frame_id = odom_frame;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    
    broadcaster.sendTransform(t);

     //  Publish IMU ทุก 10 cycles 
    static uint8_t imu_counter = 0;
    if (++imu_counter >= 10) {
        imu_counter = 0;
        imu_msg.header.stamp = now;  // ใช้ now เดียวกัน
        imu_msg.angular_velocity.z  = gyro_z;
        imu_msg.angular_velocity.x  = 0.0;
        imu_msg.angular_velocity.y  = 0.0;
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
        pub_imu.publish(&imu_msg);
    }
}

void calculateOdometry(float dt, double v_fl, double v_fr, double v_rl, double v_rr) {
    // Control Inputs (จากล้อและ Gyro) ---
    linear_x  = (v_fl + v_fr + v_rl + v_rr) * (WHEEL_RADIUS / 4.0);
    linear_y  = (-v_fl + v_fr + v_rl - v_rr) * (WHEEL_RADIUS / 4.0);

    if (mpu_status == 0) {
    mpu.update();
    double gyro_z_reading = mpu.getGyroZ() * DEG_TO_RAD;

    // dead zone ใหญ่ขึ้นเมื่อ robot กำลังเคลื่อนที่ (มี vibration)
    bool is_moving = (abs(linear_x) > 0.01 || abs(linear_y) > 0.01);
    double dead_zone = is_moving ? 0.05 : 0.015; // 2.86°/s : 0.86°/s

    if (abs(gyro_z_reading) < dead_zone) gyro_z_reading = 0.0;
    gyro_z = gyro_z_reading;
    } else {
        gyro_z = 0.0;
    }

    //คำนวณพิกัด (Basic Odometry) ---
    theta += gyro_z * dt; // เชื่อ Gyro เพียวๆ

    // Normalize มุม Theta ให้อยู่ในช่วง -PI ถึง PI
    if (theta > PI)  theta -= TWO_PI;
    if (theta < -PI) theta += TWO_PI;
    
    // คำนวณตำแหน่ง X, Y
    x_pos += (linear_x * cos(theta) - linear_y * sin(theta)) * dt;
    y_pos += (linear_x * sin(theta) + linear_y * cos(theta)) * dt;

    //อัปเดต Quaternion เตรียม Publish ---
    odom_quat.w = cos(theta / 2.0);
    odom_quat.z = sin(theta / 2.0);
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
}

void setupSensors() {
  analogReadResolution(12); // อ่านละเอียด 12-bit
  analogSetAttenuation(ADC_11db); // อ่านได้เต็มช่วง 0-3.3V
  
  // Calibrate Current Sensor Zero Point (ตอนเปิดเครื่องต้องไม่มีโหลด)
  long sum = 0;
  for(int i=0; i<50; i++) {
    sum += analogRead(CURRENT_SENSOR_PIN);
    delay(2);
  }
   current_zero_point = sum / 50;
  filter_amp_adc = current_zero_point;
  current_zero_point -= 41;
}

float readVoltageEMA() {
    float raw = analogRead(VOLTAGE_SENSOR_PIN);
    
    if (filter_volt == 0.0) {
       filter_volt = raw; 
    } else {
       // ถ้าไม่ใช่ครั้งแรก ค่อยเข้าสูตร Filter
       filter_volt = (filter_volt * (1.0 - alpha_volt)) + (raw * alpha_volt);
    }
    
    // แปลง filter_volt เป็น Voltage ตามสูตรเดิมของคุณ
    float voltage = (filter_volt / ADC_RES) * ADC_VREF * VOLTAGE_MULTIPLIER;
    return voltage;
}

float readCurrentEMA() {
  int raw_adc = analogRead(CURRENT_SENSOR_PIN);

  //เข้าสูตร EMA (กรอง Noise)
  // alpha = 0.1 (เชื่อค่าใหม่ 10%, ค่าเดิม 90%) ช่วยลดการแกว่ง
  filter_amp_adc = (filter_amp_adc * (1.0 - alpha_curr)) + (raw_adc * alpha_curr);

  //เอาค่าที่กรองแล้ว (filter_amp_adc) ไปคำนวณสูตรเดิม
  // ลบค่า Zero Point (Calibration)
  float delta_adc = filter_amp_adc - current_zero_point;
  
  // แปลง ADC -> Voltage ที่ขา ESP
  float delta_volts = (delta_adc / ADC_RES) * ADC_VREF;
  
  // ย้อนกลับ Voltage Divider
  float sensor_volts = delta_volts / 0.647; 
  
  // แปลงเป็น Amps (Sensitivity 0.100 V/A)
  float amps = sensor_volts / 0.100;
  
  return abs(amps); // ส่งกลับเป็น A
}

// ฟังก์ชันช่วยคำนวณการขยับค่าทีละนิด
double stepTowards(double current, double target, double step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return target;
}

void applyRampFilter() {
  sp_FL = stepTowards(sp_FL, target_FL, RAMP_STEP);
  sp_FR = stepTowards(sp_FR, target_FR, RAMP_STEP);
  sp_RL = stepTowards(sp_RL, target_RL, RAMP_STEP);
  sp_RR = stepTowards(sp_RR, target_RR, RAMP_STEP);
}

void publishRobotStatus() {
  // เช็คสถานะโหมดการทำงาน ---
  String modeStr = manual_mode ? "MAN" : "AUTO";
  if (debug_mode) modeStr += "+DBG";
  if (enable_lpf) modeStr += "+LPF"; 

  // เช็คสถานะ Watchdog ---
  unsigned long timeSinceLastCmd = millis() - lastCmdTime;
  String wdStr = (timeSinceLastCmd > CMD_TIMEOUT) ? "STOP" : "OK";

  // เช็คสถานะ Relay (Active LOW) ---
  bool r2 = !((pcf_buffer >> RELAY2_PCF) & 1);

  // อัดทุกอย่างลง Buffer เตรียมส่ง ---
  snprintf(status_buffer, sizeof(status_buffer), 
    "[%s] CPU:%.1f%% | Bat:%.2fV(%.2fA) | WD:%s(%lums) | PID:%.1f,%.1f,%.2f | R:%d",
    modeStr.c_str(),
    cpu_usage_percent,
    voltage, current,
    wdStr.c_str(), timeSinceLastCmd,
    Kp, Ki, Kd,
    r2
  );
  status_msg.data = status_buffer;
  status_pub.publish(&status_msg);
}