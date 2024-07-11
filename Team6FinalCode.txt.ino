/********* HOVERCRAFT PROJECT CODE ********************************

    Description: This code will operate the hovercraft through a maze
      using 2 Fans, 1 Servo motor, 2 IR sensors, 1 IMU, and 1
      ultrasonic sensor.

    Team #6

    -------------Members ------------
    Batu Erata 40170153
    David Girma, 40208764
    John Hourani, 40155454
    Mohammed Almasri, 40248819
    Samy Belmihoub, 40251504
    Nihal Islam, 40242307
    ---------------------------------
    
    ENGR 290
    june 18, 2024

********************************************************************/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SharpIR.h>  //Casas-Cartagena, M. (Accessed June, 2024). SharpIR Library [Arduino Library]. Retrieved from https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/
#include <Servo.h>

#include <avr/io.h>
#include <util/twi.h>

// MPU6050 Regs:
#define MPU6050_ADDRESS 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B

// MPU6050 Gyroscope Regs:
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47
#define GYRO_CONFIG     0x1B

// MPU6050 Accelerometer Regs:
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define ACCEL_CONFIG    0x1C

// Sensitivity (for normalization)
#define MPU6050_ACCEL_SENSITIVITY 16384.0  // Accelerometer sensitivity at +/-2g
#define MPU6050_GYRO_SENSITIVITY 131.0     // Sensitivity for gyroscope at +/-250 degrees/sec

// Scale-factors:
#define MPU6050_ACCEL_SCALE_FACTOR_2G  (1.0 / 16384.0)

// Default angle 
#define THRUST_FAN_DEFAULT_ANGLE 90
#define FRONT_DISTANCE_TO_CHECK 40.0
#define TURN_LEFT_SHARP_ANGLE 0
#define TURN_RIGHT_SHARP_ANGLE 155
#define SHARP_TURN_LIMIT_DISTANCE 6
#define TURN_LEFT_NORMAL_ANGLE 36
#define TURN_RIGHT_NORMAL_ANGLE 120


// IR Sensors' library:
#include "SharpIR.h"  //Casas-Cartagena, M. (Accessed April, 2024). SharpIR Library [Arduino Library]. Retrieved from https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/


// Define input pin:
#define IRPinL A1         //IR LEFT A1
#define IRPinR A0         //IR RIGHT A0
#define model 1080        //IR Sensor Model
#define SERVO_PIN 9       //Servo 9


// HS-311 Servo Motor:
Servo servoL;

// PID constants
const double Kp = 2.0, Ki = 0.5, Kd = 1.0;

// Setpoint for yaw, setpoint is 0
const double setpoint = 0;

// Variables for PID calculations:
double input, output;
double error, previous_error = 0, integral = 0, derivative;
unsigned long lastTime;


const int usTrigPin = 11; // US Sensor Trigger pin 11
const int usEchoPin = 2;  // US Sensor Echo pin 2 

const int ThrustFanPin = 6; // Thrust Fan pin 5
const int UpliftFanPin = 5; // Uplift Fan connected to digital pin 6


// Left & Right IR sensor distance variables:
int n_leftDistance;
int n_rightDistance;

// IMU calculated variables:
float yaw;  // Yaw angle
float pitch;  // Pitch angle
float roll;   // Roll angle
float prevAccel = 0.0;  // Previous acceleration


// SharpIR class object declaration::
SharpIR mySensorL = SharpIR(IRPinL, model);
SharpIR mySensorR = SharpIR(IRPinR, model);

long usSensorDistance; // US Sensor distance variable
double timeChange;  // For PID calculations

unsigned long currentTime;  // For PID calculations

// TWI Functions:
void TWI_init() {
    TWBR = 72;  // Bit rate register, baudrate -> 100kHz
    TWSR = 0;   // Set prescaler
}

void TWI_start() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_stop() {
    TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t TWI_read_ack() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t TWI_read_nack() {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void MPU6050_init() {
    // Initialize MPU6050:
    TWI_start();
    TWI_write(MPU6050_ADDRESS << 1);
    TWI_write(MPU6050_REG_PWR_MGMT_1);  // MPU6050_REG_PWR_MGMT_1 register
    TWI_write(0x00);  // Activate MPU6050
    TWI_stop();

    // Configure Gyro range-scale:
    TWI_start();
    TWI_write(MPU6050_ADDRESS << 1);
    TWI_write(GYRO_CONFIG);  
    TWI_write(0x00);  // Gyro: +- 250°/sec
    TWI_stop();

    // Configure Accel range-scale:
    TWI_start();
    TWI_write(MPU6050_ADDRESS << 1);
    TWI_write(ACCEL_CONFIG);  
    TWI_write(0x00);  // Accel: +- 2g
    TWI_stop();
}


int16_t readMPU(uint8_t reg) {

    //Read IMU data:
    int16_t data;
    TWI_start();
    TWI_write(MPU6050_ADDRESS << 1);
    TWI_write(reg);

    TWI_start();
    TWI_write((MPU6050_ADDRESS << 1) | 1);
  
    data = (TWI_read_ack() << 8);
    data |= TWI_read_nack();
  
    TWI_stop();
    return data;
}


int16_t displayMPUData(){
  //Read gyroscope:
  int16_t gyroZ = readMPU(MPU6050_GYRO_ZOUT_H);
  
  //Normalize Gyroscope data:
  float normGyroZ = gyroZ / MPU6050_GYRO_SENSITIVITY;

  Serial.print("  NORM Gyro Z: ");
  Serial.println(normGyroZ);

  
  return (normGyroZ);

}


void setup() {

  Serial.begin(9600);
  servoL.attach(SERVO_PIN); // Set servo pin (9)

  //Pin Mode Initialization:
  pinMode(ThrustFanPin, OUTPUT);
  pinMode(UpliftFanPin, OUTPUT);
  pinMode(usTrigPin, OUTPUT);
  pinMode(usEchoPin, INPUT);

  //Set Both Fans to max
  analogWrite(UpliftFanPin, 255);
  analogWrite(ThrustFanPin, 255);

  lastTime = millis();

  servoL.write(THRUST_FAN_DEFAULT_ANGLE); // Set servo centered (90 deg)
  delay(2000);

  TWI_init();
  MPU6050_init();

  yaw = 0.0;  //initialize yaw
  pitch = 0.0;  //initialize pitch
  roll = 0.0;   //initialize roll

  // PWM Timer2 Initialization:
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  TCCR2A |= (1 << COM2A1); 
  TCCR2B |= (1 << CS20); 
  OCR2A = 0; 

}

void loop() {

  
    int16_t gz;
  
    Serial.println("***************");
    gz = displayMPUData();  // Read the gyroscope Z-axis (yaw rate)

    input = gz;  // Set gyroscope Z-axis as input
    Serial.print("input: ");
    Serial.println(input);
    
    
    // Time Calculations:
    currentTime = millis();
    timeChange = (double)(currentTime - lastTime);


    // PID Calculations:
    error = setpoint - input;  
    integral += error * (timeChange / 1000.0);  // Integral -> sum / time
    derivative = (error - previous_error) / (timeChange / 1000.0);  // Rate of change

    
    Serial.print("error: ");
    Serial.println(error);

    // Calculate output:
    output = Kp * error + Ki * integral + Kd * derivative;
    Serial.print("OUTPUT: ");
    Serial.println(output);
    output = constrain((0.1 * output), -90, 90);  // Constrain output to servo's range
    Serial.print("OUTPUT CONSTRAINED: ");
    Serial.println(output);

  // Read Left & Right IR Sensors:
  n_leftDistance = mySensorL.distance();
  n_rightDistance = mySensorR.distance();

  // Read Ultrasonic sensor:
  usSensorDistance = readUltrasonicDistance();
  Serial.print("Front Distance: ");
  Serial.print(usSensorDistance);
  Serial.println(" cm");

  
  // Print the measured distance:
  Serial.print("Mean distance LEFT: ");
  Serial.print(n_leftDistance);
  Serial.println(" cm");

  Serial.print("Mean distance RIGHT: ");
  Serial.print(n_rightDistance);
  Serial.println(" cm");

 
  mainLogic(n_leftDistance, n_rightDistance); //main logic to control movement an turns
  
  delay(100);

  // Update variables:
  previous_error = error;
  lastTime = currentTime;

}

long readUltrasonicDistance() {

  // Read ultrasonic sensor:
  digitalWrite(usTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(usTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrigPin, LOW);
  return pulseIn(usEchoPin, HIGH) / 58;

}


void mainLogic(int n_leftDistance, int n_rightDistance) {

  delay(100);

  usSensorDistance = readUltrasonicDistance();

  // Make sharp or sharper turns based on how close US sensor is to wall:
  if (usSensorDistance < FRONT_DISTANCE_TO_CHECK) {
    turnLogic(n_leftDistance,n_rightDistance);
  } 

  else { // For moving hovercraft straight:

    moveForward();
       
  }

}


void turnRight(int angle){

  // Turn servo right if angle is rightward:
  if (angle > THRUST_FAN_DEFAULT_ANGLE) {
    analogWrite(ThrustFanPin, 215);
    servoL.write(angle);
    Serial.print("SERVO angle: ");
    Serial.println(angle);

  }
  
}

void turnLeft(int angle){

  // Turn servo left if angle is leftward:
  if (angle < THRUST_FAN_DEFAULT_ANGLE) {
    analogWrite(ThrustFanPin, 215);
    servoL.write(angle);
    Serial.print("SERVO angle: ");
    Serial.println(angle);
    
  }

}


void moveForward(){

 // If IMU Gyro Z senses sharp changes:
    if (abs(input) > 2) {

      // Correct thrust fan accordingly
      analogWrite(ThrustFanPin, 255);
      Serial.print("SERVO angle: 90 - ");
      Serial.println((0.8 * output));
      servoL.write(90 - (0.8 * output));  // Take 80% of IMU output value
    }
    else
    {
      // Else move straight:
      analogWrite(ThrustFanPin, 255);
      Serial.println("SERVO angle: 90");
      servoL.write(THRUST_FAN_DEFAULT_ANGLE);
    }

}

void turnLogic(int n_leftDistance, int n_rightDistance){

  if ((n_leftDistance < n_rightDistance)) {

    if (usSensorDistance < SHARP_TURN_LIMIT_DISTANCE) {
      turnRight(TURN_RIGHT_SHARP_ANGLE);
            
    } 
    else {
      turnRight(TURN_RIGHT_NORMAL_ANGLE);
            
    }
          

  } 
  else if ((n_rightDistance < n_leftDistance)) {

    if (usSensorDistance < SHARP_TURN_LIMIT_DISTANCE) {
      turnLeft(TURN_LEFT_SHARP_ANGLE);
            
    } 
    else {
      turnLeft(TURN_LEFT_NORMAL_ANGLE);
          

    }
        
  
  } 
  
}
  