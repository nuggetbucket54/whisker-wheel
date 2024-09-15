/*
  SparkFun Inventor’s Kit
  Circuit 5C - Autonomous Robot

  This robot will drive around on its own and react to obstacles by backing up and turning to a new direction.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v41
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/

#include "BluetoothSerial.h"
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Adafruit_MPU6050.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Adafruit_MPU6050 mpu;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

BluetoothSerial SerialBT;
unsigned long previousMillis = 0;  // Store the last time data was sent
const long interval = 1000;  // Interval to send data (1 second)
// network creds
const char* ssid = "Meow?";
const char* password = "meow!meow!uwu";

//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 26; // 13 on Arduino           //control pin 1 on the motor driver for the right motor
const int AIN2 = 25; // 12 on Arduino           //control pin 2 on the motor driver for the right motor
const int PWMA = 33; // 11 on Arduino           //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 12; // 10 on Arduino           //speed control pin on the motor driver for the left motor
const int BIN2 = 14; // 9 on Arduino           //control pin 2 on the motor driver for the left motor
const int BIN1 = 27; // 8 on Arduino           //control pin 1 on the motor driver for the left motor


//distance variables
const int trigPin = 35; // 6 on Arduino
const int echoPin = 32; // 5 on Arduino

int switchPin = 13; // 7 on Arduino             //switch to turn the robot on and off

float distance = 0;            //variable to store the distance measured by the distance sensor

//robot behaviour variables
int backupTime = 300;           //amount of time that the robot will back up when it senses an object
int turnTime = 200;             //amount that the robot will turn once it has backed up

/********************************************************************************/
void setup()
{
    pinMode(trigPin, OUTPUT);       //this pin will send ultrasonic pulses out from the distance sensor
    pinMode(echoPin, INPUT);        //this pin will sense when the pulses reflect back to the distance sensor

    pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

    //set the motor control pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    Serial.begin(115200);   
    Wire.begin();
  
    while (!mpu.begin()) {
        delay(500);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    /* Set kalman and gyro starting angle */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();

    //begin serial communication with the computer
    Serial.print("To infinity and beyond!");  //test the serial connection
    SerialBT.begin("ESP32_BT");  // Name the device as "ESP32_BT"
    Serial.println("Bluetooth Started! Pair your device.");
}


void loop() {
    /* Update all the values */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

    // Get the current time
    unsigned long currentMillis = millis();

    // Check if 1 second has passed since last transmission
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;  // Save the current time

        // Automatically generate and send data
        String data = "Counter value: " + String(millis() / 1000) + " seconds\n";

        String angles = "Roll: " + String(kalAngleX) + ", Pitch: " + String(kalAngleY);

        SerialBT.print(angles);  // Send the data over Bluetooth
    }

  // Other non-blocking tasks can go here
  //DETECT THE DISTANCE READ BY THE DISTANCE SENSOR
  distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" in");              // print the units

  if (digitalRead(switchPin) == LOW) { //if the on switch is flipped

    if (distance < 10) {              //if an object is detected
      //back up and turn
      Serial.print(" ");
      Serial.print("BACK!");

      //stop for a moment
      rightMotor(0);
      leftMotor(0);
      delay(200);

      //back up
      rightMotor(-255);
      leftMotor(-255);
      delay(backupTime);

      //turn away from obstacle
      rightMotor(255);
      leftMotor(-255);
      delay(turnTime);

    } else {                        //if no obstacle is detected drive forward
      Serial.print(" ");
      Serial.print("Moving...");


      rightMotor(255);
      leftMotor(255);
    }
  } else {                        //if the switch is off then stop

    //stop the motors
    rightMotor(0);
    leftMotor(0);
  }

  delay(50);                      //wait 50 milliseconds between readings
}

/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calculatedDistance;              //send back the distance that was calculated
}
