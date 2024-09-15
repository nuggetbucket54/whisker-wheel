## Inspiration
Cats can be a toss-up... Some people love them, while some people hate them. But just like any other pet, cats need and deserve the opportunity to release their energy. Some owners may play with their own cats with different toys, while others may spend agonizing hours training their cats to only scratch some arbitrary pole within their homes. We realized that there could be an interactive avenue for cats to expend their energy at home that could come naturally to our kitten friends' instincts.

## What it does

The Whisker Wheel is a quick, fun robot that constantly runs about with fluttering whiskers. Its speed, quick change of motion, as well as flashy whiskers are sure to activate the neurons of any feline friend. With its two DC motors, ultrasonic sensors, and IMU, the Whisker Wheel is capable of extremely unpredictable movement all while avoiding imminent obstacles. All these sensors communicate with an ESP32, which can be connected via Bluetooth on any device to gain insights as to your pet's activity.

## How it's built

The Whisker Wheel contains the HC-SR04 Ultrasonic Sensor, a motor driver, two 48:1 DC motors, MPU6050 IMU, and an ESP32, all connected with many many jumper cables and two mini breadboards. On the software side, we were able to write all the firmware through Arduino using PlatformIO. Though a simple-looking bot, the firmware involved was actually quite extensive, with complex operations like Kalman filters performed on the accelerometer and gyroscope readings of the MPU6050 in order to extract meaningful data on our cat bots.
