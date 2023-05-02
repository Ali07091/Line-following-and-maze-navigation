#include <Wire.h>
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <math.h>  //include math library for use with MPU6050
#include <MPU6050_tockn.h>  //include MPU6050 library
#define I2C_SLAVE_ADDR  0x04 //Define the I2C address of the Arduino Nano




int leftMotor_speed, rightMotor_speed;  //variables to store the motor speeds and servo angle
int servoAngle = 10;
int x = 0;  //variable to store data for transmission

// Define the keypad pins and keys
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {15, 2, 0, 4};
byte colPins[COLS] = {16, 17, 5};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Define variables for storing the key sequence
char keySequence[50];
int sequenceLength = 0;

//Encoder
int leftEncoderCount;
int rightEncoderCount;
int distanceL;
double prevDistanceL = 0;  
int distanceR;
double prevDistanceR = 0;
//Set constants for pi, diameter of wheel (D) and number of encoders counts of revolution (N)
const float pi = 3.14;
const float D = 5.9;
const float N = 25;

LiquidCrystal My_LCD(14, 27, 26, 25, 33, 32);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize OLED display
  
  My_LCD.begin(16, 2);  // Initialize the OLED display
  My_LCD.clear();


  // Display initial message
  My_LCD.setCursor(0, 0);
  My_LCD.println("Entr cmnd and sequence,5 to execute");
   // Update the display with the new content  
  // Initialize I2C communication
  Wire.begin();
}


void loop() {
  
  My_LCD.scrollDisplayLeft();
  My_LCD.display();  // Update the display with the new content 
  delay(500);
  Wire.requestFrom(0x04, 2);  // Request 2 bytes of data from the Arduino Nano
  if (Wire.available() >= 2) {
    leftEncoderCount = Wire.read();
    rightEncoderCount = Wire.read();
  }
  //Serial.print("Left Encoder distance: ");
  //Serial.println(distanceL);
  //Serial.print("Right Encoder distance: ");
 // Serial.println(distanceR);


  distanceL = leftEncoderCount * ((D*pi)/N);
  distanceR = rightEncoderCount * ((D*pi)/N);

 


  // Read the keypad input
  char key = keypad.getKey();

//Debug
  if (key != NO_KEY) {
    // Display the key on the OLED display
    My_LCD.clear();
    My_LCD.setCursor(0, 0);
    My_LCD.println(" Key pressed: " + String(key));
    My_LCD.display();
  if (key) {
    Serial.println(key);
  }
}

    if (key == '2') {
      keySequence[sequenceLength] = 'F';
      sequenceLength++;
     
       My_LCD.setCursor(0, 20);
       My_LCD.println("    forward");
       My_LCD.display(); 
    } else if (key == '4') {
      keySequence[sequenceLength] = 'L';
      sequenceLength++;
      My_LCD.setCursor(0, 20);
      My_LCD.println("    Left");
      My_LCD.display();
    } else if (key == '6') {
      keySequence[sequenceLength] = 'R';
      sequenceLength++;
      My_LCD.setCursor(0, 20);
       My_LCD.println("    Right");
       My_LCD.display();
    } else if (key == '8') {
      keySequence[sequenceLength] = 'B';
      sequenceLength++;
      My_LCD.setCursor(0, 20); 
      My_LCD.println("    backward");
      My_LCD.display();      
    }else if (key == '0') {
      keySequence[sequenceLength] = 'S';
      sequenceLength++;
      My_LCD.setCursor(0, 20);
      My_LCD.println("    Stop");
      My_LCD.display();
    } else if (key == '5') {
      // Execute the key sequence and display a message on the OLED display
      My_LCD.clear();
      My_LCD.setCursor(0, 0);
      My_LCD.println("Executing command");
      My_LCD.display();
      for (int i = 0; i <= sequenceLength; i++) {
        char command = keySequence[i];
        switch (command) {
          case 'F':
            goForward();
           
            break;
            case 'L':
            goLeft();
             
            break;
            case 'R':
            goRight();
            
            break;
            case 'B':
            goBackward();
            
            break;
            case 'S':
            stop();
           
            break;
            }
           }
             // Reset the key sequence
            sequenceLength = 0;
            My_LCD.setCursor(0, 20);
            My_LCD.println("Command executed");
           
        } else if (key == '7') {
            // Clear the key sequence and display a message on the OLED display
            sequenceLength = 0;
            My_LCD.clear();
            My_LCD.setCursor(0, 0);
            My_LCD.println("sequence cleared");
          }
  }

      // Define motor control functions
      void goForward() {
        // Move the robot forward
         delay(1000);
        leftMotor_speed = 150;
        rightMotor_speed = 150;
        servoAngle = 90;
        Serial.println("Going Forward!");
        SendingToSlave();
        }


      void goLeft() {
        // Code to turn the robot left
        delay(1000);
        leftMotor_speed = 100;
        rightMotor_speed = 250;
        servoAngle = -20;
        SendingToSlave();
        Serial.println("Going Left!");
        }



      void goRight() {
        // Code to turn the robot right
        leftMotor_speed = 250;
        rightMotor_speed = 100;
        servoAngle = 130;
        SendingToSlave();
        Serial.println("Going Right!");
        delay(1000);
      }

      void goBackward() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = -150;
        rightMotor_speed = -150;
        servoAngle = 88;
        SendingToSlave();
        Serial.println("Going Backwards!");
      }

      void stop() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = 0;
        rightMotor_speed = 0;
        servoAngle = 88;
        SendingToSlave();
        Serial.println("Stop!");
      }

      void SendingToSlave() {

  Wire.beginTransmission(I2C_SLAVE_ADDR);  // transmit to device #4
  
  Serial.print(leftMotor_speed);

  // sending Left Motor Speed to the Slave
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));  // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));         // second byte of x, containing the 8 LSB - bits 8 to 1

  // sending Right Motor Speed to the Slave
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1

  // sending Servo Angle to the Slave
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1

  Wire.endTransmission();  // stop transmitting
  delay(10);
}


