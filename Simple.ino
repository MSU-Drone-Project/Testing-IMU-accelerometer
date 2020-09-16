

#include <Arduino_LSM9DS1.h>
//#define blueLED 22
//#define redLED 25
//#define greenLED 26
//#define yellowLED 20
//#define whiteLED 1
const int greenLEDpin = 8;
const int redLEDpin1 = 7;
const int yellowLEDpin2 = 2;
const int blueLEDpin3 = 4;
const int whiteLEDpin4 = 13;

void setup() {
  Serial.begin(9600);
  pinMode(whiteLEDpin4, OUTPUT);        //set LEDs as outputs
  pinMode(yellowLEDpin2, OUTPUT);
  pinMode(blueLEDpin3, OUTPUT);
  pinMode(redLEDpin1, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  digitalWrite(redLEDpin1, LOW);
  digitalWrite(greenLEDpin, LOW);    //set initial state as off
  digitalWrite(blueLEDpin3, LOW);
  digitalWrite(yellowLEDpin2, LOW);
  digitalWrite(whiteLEDpin4, LOW);
  
  while (!Serial);
  Serial.println("Started");

 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleation in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float acceleration_x, acceleration_y, acceleration_z, delta = 0.05;

    if (IMU.accelerationAvailable()) 
    {
      IMU.readAcceleration(acceleration_x, acceleration_y, acceleration_z);
  
      Serial.print(acceleration_x);
      Serial.print('\t');
      Serial.print(acceleration_y);
      Serial.print('\t');
      Serial.println(acceleration_z);
    
      if (acceleration_y <= delta && acceleration_y >= -delta){//The drone is hovering
            digitalWrite(greenLEDpin, LOW);
            digitalWrite(redLEDpin1, LOW);
            digitalWrite(blueLEDpin3, LOW);
            digitalWrite(yellowLEDpin2, LOW);
            digitalWrite(whiteLEDpin4, HIGH);
            }
      else if (acceleration_y > delta && acceleration_y < 1-delta){//pitch
            digitalWrite(greenLEDpin, HIGH);
            digitalWrite(redLEDpin1, HIGH);
            digitalWrite(blueLEDpin3, LOW);
            digitalWrite(yellowLEDpin2, LOW);
            digitalWrite(whiteLEDpin4, LOW);
            }
//     else if (acceleration_y >= 1-delta){
//            digitalWrite(greenLEDpin, LOW);
//            digitalWrite(redLEDpin1, HIGH);
//            digitalWrite(blueLEDpin3, HIGH);
//            digitalWrite(yellowLEDpin2, LOW);
//            digitalWrite(whiteLEDpin4, LOW);
//           }
        else if (acceleration_y < -delta && acceleration_y > delta-1){//pitch right 
            digitalWrite(greenLEDpin, LOW);
            digitalWrite(redLEDpin1, LOW);
            digitalWrite(blueLEDpin3, HIGH);
            digitalWrite(yellowLEDpin2, HIGH);
            digitalWrite(whiteLEDpin4, LOW);
             }
          else{
            digitalWrite(greenLEDpin, HIGH);
            digitalWrite(redLEDpin1, HIGH);
            digitalWrite(blueLEDpin3, LOW);
            digitalWrite(yellowLEDpin2, LOW);
            digitalWrite(whiteLEDpin4, LOW);
            
          }
            }
    delay(100);
  }
