#include <Arduino_LSM9DS1.h>
//LEDs
int greenLEDpin = 8;
int redLEDpin1 = 7;
int yellowLEDpin2 = 2;
int blueLEDpin3 = 4;
int whiteLEDpin4 = 13;

//variables for Gyroscope
float gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

float acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;


void setup() {

  if (!IMU.begin()) //Initialize IMU sensor
  { Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //set LEDs as outputs
  pinMode(whiteLEDpin4, OUTPUT);
  pinMode(yellowLEDpin2, OUTPUT);
  pinMode(blueLEDpin3, OUTPUT);
  pinMode(redLEDpin1, OUTPUT);
  pinMode(greenLEDpin , OUTPUT);

  //Read the raw acc and gyro data from the IMU
  for (int cal_int = 0; cal_int < 1000; cal_int ++) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    //Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    //Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    //Delay 3us to have 250Hz for-loop
    delay(3);
  }
  

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  // Start Serial Monitor
  Serial.begin(115200);

  // Init Timer
  loop_timer = micros();

}

void loop() {
  // Get data from:
  IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  IMU.readAcceleration(acc_x, acc_y, acc_z);

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal; 


  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch+= gyro_x * 0.0000611;  
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
  angle_roll += gyro_y * 0.0000611; 
                                     
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);    

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  angle_roll_acc -= 0.0;                                               
 
  if(set_gyro_angles){ 
  
  //If the IMU has been running 
  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle                      
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
    //Correct the drift of the gyro roll angle with the accelerometer roll angle    
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
    //IMU has just started  
    //Set the gyro pitch angle equal to the accelerometer pitch angle                                                           
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle                                       
    angle_roll = angle_roll_acc;
    //Set the IMU started flag                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  //Take 90% of the output roll value and add 10% of the raw roll value 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 

  // Print to Serial Monitor   
  Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);

  // Check Angle for Level LEDs
//
//  if (true){
  
  if (angle_pitch_output < -2.01) {
    // Turn on Level LED
    digitalWrite(redLEDpin1, HIGH);
    digitalWrite(whiteLEDpin4, LOW);
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(blueLEDpin3, LOW);
    digitalWrite(yellowLEDpin2, LOW);
    
    } else if ((angle_pitch_output > -2.00) && (angle_pitch_output < -1.01)) {
    // Turn on Level LED
    digitalWrite(redLEDpin1, LOW);
    digitalWrite(whiteLEDpin4, HIGH);
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(blueLEDpin3, LOW);
    digitalWrite(yellowLEDpin2, LOW);
    
    } else if ((angle_pitch_output < 1.00) && (angle_pitch_output > -1.00)) {
    // Turn on Level LED
    digitalWrite(redLEDpin1, LOW);
    digitalWrite(whiteLEDpin4, LOW);
    digitalWrite(greenLEDpin, HIGH);
    digitalWrite(blueLEDpin3, LOW);
    digitalWrite(yellowLEDpin2, LOW);
    
    } else if ((angle_pitch_output > 1.01) && (angle_pitch_output < 2.00)) {
    // Turn on Level LED
    digitalWrite(redLEDpin1, LOW);
    digitalWrite(whiteLEDpin4, LOW);
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(blueLEDpin3, HIGH);
    digitalWrite(yellowLEDpin2, LOW);
    
    } else if (angle_pitch_output > 2.01) {
    // Turn on Level LED
    digitalWrite(redLEDpin1, LOW);
    digitalWrite(whiteLEDpin4, LOW);
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(blueLEDpin3, LOW);
    digitalWrite(yellowLEDpin2, HIGH);

}
}
