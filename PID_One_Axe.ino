  #include <Arduino_LSM9DS1.h>
  #include <Servo.h>
  
  
  Servo right_prop;
  Servo left_prop;
 
  
  //variables for Gyroscope
  float gyro_x, gyro_y, gyro_z;
  float gyro_x_cal, gyro_y_cal, gyro_z_cal;
  boolean set_gyro_angles;
  
  float acc_x, acc_y, acc_z, acc_total_vector;
  float angle_roll_acc, angle_pitch_acc;
  
  float angle_pitch, angle_roll;
  float angle_pitch_buffer, angle_roll_buffer;
  float angle_pitch_output, angle_roll_output;
  
  float elapsedTime;
  float times;
  float timePrev;
 
  
  
  float PID, pwmLeft, pwmRight, errors, previous_error;
  float pid_p=0;
  float pid_i=0;
  float pid_d=0;
  /////////////////PID CONSTANTS/////////////////
  float kp=1.0;//3.55
  float ki=0.00;//0.003
  float kd=0.00;//2.05
  ///////////////////////////////////////////////
  
  float throttle=1300.0; //initial value of throttle to the motors
  float desired_angle = 0.000; //This is the angle in which we whant the
                           //balance to stay steady
  
  
  void setup() {
  
    if (!IMU.begin()) //Initialize IMU sensor
    { Serial.println("Failed to initialize IMU!");
      while (1);
    }
  
      
    right_prop.attach(3); //attatch the right motor to pin 3
    left_prop.attach(5);  //attatch the left motor to pin 5
  
    times = millis(); //Start counting time in milliseconds
    /*In order to start up the ESCs we have to send a min value
     * of PWM to them before connecting the battery. Otherwise
     * the ESCs won't start up or enter in the configure mode.
     * The min value is 1000us and max is 2000us*/
    left_prop.writeMicroseconds(1000); 
    right_prop.writeMicroseconds(1000);
    delay(7000); /*Give some delay, 7s, to have time to connect
                  *the propellers and let everything start up*/ 
  
  
    //Read the raw acc and gyro data from the IMU
    for (int cal_int = 0; cal_int < 1000; cal_int ++) {
     IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
     //Add the gyro x offset to the gyro_x_cal variable
     gyro_x_cal += gyro_x;
     //Add the gyro y offset to the gyro_y_cal variable
     gyro_y_cal += gyro_y;
     //Add the gyro z offset to the gyro_z_cal variable
     gyro_z_cal += gyro_z;
     
      
   }
    
  
    // Divide all results by 1000 to get average offset
    gyro_x_cal /= 1000.0;
    gyro_y_cal /= 1000.0;
    gyro_z_cal /= 1000.0;
  
    // Start Serial Monitor
    Serial.begin(115200);
    //Serial.begin(250000);
  
  
  }
  
  void loop() {
    // Get data from:
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    IMU.readAcceleration(acc_x, acc_y, acc_z);
  
    timePrev = times;  // the previous time is stored before the actual time read
    times = millis();  // actual time read
    elapsedTime = (times - timePrev) / 1000.0; 
      
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
    angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296; 
    //Calculate the roll angle      
    angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;       
    
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
    Serial.print(" | Angle  = "); 
    Serial.println(angle_roll_output);
    Serial.println(angle_pitch_output);
    
  
  
  
  /*///////////////////////////P I D///////////////////////////////////*/
  /*For this part I will write a pid control to balance one axis. I've choose the x angle
  to implement the PID with. That means that the x axis of the IMU has to be paralel to
  the balance*/
  
  /*First calculate the errors between the desired angle and 
  *the real measured angle*/
    errors = angle_roll_output - desired_angle;
  
  
  /*Next the proportional value of the PID is just a proportional constant
  *multiplied by the error*/
  
   pid_p = kp*errors;
  
  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  if(-3 <errors <3)
  {
    pid_i = pid_i+(ki*errors);  
  }
  
  /*The last part is the derivate. The derivate acts upon the speed of the error.
  As we know the speed is the amount of error that produced in a certain amount of
  time divided by that time. For taht we will use a variable called previous_error.
  We substract that value from the actual error and divide all by the elapsed time. 
  Finnaly we multiply the result by the derivate constant*/
  
  pid_d = kd*((errors - previous_error)/elapsedTime);
  
  /*The final PID values is the sum of each of this 3 parts*/
  PID = pid_p + pid_i + pid_d;
  
  /*We know taht the min value of Pulse-width modulation signal is 1000us and the max is 2000. So that
  tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
  have a value of 2000us the maximum value taht we could sybstract is 1000 and when
  we have a value of 1000us for the Pulse-width modulation sihnal, the maximum value that we could add is 1000
  to reach the maximum 2000us*/
  if(PID < -1000.0)
  {
    PID=-1000.0;
  }
  if(PID > 1000.0)
  {
    PID=1000.0;
  }
  
  /*Finnaly we calculate the Pulse-width modulation width. We sum the desired throttle and the PID value*/
  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;
  
  
  /* We map the Pulse-width modulation values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for 
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/
  //Right
  if(pwmRight < 1000.0)
  {
    pwmRight= 1000.0;
  }
  if(pwmRight > 2000.0)
  {
    pwmRight=2000.0;
  }
  //Left
  if(pwmLeft < 1000.0)
  {
    pwmLeft= 1000.0;
  }
  if(pwmLeft > 2000.0)
  {
    pwmLeft=2000.0;
  }
  
  /*Finnaly using the servo function we create the Pulse-width modulation pulses with the calculated
  width for each pulse*/
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
  previous_error = errors; // store the previous error.
  

  }
