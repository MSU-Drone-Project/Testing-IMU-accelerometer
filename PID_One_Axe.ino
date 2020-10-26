#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <Servo.h>

 Servo right_prop;
 Servo left_prop;


float times;
float timePrev;
float DT = 0.02;        //loop period. i use a loop of 30ms. so DT = 0.03.
float G_GAIN = 0.07;   // sensitivity level of 2000dps
float gyroXangle, gyroYangle, gyroZangle, Acceleration_angle_x, Acceleration_angle_y;
float Total_angle_x, Total_angle_y;

float PID, pwmLeft, pwmRight, errors, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
/////////////////PID CONSTANTS/////////////////
 double kp=1.20;//3.55
 double ki=0.003;//0.003
 double kd=0.70;//2.05
///////////////////////////////////////////////
double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0.0; //This is the angle in which we whant the
                         //balance to stay steady

// initialize a Madgwick filter:
Madgwick filter;
// sensor's sample rate is fixed at 119 Hz:
const float sensorRate = 119.00;

void setup() {
Serial.begin(250000);
// attempt to start the IMU:
if (!IMU.begin()) {
Serial.println("Failed to initialize IMU");
// stop here if you can't access the IMU:
while (true);
}
// start the filter to run at the sample rate:
filter.begin(sensorRate);


  right_prop.attach(5); //attatch the right motor to pin 3
  left_prop.attach(3);  //attatch the left motor to pin 5

  times = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
   
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  
  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}
void loop() {
// values for acceleration and rotation:
float xAcc, yAcc, zAcc;
float xGyro, yGyro, zGyro;

// values for orientation:
float roll, pitch, heading;
// check if the IMU is ready to read:
if (IMU.accelerationAvailable() &&
IMU.gyroscopeAvailable()) {
// read accelerometer &and gyrometer:
IMU.readAcceleration(xAcc, yAcc, zAcc);
IMU.readGyroscope(xGyro, yGyro, zGyro);

// update the filter, which computes orientation:
filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);

// print the heading, pitch and roll
roll = filter.getRoll();
pitch = filter.getPitch();
heading = filter.getYaw();
Serial.print("Orientation: ");
Serial.print(heading);
Serial.print(" ");
Serial.print(pitch);
Serial.print(" ");
Serial.println(roll);
}

/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
errors = roll - desired_angle;

/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*errors;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3< errors < 3)
{
  pid_i +=(ki*errors);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For that we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_d = kd*((errors - previous_error)/DT);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeft = throttle + PID;
pwmRight = throttle - PID;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = errors; //Remember to store the previous error.


}
