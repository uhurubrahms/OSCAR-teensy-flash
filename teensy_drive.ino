#include <TeensyThreads.h>

#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>      
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>


ros::NodeHandle  nh;


boolean flagStop = false;     // These values were cacluated for the specific Teensy microcontroller using an oscilloscope. 
int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 13108;   //  20% duty cycle - corresponds to max forward velocity, extreme right steering

std_msgs::Int32 str_msg;          // create a ROS Publisher called "chatter" of type str_msg
ros::Publisher chatter("chatter", &str_msg);

std_msgs::Int32 rpm_msg; // create a publisher called "rpm" that sends rpm_msg messages.
ros::Publisher rpm_pub("rpm", &rpm_msg);


int kill_pin = 2;     // This is the GPIO pin for emergency stopping.
unsigned long duration = 0;


/* Process values from hall sensor and calculate RPM */
// Currently A8, A9 soldered.
int rpmSensorPin = A9;//
int rpmRefSig = 1; // When connected to analog pin, the signal is 0 when the gear meets(?) the magnet.
//when you set analogReadResolution to 8 bits -> it gives 1 
volatile int rpmStateCur = 0; //the digital value of the incoming analog signals
volatile int rpmStatePrev = 0;
volatile int rpmCnt = 0; // Increment each time when sensor hits the magnet.
int rpmVal = 0;
unsigned long lastmillis = 0;
unsigned long oneSecInterval = 1000;
int t, cur_t; //time variables
const float tireDiameter = 10.95; //traxxas slash 4x4 tire diameter
const float Pi = 3.14159;
float linearVelocity = 0.0; // m/s


void messageDrive( const race::drive_values& pwm ) 
{
//  Serial.print("Pwm drive : ");
//  Serial.println(pwm.pwm_drive);
//  Serial.print("Pwm angle : ");
//  Serial.println(pwm.pwm_angle);
  
  if(flagStop == false)
  {
    str_msg.data = pwm.pwm_drive;
    chatter.publish( &str_msg );

    rpm_msg.data = rpmVal;
    rpm_pub.publish( &rpm_msg );

    if(pwm.pwm_drive < pwm_lowerlimit)  // Pin 5 is connected to the ESC..dive motor
    {
      analogWrite(5,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_drive > pwm_upperlimit)
    {
      analogWrite(5,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(5,pwm.pwm_drive);     //  Incoming data                    
    }

    
    if(pwm.pwm_angle < pwm_lowerlimit) // Pin 6 is connected to the steering servo.
    {
      analogWrite(6,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_angle > pwm_upperlimit)
    {
      analogWrite(6,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(6,pwm.pwm_angle);     //  Incoming data                    
    }

  }
  else
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag )
{
  flagStop = flag.data;
  if(flagStop == true)
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}


ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive );   // Subscribe to drive_pwm topic sent by Jetson
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );  // Subscribe to estop topic sent by Jetson



int analogToInterrupt(int rpmSensorPin){ 
   // rpmStatePrev = rpmStateCur;
   Serial.println("!");
   int rpmSig = analogRead(rpmSensorPin);
   if (rpmSig <= rpmRefSig) rpmStateCur = HIGH; //convert it to digital 0,1 form
   else rpmStateCur = LOW;
   lastmillis = millis();//?
   return rpmStateCur;
}

void ISR() {
   Serial.println("counting up");
   rpmCnt++;
}


//Using TeensyThread
void rpmThread(){
  while(1){
   rpmStatePrev = rpmStateCur;
   
   int rpmSig = analogRead(rpmSensorPin);
   //Serial.println(rpmSig);
   
   if (rpmSig <= rpmRefSig) rpmStateCur = HIGH; //convert it to digital 0,1 form
   else rpmStateCur = LOW;
   

   if(rpmStatePrev == 0 && rpmStateCur == 1){
     ISR();   
   }

   threads.yield();
  }  
}


void setup() {
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(5, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16); // Resolution for the PWM signal
  analogReadResolution(8);
  analogWrite(5,pwm_center_value); // Setup zero velocity and steering.
  analogWrite(6,pwm_center_value);
  pinMode(13,OUTPUT); // Teensy's onboard LED pin. 
  digitalWrite(13,HIGH); // Setup LED.
  pinMode(kill_pin,INPUT); // Set emergency pin to accept inputs.
//  digitalWrite(2,LOW);

  int rpmSensorPin = A9; // sensor connected to A9 pin
  pinMode(rpmSensorPin, INPUT);
  //attachInterrupt(analogToInterrupt(rpmSensorPin), ISR, CHANGE);  

  threads.addThread(rpmThread);

  
  Serial.begin(9600);
 
  nh.initNode();  // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_stop);

  nh.advertise(chatter);  // start the publisher..can be used for debugging.
  nh.advertise(rpm_pub);

  


}

void loop() {
  nh.spinOnce();
  duration = pulseIn(kill_pin, HIGH, 30000);  // continuously monitor the kill pin.
  while(duration > 1900) // stop if kill pin activated..setup everything to zero. 
  {
    duration = pulseIn(kill_pin, HIGH, 30000);
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);        
  }
  // put your main code here, to run repeatedly:


  // *******************************************************
  if (millis() - lastmillis >= oneSecInterval){//per second
    
    Serial.print("RPM count: ");
    Serial.println(rpmCnt);

    rpmVal = rpmCnt * 60; //rpmCnt = rps (revolution per second)
    linearVelocity = Pi * tireDiameter * rpmCnt;

    lastmillis = millis();
    rpmCnt = 0; //flush every one sec  

    
    Serial.print("RPM: ");
    Serial.println(rpmVal);
  }

  // *******************************************************
}

