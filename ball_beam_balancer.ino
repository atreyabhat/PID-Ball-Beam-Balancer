
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.







#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 11;         



//Servo Pin
 
                                                  //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  
 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-55,45);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
 
  Setpoint = 7;
  Input = readPosition();                                            
 
   double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  
 ServoOutput=98+Output;                                            // 102 degrees is my horizontal 
  myServo.write(ServoOutput);   
  
 

  

 
}
      
      
      

long readPosition() {
  delay(40);                                                            //Don't set too low or echos will run into eachother.      
  
  
const int pingPin = 9;
const int pingPin2 = 10;
;

long duration, cm;

int x,error;
unsigned long now = millis();
pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin, LOW);


pinMode(pingPin2, INPUT);
duration = pulseIn(pingPin2, HIGH);
  
  cm = duration/29/2;
  x=cm;
  
  
  if(cm > 31)     // 30 cm is the maximum position for the ball
  {cm=30;}


if(x>1 && x<31)
{
Serial.print(x-7);
  Serial.println(x-7);
  delay(50);
}

  return cm;       
  
  //Returns distance value.
}





