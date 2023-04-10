#include <Encoder.h>
#include <FlexyStepper.h>
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <PID_v1.h>


double Setpoint = 0;
double Input, Output;


int limit = 5000;
int count = 0;

const int stepPin = 3;
const int dirPin = 6;
const int Enable = 5;

const int Heading_Hold = 7;
const int SET = 8;

double SteeringPosition = 0;
double Steering_Rotations = 0;
float lastMicros = 0;

int Mode = 0;
float heading = 0;
float point_bearing =0;
int Hold_Heading = 0;
int Hold_Reset = 0;

int error = 0;
 
int wrap360(int directio) {
 while (directio > 359) directio -= 360;
 while (directio <   0) directio += 360;
 return directio;
}


int heading_error(int bearing, int current_heading){
 int error = current_heading - bearing;
 if (error >  180) error -= 360;
 if (error < -180) error += 360;
 return error;
}

Encoder myEnc(2, 4);
FlexyStepper stepper;
//HMC5883L_Simple Compass;
//PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(Heading_Hold, INPUT_PULLUP);
  pinMode(SET, INPUT_PULLUP);
  pinMode(Enable,OUTPUT);
  digitalWrite(Enable,LOW);
  //Wire.begin();
  //myPID.SetOutputLimits(-1000, 1000);
  stepper.connectToPins(stepPin,dirPin);
  stepper.setSpeedInStepsPerSecond(50000*16);
  stepper.setAccelerationInStepsPerSecondPerSecond(2000*16);
  stepper.setStepsPerRevolution(1600);
  //Compass.SetSamplingMode(COMPASS_CONTINUOUS);
  //Compass.SetDeclination(-0, 16, 'W');
  //Compass.SetScale(COMPASS_SCALE_810);
  //myPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
  //Serial.println("Encoder Test:");
}

long oldPosition  = -999;

void loop() {
if (count > limit){
  count = 0;
  //heading = Compass.GetHeadingDegrees();
  if (Mode = 1 ){
    //myPID.Compute();
  }
//Serial.println(Input);
  
} else {
  count = count + 1;
}





  if (digitalRead(Heading_Hold) == false){
    Mode = 1;
    Hold_Reset = 0;
    if (Hold_Heading == 0){
      //point_bearing = Compass.GetHeadingDegrees();
      Hold_Heading = 1;
    }
    //Input = heading_error(point_bearing, heading);
    //SteeringPosition = map(Input,-180,180,-2000,2000);
  }

  
  if (digitalRead(SET) == false){
    Mode = 2;
    Hold_Heading = 0;
    if (Hold_Reset == 0){
       myEnc.write(0);
       stepper.setCurrentPositionInRevolutions(0);
       Hold_Reset = 1;
    }
  }

  
  if (digitalRead(SET) == true & digitalRead(Heading_Hold) == true){ ///--------------Manual Steering Mode
    Mode = 0;
    Hold_Heading = 0;
    Hold_Reset = 0;                                                            
//###     Steering Input    ###//                                             
  long newPosition = myEnc.read();                                            
  if (newPosition != oldPosition) {                                           
    if (newPosition < 2000) {                                                 
      if (newPosition > -2000) {                                               
        oldPosition = newPosition;                                            
        SteeringPosition = oldPosition;                                       
        //Serial.println(newPosition);                                        
      } else {                                                                
        myEnc.write(oldPosition);                                             
      }                                                                       
    } else {                                                                  
      myEnc.write(oldPosition);                                               
    }                                                                         
  }                                                                           
//### End of Steering Input ###//                                             
  }
 


//### Command Stepper Motor ###//

  Steering_Rotations = map(SteeringPosition,-2000,2000,-800,800);
  Steering_Rotations = Steering_Rotations/100;
  
  stepper.setTargetPositionInRevolutions(Steering_Rotations);
  
  if(!stepper.motionComplete()){ // if stepper motor is not complete its move
      stepper.processMovement(); // Command stepper to move to target
      }
 //## End of Stepper Command ###//
 //Serial.println(Mode);
}
