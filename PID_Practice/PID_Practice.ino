
/* 
INPUTS
Actual positions ===== Gyro axis (unknown ranges)
Demanded Setpoints ===== Jowstick inputs (0 to 2000) need to map to (0 to 180)as we only want attitudes facing up

OUTPUTS 
PID values for all 3 axis, no mapping needed as should not be needed provided gains are tuned correctly and
Quadcopter remains balanced. Each axis output should be assigned the the ESCs in the same way as outputs 
are assigned in the first proportional controller.
*/






#define Kp 1
#define Ki 0.5
#define Kd 2

#define ROLL_IN_PIN 0

//Inputs
int RollSetpoint;
int Roll;

//PID Loop
int LoopCount;
int RollError;
int PreviousRollError;


int RollPError;
float RollIError;
int RollDError;

float RollPID;

//Outputs
int RxP;
int RxI;
int RxD;



void setup()
{
  Serial.begin(9600);
}

void loop()
{
Roll = map(analogRead(ROLL_IN_PIN) , 0 , 1023 , -500 , 500);
RollSetpoint = 0;
RollError = RollSetpoint - Roll;




//Proportional
RollPError = (RollError/4) * Kp;

//Integral
RollIError += (RollError/200) * Ki;

//Derivative
if(LoopCount == 0)
{
  PreviousRollError = RollError;
  LoopCount = 1;
}
else if(LoopCount == 1)
{
  RollDError = 0-(RollError - PreviousRollError) * Kd; 
  LoopCount = 0;
}






//PID Output
RollPID = (RollPError + RollIError + RollDError);





Serial.print(RollPID);
Serial.print("\t");
Serial.println(Roll);


}
