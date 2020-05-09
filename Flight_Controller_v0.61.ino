//-----------------------------------------Flight Controller Development Version-----------------------------------------------------------------------------------------------------------------------

#include <PinChangeInt.h>                                           // Allows pin interrupts to be attached to arduino pins with no built in interrupts
#include <Servo.h>                                                  // Allows ESC signals to be outputted at a 50Hz frequency 
  
#define ROLL_IN_PIN 8                                               // CH1 Roll Input Pin Allocation (Grey wire)
#define PITCH_IN_PIN 4                                              // CH2 Pitch Input Pin Allocation (Purple wire)
#define THROTTLE_IN_PIN 10                                          // CH3 Throttle Input Pin Allocation (Blue wire)
#define YAW_IN_PIN 11                                               // CH4 Yaw Input Pin Allocation (Green wire)
#define SWD_IN_PIN 12                                               // CH5 2-Position Switch Input Pin Allocation (Orange Wire)
#define SWC_IN_PIN 13                                               // CH6 3-Position Switch Input Pin Allocation (Yellow Wire)

#define ESC1_OUT_PIN 3                                              // Output pin number for ESC 1 (Front Left)
#define ESC2_OUT_PIN 5                                              // Output pin number for ESC 2 (Front Right)
#define ESC3_OUT_PIN 6                                              // Output pin number for ESC 3 (Rear Left) 
#define ESC4_OUT_PIN 9                                              // Output pin number for ESC 4 (Rear Right)

#define RX_AUTHORITY 0.15                                           // Authority limiter for Rx value. All Authority limits must sum to 1 (100%)
#define RY_AUTHORITY 0.15                                           // Authority limiter for Ry value. All Authority limits must sum to 1 (100%)
#define LY_AUTHORITY 0.55                                           // Authority limiter for Ly value. All Authority limits must sum to 1 (100%)
#define LX_AUTHORITY 0.15                                           // Authority limiter for Lx value. All Authority limits must sum to 1 (100%)

#define ROLL_FLAG 1                                                 // Define the bit word allocation for a detected change in Rx
#define PITCH_FLAG 2                                                // Define the bit word allocation for a detected change in Ry
#define THROTTLE_FLAG 4                                             // Define the bit word allocation for a detected change in Ly
#define YAW_FLAG 8                                                  // Define the bit word allocation for a detected change in Lx
#define SWD_FLAG 16                                                 // Define the bit word allocation for a detected change in SWD
#define SWC_FLAG 32 

#define KILL_SWITCH_MONITORED_INPUT unRollIn                        // Defines the variable for the program to monitor for no changes
#define KILL_ENABLED 1                                              // Enable kill switch [ 1 = On | 0 = Off ] 

Servo ESC_1, ESC_2, ESC_3, ESC_4;                                   // Output object at 50Hz to corresponding ESC

int MINIMUM_PW = 1000;                                              // The minimum pulse width that will be written to the ESCs
int MAXIMUM_PW = 2000;                                              // The maximum pulse width that will be written to the ESCs

int SWC, SWD, Rx, Ry, Ly, Lx;                                       // Renamed transmitter values to be used in the FC
int ESCRx, ESCRy, ESCLy, ESCLx;                                     // Specified axis contributions to ESC outputs
int ESC1, ESC2, ESC3, ESC4;                                         // Output value for ESCs
int ESCOff;                                                         // Outputs a value of 0 when written to ESCs

int ESCRxRange, ESCRyRange, ESCLyRange, ESCLxRange;                 // Control stick input ranges, calculated intially as a default estimated value

int ESCRxMidpoint, ESCRyMidpoint, ESCLyMidpoint, ESCLxMidpoint;     // Control stick inputs centre points, calculated intially as a default estimated value

int unRollInMidpoint = 1470;                                        // Value for storing calibrated Roll midpoint, initialised as a default estimate
int unPitchInMidpoint = 1440;                                       // Value for storing calibrated Pitch midpoint, initialised as a default estimate
int unThrottleInMidpoint = 1464;                                    // Value for storing calibrated Throttle midpoint, initialised as a default estimate
int unYawInMidpoint = 1476;                                         // Value for storing calibrated Yaw midpoint, initialised as a default estimate

int unRollInMax = 1948;                                             // Value for storing calibrated Roll maximum point, initialised as a default estimate                                       
int unRollInMin = 984;                                              // Value for storing calibrated Roll minimum point, initialised as a default estimate
int unPitchInMax = 1916;                                            // Value for storing calibrated Pitch maximum point, initialised as a default estimate        
int unPitchInMin = 964;                                             // Value for storing calibrated Pitch minimum point, initialised as a default estimate
int unThrottleInMax = 1956;                                         // Value for storing calibrated Throttle maximum point, initialised as a default estimate
int unThrottleInMin = 968;                                          // Value for storing calibrated Throttle minimum point, initialised as a default estimate  
int unYawInMax = 1964;                                              // Value for storing calibrated Yaw maximum point, initialised as a default estimate
int unYawInMin = 984;                                               // Value for storing calibrated Yaw minimum point, initialised as a default estimate

int unRollInMaxLimit = 1800, unPitchInMaxLimit = 1800, unThrottleInMaxLimit = 1800, unYawInMaxLimit = 1800;   // Value used in calibration to trigger starting to detect the maximum control stick values
int unRollInLowLimit = 1200, unPitchInLowLimit = 1200, unThrottleInLowLimit = 1200, unYawInLowLimit = 1200;   // Value used in calibration to trigger starting to detect the minimum control stick values

int Kill_Switch_Monitor;                                            // Creates a parameter to contain a value for the previous cycles monitored input
int Kill_Switch_Count;                                              // Counts the amount of cycles the monitored input does not change consecutively from last loop

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unRollInShared, unPitchInShared, unThrottleInShared, unYawInShared, unSwitchDInShared, unSwitchCInShared;
uint32_t ulRollStart, ulPitchStart, ulThrottleStart, ulYawStart, ulSwitchDStart, ulSwitchCStart;



void setup()
{
  Serial.begin(9600);

  ESC_1.attach(ESC1_OUT_PIN , MINIMUM_PW , MAXIMUM_PW);
  ESC_2.attach(ESC2_OUT_PIN , MINIMUM_PW , MAXIMUM_PW);
  ESC_3.attach(ESC3_OUT_PIN , MINIMUM_PW , MAXIMUM_PW);
  ESC_4.attach(ESC4_OUT_PIN , MINIMUM_PW , MAXIMUM_PW);

  PCintPort::attachInterrupt(ROLL_IN_PIN , calcRoll , CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN , calcPitch , CHANGE);
  PCintPort::attachInterrupt(THROTTLE_IN_PIN , calcThrottle , CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN , calcYaw , CHANGE);
  PCintPort::attachInterrupt(SWD_IN_PIN , calcSWD , CHANGE);
  PCintPort::attachInterrupt(SWC_IN_PIN , calcSWC , CHANGE);
}



void loop()
{
  static uint16_t unRollIn;
  static uint16_t unPitchIn;
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unSwitchDIn;
  static uint16_t unSwitchCIn;
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }
   
    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }

    if(bUpdateFlags & SWD_FLAG)
    {
      unSwitchDIn = unSwitchDInShared;
    }

    if(bUpdateFlags & SWC_FLAG)
    {
      unSwitchCIn = unSwitchCInShared;
    }

  bUpdateFlagsShared = 0;
  
  interrupts();
  }
    
    if(Kill_Switch_Monitor == KILL_SWITCH_MONITORED_INPUT)
    {
      Kill_Switch_Count = Kill_Switch_Count + KILL_ENABLED;
      if(Kill_Switch_Count > 70)
      {
        Kill_Switch_Count = 70;
      }
    }
    else
    {
      Kill_Switch_Count = 0;
    }

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------Flight Controller (Proportional)-----------------------------------------------------------------------------------------------------


Rx = unRollIn;
Ry = unPitchIn;
Ly = unThrottleIn;
Lx = unYawIn;

if(Kill_Switch_Count > 60)
{
ESC_1.write(ESCOff); 
ESC_2.write(ESCOff);
ESC_3.write(ESCOff);
ESC_4.write(ESCOff);
}
else if(unSwitchDIn > 1500 && unSwitchCIn <= 1250)
{
Calibration();  
}
else if(unSwitchCIn <= 1250)
{
ESC_1.write(ESCOff); 
ESC_2.write(ESCOff);
ESC_3.write(ESCOff);
ESC_4.write(ESCOff);
}
else if(unSwitchCIn > 1250 && unSwitchCIn <= 1750)
{
ESCRxRange = map(Rx , unRollInMin , unRollInMax , -90 , 90);
ESCRyRange = map(Ry , unPitchInMin , unPitchInMax , -90 , 90);
ESCLyRange = map(Ly , unThrottleInMin , unThrottleInMax , -90 ,90);
ESCLxRange = map(Lx , unYawInMin , unYawInMax , -90 , 90);

ESCRxMidpoint = map(unRollInMidpoint , unRollInMin , unRollInMax , -90 , 90);
ESCRyMidpoint = map(unPitchInMidpoint , unPitchInMin , unPitchInMax , -90 , 90);
ESCLyMidpoint = map(unThrottleInMidpoint , unThrottleInMin , unThrottleInMax , -90 , 90);
ESCLxMidpoint = map(unYawInMidpoint , unYawInMin , unYawInMax , -90 , 90);

ESCRx = (ESCRxMidpoint + ESCRxRange) * RX_AUTHORITY;
ESCRy = (ESCRyMidpoint + ESCRyRange) * RY_AUTHORITY;
ESCLy = (ESCLyMidpoint + ESCLyRange) * LY_AUTHORITY;
ESCLx = (ESCLxMidpoint + ESCLxRange) * LX_AUTHORITY;

ESC1 = (ESCRx + (0-ESCRy) + ESCLy + (0-ESCLx)) + 90;
ESC_1.write(ESC1);

ESC2 = ((0-ESCRx) + (0-ESCRy) + ESCLy + ESCLx) + 90;
ESC_2.write(ESC2);

ESC3 = (ESCRx + ESCRy + ESCLy + ESCLx) + 90;
ESC_3.write(ESC3);

ESC4 = ((0-ESCRx) + ESCRy + ESCLy + (0-ESCLx)) + 90;
ESC_4.write(ESC4);
}
else if(unSwitchCIn > 1750)
{
ESC_1.write(ESCOff); 
ESC_2.write(ESCOff);
ESC_3.write(ESCOff);
ESC_4.write(ESCOff);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

  Serial.print(ESC1);
  Serial.print("\t");
  Serial.print(ESC2);
  Serial.print("\t");
  Serial.print(ESC3);
  Serial.print("\t");
  Serial.print(ESC4);
  Serial.print("\t");
  Serial.println(Kill_Switch_Count);
  
Kill_Switch_Monitor = KILL_SWITCH_MONITORED_INPUT;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------


//---------------------------------Calibration Sequence-------------------------------------------------------------------------------
void Calibration()
{
  
//Midpoints 
unRollInMidpoint = unRollInShared;
unPitchInMidpoint = unPitchInShared;
unThrottleInMidpoint = unThrottleInShared;
unYawInMidpoint = unYawInShared;

while(unSwitchDInShared > 1500)
  {
  static uint16_t unRollIn;
  static uint16_t unPitchIn;
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unSwitchDIn;
  static uint16_t unSwitchCIn;
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }
   
    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }

        if(bUpdateFlags & SWD_FLAG)
    {
      unSwitchDIn = unSwitchDInShared;
    }
  bUpdateFlagsShared = 0;
  interrupts();
  }

  
  //RollMax
  if(unRollIn > unRollInMaxLimit) {unRollInMaxLimit = unRollIn;}
  if(unRollIn < unRollInMaxLimit) {unRollInMaxLimit = unRollInMaxLimit;}

  //RollMin
  if(unRollIn < unRollInLowLimit && unRollIn > 500) {unRollInLowLimit = unRollIn;}
  if(unRollIn > unRollInLowLimit) {unRollInLowLimit = unRollInLowLimit;}
  
  //PitchMax
  if(unPitchIn > unPitchInMaxLimit) {unPitchInMaxLimit = unPitchIn;}
  if(unPitchIn < unPitchInMaxLimit) {unPitchInMaxLimit = unPitchInMaxLimit;}

  //PitchMin
  if(unPitchIn < unPitchInLowLimit && unPitchIn > 500) {unPitchInLowLimit = unPitchIn;}
  if(unPitchIn > unPitchInLowLimit) {unPitchInLowLimit = unPitchInLowLimit;}

  //ThrottleMax
  if(unThrottleIn > unThrottleInMaxLimit) {unThrottleInMaxLimit = unThrottleIn;}
  if(unThrottleIn < unThrottleInMaxLimit) {unThrottleInMaxLimit = unThrottleInMaxLimit;}

  //ThrottleMin
  if(unThrottleIn < unThrottleInLowLimit && unThrottleIn > 500) {unThrottleInLowLimit = unThrottleIn;}
  if(unThrottleIn > unThrottleInLowLimit) {unThrottleInLowLimit = unThrottleInLowLimit;}
  
  //YawMax
  if(unYawIn > unYawInMaxLimit) {unYawInMaxLimit = unYawIn;}
  if(unYawIn < unYawInMaxLimit) {unYawInMaxLimit = unYawInMaxLimit;}

  //YawMin
  if(unYawIn < unYawInLowLimit && unYawIn > 500) {unYawInLowLimit = unYawIn;}
  if(unYawIn > unYawInLowLimit) {unYawInLowLimit = unYawInLowLimit;}

  Serial.print(unRollInMaxLimit);
  Serial.print("\t");
  Serial.print(unRollInLowLimit);
  Serial.print("\t");
  Serial.print(unPitchInMaxLimit);
  Serial.print("\t");
  Serial.print(unPitchInLowLimit);
  Serial.print("\t");
  Serial.print(unThrottleInMaxLimit);
  Serial.print("\t");
  Serial.print(unThrottleInLowLimit);
  Serial.print("\t");
  Serial.print(unPitchInMaxLimit);
  Serial.print("\t");
  Serial.println(unPitchInLowLimit);
  }

unRollInMax = unRollInMaxLimit;
unRollInMin = unRollInLowLimit;
unPitchInMax = unPitchInMaxLimit;
unPitchInMin = unPitchInLowLimit;
unThrottleInMax = unThrottleInMaxLimit;
unThrottleInMin = unThrottleInLowLimit;
unYawInMax = unYawInMaxLimit;
unYawInMin = unYawInLowLimit;

  }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------Interrupt Service Routines-------------------------------------------------------------------------------

void calcRoll() 
{
  if(digitalRead(ROLL_IN_PIN) == HIGH)
  {
    ulRollStart = micros();
  }
  else
  {
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void calcPitch()
{
  if(digitalRead(PITCH_IN_PIN) == HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void calcThrottle()
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcYaw()
{
  if(digitalRead(YAW_IN_PIN) == HIGH)
  {
    ulYawStart = micros();
  }
  else
  {
    unYawInShared = (uint16_t)(micros() - ulYawStart);
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcSWD()
{
  if(digitalRead(SWD_IN_PIN) == HIGH)
  {
    ulSwitchDStart = micros();
  }
  else
  {
    unSwitchDInShared = (uint16_t)(micros() - ulSwitchDStart);
    bUpdateFlagsShared |= SWD_FLAG;
  }
}

void calcSWC()
{
  if(digitalRead(SWC_IN_PIN) == HIGH)
  {
    ulSwitchCStart = micros();
  }
  else
  {
    unSwitchCInShared = (uint16_t)(micros() - ulSwitchCStart);
    bUpdateFlagsShared |= SWC_FLAG;
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
