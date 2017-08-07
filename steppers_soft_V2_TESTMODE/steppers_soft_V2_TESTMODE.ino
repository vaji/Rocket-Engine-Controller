#include <AccelStepper.h>

#define LED_PIN 13

#define X_STEP 6
#define X_DIR 5
#define X_EN 4
#define X_BASE 11
 
#define Y_STEP 10
#define Y_DIR 9
#define Y_EN 8
#define Y_BASE 12

#define INPUT_A_CLOSE_VALVES 2  //piny do wejvcscia ze stacji, stan wysoki - RESET i flag1
#define INPUT_B_MAINSTAGE 3   // piny do wejścia ze stacji, stan wysoki - mainstage

#define CALIBRATION_STEPS 4000

#define LED_PIN            13

#define FlowRateX 44 //procent otwarcia zaworu F
#define FlowRateY 10//procent otwarcia zaworu OX

#define FlowRateXX 42//procent otwarcia zaworu F po sygnale
#define FlowRateYY 100//procent otwarcia zaworu OX po sygnale

#define X_STEPS_TO_MAX_OPEN 1300
#define Y_STEPS_TO_MAX_OPEN 1650

AccelStepper EngineX(AccelStepper::DRIVER, X_STEP, X_DIR);    //tworzenie klasy silnika 1 oznacza tryb sterownikow step/dir
AccelStepper EngineY(AccelStepper::DRIVER, Y_STEP, Y_DIR);


// ************* DEFINE SEQUENCE HERE *******************
// REMARKS: @ DO NOT put 0 in ListTime.
// @ Time in miliseconds, Pos in %
const float CommandListPosX[] = {0, 0, 17, 65, 65, 0};
const float CommandListTimeX[] = {1, 3000, 3000, 500, 2000, 500};

const float CommandListPosY[] = {0, 0, 35, 39, 65, 65, 0};
const float CommandListTimeY[] = {1, 3500, 1000, 1500, 500, 2000, 500};
// ************* DEFINE SEQUENCE HERE *******************

bool CommandListFlagX[] = {0,0,0,0,0,0,0};
bool CommandAllDoneX = false;
int CommandTotalX = sizeof(CommandListPosX)/sizeof(float);
int CommandCurrentX = 0;
float CommandTimeX = 0;
int CommandTargetX = 0;

bool CommandListFlagY[] = {0,0,0,0,0,0,0};
bool CommandAllDoneY = false;
int CommandTotalY = sizeof(CommandListPosY)/sizeof(float);
int CommandCurrentY = 0;
float CommandTimeY = 0;
int CommandTargetY = 0;


bool CommandLoop = false;
bool emergencyShutDown = false;

bool MODE_SERVICE = false;
bool MODE_TEST = false;

int ENGINE_X_DEFAULT_TIME = 1000;
int ENGINE_Y_DEFAULT_TIME = 1000;

long mapPercentageToSteps(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min + 0.5;
}

bool valvesOpenFlag = false;
bool prestageFlag = false;
bool mainstageFlag = false;

bool xEndstop() {
  if (HIGH == digitalRead(X_BASE)) return true;
  else return false;
}

bool yEndstop() {
  if (HIGH == digitalRead(Y_BASE)) return true;
  else return false;
}

void setXEnginePositionToGo(int percent)
{
  
  long PositionToGoX = mapPercentageToSteps(percent, 0, 100, 0, X_STEPS_TO_MAX_OPEN);
  long stepsToMakeX = abs(PositionToGoX - EngineX.currentPosition());

   EngineX.setMaxSpeed(1500);  
     EngineX.setAcceleration(50000);
  EngineX.moveTo(-PositionToGoX);
}

void setYEnginePositionToGo(int percent)
{
  long PositionToGoY = mapPercentageToSteps(percent, 0, 100, 0, Y_STEPS_TO_MAX_OPEN);
  long stepsToMakeY = abs(PositionToGoY - EngineY.currentPosition());
     EngineY.setMaxSpeed(1500);  
       EngineY.setAcceleration(50000);
  EngineY.moveTo(-PositionToGoY);
}

void Engines_ShutDown() {
  
  EngineX.moveTo(0);
  EngineY.moveTo(0); //krec sie z koncem gdyby endstop nie dziala
  while (EngineX.distanceToGo() != 0 || EngineY.distanceToGo() != 0) {
    
     EngineX.run();
    EngineY.run();
    };
  }
void Engines_StopAndCalibrateAtEndStop() {

  bool x_ready = false;
  bool y_ready = false;
  EngineX.moveTo(CALIBRATION_STEPS);
  EngineY.moveTo(CALIBRATION_STEPS); //krec sie z koncem gdyby endstop nie dziala
  while (EngineX.distanceToGo() != 0 || EngineY.distanceToGo() != 0) {
    Serial.println("CALIBRATE");
    Serial.print("X endstop:");
    Serial.print(xEndstop());
    Serial.print(" Y endstop:");
    Serial.println(yEndstop());
    EngineX.run();
    EngineY.run();

    if (xEndstop()) {
      x_ready = true;
      EngineX.stop();
      EngineX.setCurrentPosition(0);
    }
    if (yEndstop()) {
      EngineY.stop();
      EngineY.setCurrentPosition(Y_STEPS_TO_MAX_OPEN);
    }
    //Serial.print(EngineX.currentPosition());
    //Serial.print("\t");
    //Serial.println(EngineY.currentPosition());

  };
  
  EngineY.moveTo(0);
 
  
  while(EngineY.distanceToGo() != 0 || EngineX.distanceToGo() != 0)
  {
    EngineY.run();
  };
}
void xSetCommand(int command) {
  float percentageX = CommandListPosX[command];
  float xTime = CommandListTimeX[command];
  
  long PositionToGoX = mapPercentageToSteps(percentageX, 0, 100, 0, X_STEPS_TO_MAX_OPEN);
  long stepsToMakeX = abs(PositionToGoX - EngineX.currentPosition());
  CommandTargetX = PositionToGoX;
  xTime /= 1000;
  float xSpeed = stepsToMakeX / xTime;
  EngineX.moveTo(-PositionToGoX);
  EngineX.setMaxSpeed(xSpeed);  
  CommandTimeX = millis(); //record time when command was set 
}

void ySetCommand(int command) {
 float percentageY = CommandListPosY[command];
  float yTime = CommandListTimeY[command];
  
  long PositionToGoY = mapPercentageToSteps(percentageY, 0, 100, 0, Y_STEPS_TO_MAX_OPEN);
  long stepsToMakeY = abs(PositionToGoY - EngineY.currentPosition());
  CommandTargetY = PositionToGoY;
  yTime /= 1000;
  float ySpeed = stepsToMakeY / yTime;
  EngineY.moveTo(-PositionToGoY);
  EngineY.setMaxSpeed(ySpeed);  
  CommandTimeY = millis(); //record time when command was set 
}

void xyRunCurrentCommand() {

  while(!CommandAllDoneX || !CommandAllDoneY)
  {
    
    if(EngineX.distanceToGo() != 0 ||  (millis() - CommandTimeX) <= CommandListTimeX[CommandCurrentX])
    {
     Serial.print("x:  ");
       Serial.print(EngineX.distanceToGo());
       Serial.print("  ");
       Serial.print(EngineX.currentPosition());
       Serial.print("  ");
       Serial.println(CommandTargetX);
      EngineX.run();
    } else
    {
      CommandListFlagX[CommandCurrentX] = 1;
      if(CommandCurrentX < (CommandTotalX - 1)) {
        CommandCurrentX++;    
        xSetCommand(CommandCurrentX);
      } else CommandAllDoneX = true;
    }
    
    if(EngineY.distanceToGo() != 0 ||  (millis() - CommandTimeY) <= CommandListTimeY[CommandCurrentY])
    {
      Serial.print("y:  ");
       Serial.print(EngineY.distanceToGo());
       Serial.print("  ");
       Serial.print(EngineY.currentPosition());
       Serial.print("  ");
       Serial.println(CommandTargetY);
      
      EngineY.run();
    } else
    {
      CommandListFlagY[CommandCurrentY] = 1;
      if(CommandCurrentY < (CommandTotalY - 1)) {
        CommandCurrentY++;    
        ySetCommand(CommandCurrentY);
      } else CommandAllDoneY = true;
    }

  }
}



void setup() {
  //SERIAL SETUP
  Serial.begin(115200);
  while (!Serial);

  //PIN SETUP
  pinMode(LED_PIN  , OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_EN, OUTPUT);
  pinMode(X_BASE, INPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_EN, OUTPUT);
  pinMode(Y_BASE, INPUT);

  pinMode(INPUT_B_MAINSTAGE, INPUT);
  pinMode(INPUT_A_CLOSE_VALVES, INPUT);

  //Serial.println("ENABLE STEPMOTOR X AND Y (on - low, off - high)");
  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);

  //Serial.println("MAXSPEED [STEP/SECOND], ACCELERATION [STEP/SECOND^2]");
  EngineX.setMaxSpeed(1000);    //max predkosc silnika krokow/s
  EngineX.setAcceleration(1000);    //max przyspieszenie silnika krokow/s

  EngineY.setMaxSpeed(1000);
  EngineY.setAcceleration(1000);

  Serial.println("CALIBRATE MOTORS TO ZERO AT ENDSTOP (AND STOP THEM THERE)");
  Engines_StopAndCalibrateAtEndStop();

  valvesOpenFlag = true;
  
  CommandLoop = false;
  emergencyShutDown = false;

 xSetCommand(CommandCurrentX);
 ySetCommand(CommandCurrentY);
   
}

void loop() {

  
    // close valves in case
    if (HIGH == digitalRead(INPUT_A_CLOSE_VALVES) && true == CommandLoop ) {
      Serial.println("A");
      emergencyShutDown = true;
      CommandLoop = false;
      Engines_ShutDown();
    
      valvesOpenFlag = false;
    }
  
    if (HIGH == digitalRead(INPUT_B_MAINSTAGE) && valvesOpenFlag == true && !emergencyShutDown) {
    Serial.println("B");
      CommandLoop = true;
      valvesOpenFlag = true;
    }
  
    if (CommandLoop == true && !emergencyShutDown) {
       xyRunCurrentCommand();
    }
  

 

 
}
