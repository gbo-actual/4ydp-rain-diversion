/* 
* AUTOMATIC RAIN DIVERSION SYSTEM
* Control Code
* ME 482 Capstone Project
* March 31, 2015
* Team: C.Shum, J.Leung, F.Shabbeer, J.Gilmer, G.Ongpauco
* Coded by: G.Ongpauco
*/

#include "umbrella.h"

// inputs
// use shift register instead
// values are bitmasks
// inputs are pulled down
byte buttonStartPause = 1<<0; // general purpose input
byte limitSwitchBottom = 1<<1;
byte limitSwitchTop = 1<<2;
byte rainSensorOverrideEnable = 1<<3; // because I couldn't get a SPDT switch!
byte rainSensorOverrideSignal = 1<<4; // because I couldn't get a SPDT switch!
byte motorOverrideEnable = 1<<5;
byte motorOverrideUp = 1<<6;
byte motorOverrideDown = 1<<7;
// 0 used for RX

// Arduino pin inputs
byte rainSensor = 4;
byte inputLatch = 5;
byte inputClock = 6;
byte inputData = 7;

// outputs
int motorPWM = 3;
int motorDirection = 2;
int statusLED = 13;
// 1 used for TX

// constants

// states
const byte stateEmergencyStop = -2; // emergency state
const byte stateCanopyReset = -1; // immediately after startup, require human input
const byte stateMonitorEnvironment = 0; // redundant with canopyClosed
const byte stateCanopyOpen = 1; // canopy is fully open (at top)
const byte stateCanopyRising = 2; // motor is driving canopy upwards
const byte stateCanopyLowering = 3; // motor is driving canopy downwards
const byte stateCanopyClosed = 4; // canopy is fully closed (at bottom)
const byte statePause = 5; // motor is stopped in the middle for any reason
const byte stateMotorOverride = 6; // we are now controlling the motor

byte systemState = stateCanopyReset; // internal state variable. initialize to idle, pre-operation mode
byte lastSystemState = systemState; // controller must remember the previous state in case of emergencies
byte statePauseReturn = stateCanopyReset; // state to return to from a pause
byte stateOverrideReturn = stateCanopyReset; // state to return to from an override

// other variables
byte systemInputs = 0; // initialize all inputs to OFF
byte lastMotorDirection = LOW; // initialize to LOW
byte lastMotorSpeed = 0; // initialize to 0
byte preOverrideDirection = LOW; // separate record for manual control
byte preOverrideSpeed = 0;
bool rainSensorValue = false;
motor481 gearMotor(motorPWM, motorDirection); // declare instance of motor class
int cooldownDurationms = 3*1000; // seconds the canopy must wait after reaching either limit switch before moving again

// motor control
bool isManual = false; 
int manualUpSpeed = 80;
int manualDownSpeed = 60;
int upSpeed = 90;
int downSpeed = 60;

void setup() {
  // put your setup code here, to run once:
  
  // configure inputs
  pinMode(rainSensor, INPUT);
  pinMode(inputData, INPUT);
  
  // configure outputs
  pinMode(inputLatch, OUTPUT);
  pinMode(inputClock, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  pinMode(statusLED, OUTPUT);
  
  // shift register
  digitalWrite(inputClock, LOW);
  digitalWrite(inputLatch, HIGH);
  
  // begin serial comms
  Serial.begin(9600);
  
  Serial.println("==========Automatic Rain Diversion System - DISTRICT 12==========");
  Serial.println("C.Shum, J.Leung, F.Shabbeer, J.Gilmer, G.Ongpauco");
  Serial.println();
  Serial.println(">>STARTUP SEQUENCE"); // intro text
  Serial.print("Safety cooldown of ");
  Serial.print(cooldownDurationms/1000);
  Serial.println(" seconds is in effect.");
  Serial.println("System will not move for the specified duration before state transition.");

  
  // wait for start/pause to be toggled off then on
  // do not allow the system to be started with SP in the ON position
  systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // read inputs from sr
  // print input status
  if(digitalRead(rainSensor)==HIGH)
  {
    Serial.println("rainSensor = ON");
  }
  
  if(systemInputs&buttonStartPause)
  {
    Serial.println("buttonStartPause = ON");
  }
  
  if(systemInputs&limitSwitchBottom)
  {
    Serial.println("limitSwitchBottom = ON");
  }
  
  if(systemInputs&limitSwitchTop)
  {
    Serial.println("limitSwitchTop = ON");
  }
  
  if(systemInputs&rainSensorOverrideEnable)
  {
    Serial.println("rainSensorOverrideEnable = ON");
  }
  
  if(systemInputs&rainSensorOverrideSignal)
  {
    Serial.println("rainSensorOverrideSignal = ON");
  }
  
  if(systemInputs&motorOverrideEnable)
  {
    Serial.println("motorOverrideEnable = ON");
  }
  
  if(systemInputs&motorOverrideUp)
  {
    Serial.println("motorOverrideUp = ON");
  }
  
  if(systemInputs&motorOverrideDown)
  {
    Serial.println("motorOverrideDown = ON");
  }

  //if(digitalRead(buttonStartPause)==HIGH)
  if(systemInputs & buttonStartPause)
  {
    Serial.println("START/PAUSE button is [ON] before system startup. Toggle to [OFF] then [ON].");
  }
 
  //while(digitalRead(buttonStartPause)==HIGH)
  while(readSystemInputs(inputData, inputClock, inputLatch) & buttonStartPause)
  {
    blinkCustom(statusLED, 1, 1000/2, 0); // blink till toggle OFF
  }
  // button is now in the off position
  Serial.println("Toggle START/PAUSE to [ON] to begin operation.");

  //while(digitalRead(buttonStartPause)==LOW) // wait for toggle ON
  while((readSystemInputs(inputData, inputClock, inputLatch) & buttonStartPause)==false)
  {
    blinkCustom(statusLED, 3, 1000/4, 1000/4); // blink some LED until toggle ON
  } 
  // button is now ON
  Serial.println("[ON] signal received. Beginning autonomous operation.");
  Serial.println("Verifying canopy is fully open or closed.");  // check where the canopy is
  // transition to canopy reset in case the canopy is stuck in the middle  
  systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // read inputs from sr
  //if (digitalRead(limitSwitchBottom)==LOW&&digitalRead(limitSwitchTop)==LOW)
  //Serial.print("!(systemInputs & (limitSwitchBottom | limitSwitchTop)) = ");
  if(!(systemInputs & (limitSwitchBottom | limitSwitchTop)))
  {
    Serial.println(">>STATE - CANOPY RESET");
    Serial.println("Canopy is neither fully open nor closed. Resetting to fully closed position.");
    Serial.print("<<Cooldown in effect... ");
    delay(cooldownDurationms); // wait
    Serial.println("Cooldown complete. Ready to move.>>");
    systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // re-check inputs
    if(!(systemInputs & (limitSwitchBottom | limitSwitchTop)))
    {
      gearMotor.driveMotor(downSpeed,LOW); // drive canopy downwards
    }
    systemState = stateCanopyReset; // system is ready to go autonomous!
  }
  
  //else if(digitalRead(limitSwitchBottom)==HIGH) // canopy is fully closed
  else if(systemInputs & limitSwitchBottom)
  {
    Serial.println("Canopy is fully closed. Beginning normal operation.");
    Serial.println(">>STATE - CANOPY CLOSED");
    Serial.print("<<Cooldown in effect... ");
    delay(cooldownDurationms); // wait
    Serial.println("Cooldown complete. Ready to move.>>");
    Serial.println("Waiting for rain sensor to read [HIGH].");
    systemState = stateCanopyClosed;
  }
  
  // else if(digitalRead(limitSwitchTop)==HIGH) // canopy is fully open
  else if(systemInputs & limitSwitchTop)
  {
    //Serial.println("Canopy is fully open. Beginning normal operation.");
    Serial.println(">>STATE - CANOPY OPEN");
    Serial.print("<<Cooldown in effect... ");
    delay(cooldownDurationms); // wait
    Serial.println("Cooldown complete. Ready to move.>>");
    Serial.println("Waiting for rain sensor to read [LOW].");
    systemState = stateCanopyOpen;
  }
  
  // do not care if both limit switches are simultaneously on, prioritize closing the canopy
    
}

void loop() 
{
  // shift register input
  systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // read inputs
  // input data has been written to the byte, now bitmask to check if on
  
  rainSensorValue = digitalRead(rainSensor); // read the rain sensor (not on shift register)
  // simple state machine
  // system must never loop waiting for a condition inside a state
  // system repeatedly goes through the switch statement constantly checking states
  // this ensures that other inputs can interrupt or pause the state machine  
  
  // -----OVERRIDES--------------------
  
  // first read rain sensor, check for override
  //if(digitalRead(rainSensorOverrideEnable)==HIGH)
  if(systemInputs & rainSensorOverrideEnable)
  {
    //rainSensorValue = digitalRead(rainSensorOverrideSignal); // read from the override switch instead
    rainSensorValue = (systemInputs & rainSensorOverrideSignal);
  }  
  
  // states/transitions accessible only if the system is not paused
  if(systemState!=statePause)
  {
    // pause transition; user has toggled start/pause to OFF
    //if(digitalRead(buttonStartPause)==LOW)
    if(bool(systemInputs & buttonStartPause)==false)
    {
      Serial.println("((---PAUSED - TOGGLE START/PAUSE BUTTON TO [ON] TO RESUME---))");
      // save speed and direction of motor
      lastMotorSpeed = gearMotor.getSpeed();
      lastMotorDirection = gearMotor.getDir();
      statePauseReturn = systemState; // remember the state you paused from
      // no delay, pause must be instantaneous for safety
      // stop the motor
      gearMotor.driveMotor(0, LOW);
      systemState = statePause; // overrides any previous state
    }
    
    // motor control override
    // pause has higher priority, hence the else; if pause is TRUE, then do not enable motor override!
    //else if(digitalRead(motorOverrideEnable)==HIGH&&systemState!=stateMotorOverride)
    else if((systemInputs & motorOverrideEnable)&&!(systemState==stateMotorOverride))//&&!(systemState==stateMotorOverride))==true);
    {
      Serial.println(">>STATE - MOTOR OVERRIDE");
      Serial.println("[[---MOTOR OVERRIDE ENABLED, SYSTEM UNDER MANUAL CONTROL---]]");
      // no delay, motor override must be instantaneous for safety
      isManual = true;
      preOverrideSpeed = gearMotor.getSpeed(); // treat it like a pause; get the motor's speed and direction
      preOverrideDirection = gearMotor.getDir();
      stateOverrideReturn = systemState; // treat it like a pause
      gearMotor.driveMotor(0, LOW);
      systemState = stateMotorOverride;
    }
  }    
  
  // state logic
  // each case is a state, and will only transition out when the specified conditions occur
  switch (systemState)
  {
    // system constantly updating and checking states
    // states will transition only when conditions are fulfilled, coded at the end of each state
    
    // -----REGULAR OPERATION STATES--------------------
    
    // reset canopy at startup
    // motor is driving canopy downwards
    // one-way transition out of this state
    case stateCanopyReset:
      // check if canopy closed
      // if(digitalRead(limitSwitchBottom)==HIGH)
      if(systemInputs & limitSwitchBottom)
      {
        //Serial.println("Canopy fully closed. Beginning normal operation.");
        Serial.println(">>STATE - CANOPY CLOSED");
        Serial.println("Waiting for rain sensor to read [HIGH].");
        gearMotor.driveMotor(0, LOW); // turn off the motor!
        // Timer1.start(); // start the cooldown timer
        delay(cooldownDurationms); // wait
        changeState(lastSystemState, systemState, stateCanopyClosed);
      }
      // this can be interrupted by the START/PAUSE toggle
      // this functionality was not present before :(
      break;
    
    // canopy is at bottom, motor is off
    case stateCanopyClosed:
      // check rain sensor
      // if rain sensor reads HIGH, open the canopy
      // start driving the motors
      // set systemState to stateCanopyRising
      if(rainSensorValue==true)
      {
        Serial.println("Rain sensor reads [HIGH].");
	Serial.print("<<---Cooldown in effect... ");
	delay(cooldownDurationms); // wait
	Serial.println("Cooldown complete.--->>");
	Serial.println("Raising canopy.");

        Serial.println(">>STATE - CANOPY RISING");
        Serial.println("Waiting for top limit switch to read [HIGH].");
        systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // re-check limit switches
        if(!(systemInputs & limitSwitchTop)) // only move motor if the top limit switch hasn't been hit during delay
        {
          gearMotor.driveMotor(upSpeed, HIGH);          
        }
        
        changeState(lastSystemState, systemState, stateCanopyRising);
      }
      
      // else do nothing
      // can add delay if necessary
      break;
    
    // motor is driving canopy upwards inbetween top and bottom limit switches
    case stateCanopyRising:
      // check if the top limit switch has been actuated
      // if hit, stop motor and switch states to canopy open
      // use encoders to briefly check speed, either using interrupts or measuring pulse timing
      //if(digitalRead(limitSwitchTop)==HIGH)
      if(systemInputs & limitSwitchTop)
      {
        gearMotor.driveMotor(0, LOW); // stop motor
        Serial.println("Canopy fully open."); // serial output
        Serial.println(">>STATE - CANOPY OPEN");
	Serial.print("<<---Cooldown in effect... ");
	delay(cooldownDurationms); // wait
	Serial.println("Cooldown complete.--->>");
        Serial.println("Waiting for rain sensor to read [LOW].");

        changeState(lastSystemState, systemState, stateCanopyOpen); // switch states
      }
      
      // else do nothing
      break;
    
    // canopy is fully open, motor stopped, waiting for rain sensor to read false
    case stateCanopyOpen:
      // check if rain sensor reads LOW
      // if yes, start closing the umbrella, change state to canopyLowering
      if(rainSensorValue==false) // read rain sensor
      {
        Serial.println("Rain sensor reads [LOW].");
	Serial.print("<<---Cooldown in effect... ");
	delay(cooldownDurationms); // wait
	Serial.println("Cooldown complete.--->>");
	Serial.println("Lowering canopy.");
	Serial.println(">>STATE - CANOPY LOWERING");
        Serial.println("Waiting for bottom limit switch to read [HIGH].");
        systemInputs = readSystemInputs(inputData, inputClock, inputLatch); // re-check limit switches
        if(!(systemInputs & limitSwitchTop)) // only move motor if the top limit switch hasn't been hit during delay
        {
          gearMotor.driveMotor(downSpeed, LOW);          
        }
        changeState(lastSystemState, systemState, stateCanopyLowering);
      }
      
      // else do nothing
      break;
    
    // motor is driving canopy downwards towards bottom limit switch
    case stateCanopyLowering:
      // check if the bottom limit switch has been actuated
      // if yes, stop motor and enter canopyClosed state
      //if(digitalRead(limitSwitchBottom)==HIGH)
      if(systemInputs & limitSwitchBottom)
      {
        gearMotor.driveMotor(0, LOW); // stop motor
        Serial.println("Canopy fully closed."); // serial output
        Serial.println(">>STATE - CANOPY CLOSED");
	Serial.print("<<---Cooldown in effect... ");
	delay(cooldownDurationms); // wait
	Serial.println("Cooldown complete.--->>");
        Serial.println("Waiting for rain sensor to read [HIGH].");

        changeState(lastSystemState, systemState, stateCanopyClosed); // switch states to stateCanopyClosed
      }
      
      // else do nothing
      break;
    
    // -----NONSTANDARD/EMERGENCY CASES--------------------
    
    // pause from some input button, doing nothing
    // pause button is depressed :(
    case statePause:
      // do nothing until toggle is set to OFF
      // exit transition
      //if(digitalRead(buttonStartPause)==HIGH)
      if(systemInputs & buttonStartPause)
      {
        Serial.print("((___UNPAUSED - RETURNING TO ");        
        switch (statePauseReturn)
        {
          case stateCanopyClosed:
            Serial.print("[CANOPY CLOSED]");
            break;
            
          case stateCanopyRising:
            Serial.print("[CANOPY RISING]");
            break;
            
          case stateCanopyOpen:
            Serial.print("[CANOPY OPEN]");
            break;
            
          case stateCanopyLowering:
            Serial.print("[CANOPY LOWERING]");
            break;
            
          case stateCanopyReset:
            Serial.print("[CANOPY RESET]");
            break;
            
          case stateMotorOverride:
            Serial.print("[MOTOR OVERRIDE]");
            break;
        }
        Serial.println("___))");
        
        // restart motor to pre-pause state
        // only move if the limit switches are not actuated
        // may break if the motor starts moving but is paused before the limit switch disengages
        //if (digitalRead(limitSwitchBottom)==LOW&&digitalRead(limitSwitchTop)==LOW)
	if(!(systemInputs & (limitSwitchBottom|limitSwitchTop)))
        {
          //gearMotor.driveMotor(preOverrideSpeed, preOverrideDirection);
          //Serial.println(!(systemInputs & (limitSwitchBottom|limitSwitchTop)));
          gearMotor.driveMotor(lastMotorSpeed, lastMotorDirection); // restart motor
        }

        systemState = statePauseReturn; // get back to business
      }
      
      // else do nothing
      break;
      
    // motor manual control from remote
    case stateMotorOverride:
      //if(digitalRead(motorOverrideDown)==HIGH&&digitalRead(limitSwitchBottom)==LOW) // pressed down on the motor direction switch, and canopy 
      if(bool(systemInputs & motorOverrideDown) && bool(!(systemInputs & limitSwitchBottom)))
      {
        gearMotor.driveMotor(manualDownSpeed, LOW);
      }
      // prioritize downward movement over upward in the case that something goes wrong and both are activated
      //else if(digitalRead(motorOverrideUp)==HIGH&&digitalRead(limitSwitchTop)==LOW) // pressed up on the motor direction switch, and the canopy is not fully open
      else if (bool(systemInputs & motorOverrideUp) && bool(!(systemInputs & limitSwitchTop)))
      {
        gearMotor.driveMotor(manualUpSpeed, HIGH);
      }
      // if both are off, brake
      else
      {
        gearMotor.driveMotor(0, LOW);
      }
	
      //if(digitalRead(motorOverrideEnable)==LOW) // if motor enable switch is OFF, exit motor override
      if((systemInputs & motorOverrideEnable)==false)
      {
        Serial.print("motorOverrideEnable ");
        Serial.println(systemInputs&motorOverrideEnable);
        Serial.print("[[___Motor override disabled. Returning to ");
        switch (stateOverrideReturn)
        {
          case stateCanopyClosed:
            Serial.print("[CANOPY CLOSED]");
            break;
            
          case stateCanopyRising:
            Serial.print("[CANOPY RISING]");
            break;
            
          case stateCanopyOpen:
            Serial.print("[CANOPY OPEN]");
            break;
            
          case stateCanopyLowering:
            Serial.print("[CANOPY LOWERING]");
            break;
            
          case stateCanopyReset:
            Serial.print("[CANOPY RESET]");
            break;
            
          case statePause:
            Serial.print("[PAUSE]");
            break;
            
          case stateMotorOverride:
            Serial.print("[MOTOR OVERRIDE] (!!!)ERROR(!!!)");
            break;
        }
        Serial.println("___]]");
        // return motor to pre-override conditions
        // only move if the limit switches are not hit
        //if (digitalRead(limitSwitchBottom)==LOW&&digitalRead(limitSwitchTop)==LOW)
	if((systemInputs & (limitSwitchBottom|limitSwitchTop))==false)
        {
          gearMotor.driveMotor(preOverrideSpeed, preOverrideDirection);
        }
        
        isManual = false;
        systemState = stateOverrideReturn; // go back to what you were doing
      }
      break;
   
  // any state transitions have been applied
  // else loop back to beginning and check inputs again
  }
}
