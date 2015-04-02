#ifndef umbrella_h
#define umbrella_h

#include "Arduino.h"

// Arduino PWM-ready motor output
// takes a speed and a direction; HIGH for up, LOW for down
// speed from 0 - 100 only
// if needing to brake, set speed 0 and dir to LOW
class motor481{

  private:
    int pwmPin;
    int dirPin;
    int currentSpeed;
    int currentDir;
  
  public:
    motor481(int desPWMPin, int desDirPin)
    {
      pwmPin = desPWMPin;
      dirPin = desDirPin;
      currentSpeed = 0;
      currentDir = 0;
    }
  
    int getSpeed()
    {
      return currentSpeed;
    }
    
    int getDir()
    {
      return currentDir;
    }
    
    void driveMotor(float desSpeed, int desDir)
    {
      // constrain incoming speed to 0 - 100
      currentSpeed = constrain(desSpeed,0,100);
      currentDir = desDir;
      int speedPWM = map(currentSpeed, 0, 100, 0, 255);
      digitalWrite(dirPin, currentDir);
      analogWrite(pwmPin, speedPWM);
      
    }
    
    
};

// change state, automatically update last state variable
void changeState(byte& lastState, byte& stateVariable, byte nextState)
{
  lastState = stateVariable;
  stateVariable = nextState;
}

// for a given duty cycle, blink n times during half the cycle, off for the other half
void blinkCustom(int ledPin, int numBlinks, int blinkPeriod, int restDelay) 
{
  // blink on and off n times during the first half
  for(int i = 0; i < numBlinks; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(blinkPeriod/2);
    digitalWrite(ledPin, LOW);
    delay(blinkPeriod/2);
  }
  
  // stay off for the second half
  delay(restDelay);
}

// uses shiftIn
byte readSystemInputs(int dataPin, int clockPin, int latchPin)
{
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW); // latch inputs
  delay(10); // wait a bit before clocking high
  digitalWrite(latchPin, HIGH);
  digitalWrite(clockPin, LOW); // prepare clock for HTL transition
  
  byte systemInputByte = 0;
  int bitData;
  for (int i = 0; i < 8; i++)
  {
    bitData = digitalRead(dataPin)<<i;
    /*
    Serial.print("i = ");
    Serial.print(i);
    Serial.print(", ");
    Serial.print("bitData = ");
    Serial.println(bitData, BIN);
    */
    systemInputByte = systemInputByte | bitData;
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
  
  return systemInputByte;	
}
#endif

