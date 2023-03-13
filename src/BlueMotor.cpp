#include <Romi32U4.h>
#include "Timer.h"
#include <Arduino.h>
#include "BlueMotor.h"

long volatile count = 0;

BlueMotor::BlueMotor() {}

void BlueMotor::isrA()
{
  if (digitalRead(ENCB) == digitalRead(ENCA))
  {
    count--;
  }
  else
  {
    count++;
  }
}

void BlueMotor::isrB()
{
  //Serial.println("B");
  if (digitalRead(ENCA) == digitalRead(ENCB))
  {
    count++;
  }
  else
  {
    count--;
  }
}

void BlueMotor::setup()
{
  pinMode(PWMOutPin, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  TCCR1A = 0xA8; // 0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
  TCCR1B = 0x11; // 0b00010001;
  ICR1 = 400;
  OCR1C = 0;
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
}

void BlueMotor::reset()
{
  count = 0;
}

// Timer printTimer(500);

// void loop()
// {
//   if (printTimer.isExpired())
//   {
//     long value;
//     noInterrupts();
//     value = count;
//     interrupts();
//     Serial.println(value);
//   }
// }

void BlueMotor::setEffort(int effort)
{
  if (effort < 0)
  {
    setEffort(-effort, true);
  }
  else
  {
    setEffort(effort, false);
  }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
  if (clockwise)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::setEffortWithoutDB(int effort) {
  const int motorFF = 149;
  if (effort > 0) {
    setEffort(effort / 400.0 * (400-motorFF) + motorFF);
  } else if (effort < 0) {
    setEffort(effort / 400.0 * (400-motorFF) - motorFF);
  } else {
    setEffort(0);
  }
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

double kp = 5;

// Move to this encoder position within the specified
// tolerance in the header file using proportional control
// then stop
bool BlueMotor::moveTo(long target)
{
  long diff = -(target - getPosition());

  if (abs(diff) < 10)
  {
    setEffort(0);
    return true;
  }
  setEffortWithoutDB((diff * kp));
  return false;
}
