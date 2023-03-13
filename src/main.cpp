#include <Romi32U4.h>
#include <servo32u4.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#include <Rangefinder.h>


#define NONLINEAR_FEEDBACK A0

BlueMotor motor;
Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
Rangefinder rangefinder(3,2);
Chassis chassis;
Servo32U4 servo;

IRDecoder decoder(14);

// all standard EXCEPT: IR 6, rangefinder echo 3, 2

void setup()  
{
  Serial.begin(9600);
  pinMode(NONLINEAR_FEEDBACK, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  motor.setup();
  motor.reset();
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
  rangefinder.init();
  decoder.init();
  delay(2000);
}

typedef bool (*Loopable)(); //typedef to allow for an array of function ptrs
int idx = 0; //index in command queue. This would be implemented differently but we don't have full c++ std

Loopable waitForChassis = []() {
      return chassis.checkMotionComplete();
};


int openPos = 1651;
int closePos = 1150;
int servoClose = 1300;
int servoOpen = 1700;
int servoStop = 1490;
int closedPotVal = 610;
int openPotVal = 330;

bool closeGripper() {
  if (digitalRead(A4) == HIGH) {
    for (int pos = openPos; pos >= closePos; pos -= 3) // Gripper close
      {
        servo.writeMicroseconds(pos); // Gripper close
    }
    return true;
  } else {
    servo.writeMicroseconds(servoClose);
    int potVal = analogRead(A0);
    if (potVal >= closedPotVal) {
      servo.writeMicroseconds(servoStop);
      return true;
    }
    return false;
  }
}

bool openGripper() {
  if (digitalRead(A4) == HIGH) {
    servo.writeMicroseconds(openPos); // Gripper open
    delay(30);
    return true;
  } else {
    servo.writeMicroseconds(servoOpen);
    int potVal = analogRead(A0);
    if (potVal <= openPotVal) {
      servo.writeMicroseconds(servoStop);
      return true;
    }
    return false;
  }
}

bool driveToRoofPickupShallow() {
  Serial.print("left: ");
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  double dist_err = -(rangefinder.getDistance() - 9);
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-5, line_err * .1);
  if (abs(dist_err) < 1) {
    chassis.setTwist(0,0);
  }
  return abs(dist_err) < 1;
}

bool driveToRoofPlaceShallow() {
  Serial.print("left: ");
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  double dist_err = -(rangefinder.getDistance() - 6);
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-5, line_err * .1);
  if (abs(dist_err) < 1) {
    chassis.setTwist(0,0);
  }
  return abs(dist_err) < 1;
}

bool driveToRoofPickupSteep() {
  Serial.print("left: ");
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  double dist_err = -(rangefinder.getDistance() - 7.5);
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-5, line_err * .1);
  if (abs(dist_err) < 1) {
    chassis.setTwist(0,0);
  }
  return abs(dist_err) < 1;
}

bool driveToRoofPlaceSteep() {
  Serial.print("left: ");
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  double dist_err = -(rangefinder.getDistance() - 8);
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-5, line_err * .1);
  if (abs(dist_err) < 1) {
    chassis.setTwist(0,0);
  }
  return abs(dist_err) < 1;
}
bool driveToBlock() {
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  double dist_err = -(rangefinder.getDistance() - 4);
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-5, line_err * .1);
  if (abs(dist_err) < 1) {
    chassis.setTwist(0,0);
  }
  return abs(dist_err) < 1;
}

bool romiRacing() {
  Serial.print(analogRead(A2));
  int line_err = analogRead(A2) - 450;
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(-40, line_err * .2);
  return false;
}

bool turnToBlack() {
  int line_err = analogRead(A2) - 450;
  Serial.print("line_err: ");
  Serial.println(line_err);
  Serial.print("dist: ");
  Serial.println(rangefinder.getDistance());
  chassis.setTwist(0, 45);
  if (line_err > 0) {
    chassis.setTwist(0,0);
  }
  return line_err > 0;
}
Loopable waitForEnter = []() {
  return decoder.getKeyCode(true) == ENTER_SAVE;
};

Loopable commands[] = {
  waitForEnter,
  openGripper,
  []() { //move blue motor to high position
    return motor.moveTo(-5350);
  },
  driveToRoofPickupShallow, //drive to pickup collector
  []() {
    return motor.moveTo(-3900);
  },
  closeGripper, //PICK UP FIRST PANEL
  waitForEnter,
  []() { //drive bacwards and lower 4bar
    chassis.driveFor(15,15,false);
    return true;
  },
  waitForChassis,
  []() {
    return motor.moveTo(0);
  },
  []() {
    chassis.turnFor(-90,45,false); //turn towards the staging block
    return true;
  },
  waitForChassis,
  driveToBlock,
  waitForEnter,
  openGripper, //PLACE FIRST PANEL
  []() {
    chassis.driveFor(10,10,false);
    return true;
  },
  waitForChassis,
  waitForEnter,
  []() {
    chassis.driveFor(-12,-12,false);
    return true;
  },
  waitForChassis,
  closeGripper, //PICK UP SECOND PANEL
  []() {
    chassis.driveFor(27,10,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.turnFor(90,45,false);
    return true;
  },
  []() { //move to motor and turn at the same time
    return motor.moveTo(-3900) && chassis.checkMotionComplete();
  },
  []() {
    Serial.print("dist: ");
    Serial.println(rangefinder.getDistance());
    delay(1000);
    return true;
  },
  driveToRoofPlaceShallow, //drive to place the collector
  []() {
    return motor.moveTo(-5350);
  },
  waitForEnter,
  openGripper, //PLACED 2nd PANEL
  []() {
    chassis.driveFor(10,10,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.turnFor(-45,45,false);
    return true;
  },
  []() {
    return motor.moveTo(0) && waitForChassis;
  },
  []() {
    chassis.driveFor(-45,20,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.turnFor(90,45,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.driveFor(-50,20,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.turnFor(135,45,false);
    return true;
  },
  waitForChassis, //AT OTHER SIDE
  /* SECOND ROBOT START */
  []() { //NUM_1
    return motor.moveTo(-2400);
  },
  []() {
    chassis.driveFor(5,10,false);
    return true;
  },
  []() {
    return openGripper() && chassis.checkMotionComplete();
  },
  []() {
    Serial.print("dist: ");
    Serial.println(rangefinder.getDistance());
    delay(1000);
    return true;
  },
  driveToRoofPickupSteep,//PICKUP THIRD PANEL
  closeGripper,
  waitForEnter,
  []() {
    chassis.driveFor(25,15,false);
    return true;
  },
  []() {
    return motor.moveTo(-6000) && waitForChassis();
  },
  []() {
    chassis.turnFor(155,45,false);
    return true;
  },
  waitForChassis,
  driveToBlock,
  waitForEnter,
  []() {
    return motor.moveTo(0);
  },
  openGripper, //PLACE THIRD PANEL
  []() {
    chassis.driveFor(10,10,false);
    return true;
  },
  waitForChassis, //placed on block
  waitForEnter,
  []() {
    chassis.driveFor(-12,-12,false);
    return true;
  },
  waitForChassis,
  closeGripper, //PICK UP FOURTH PANEL
  []() {
    return motor.moveTo(-6000);
  },
  []() {
    chassis.driveFor(22,10,false);
    return true;
  },
  waitForChassis,
  []() {
    chassis.turnFor(-90,45,false);
    return true;
  },
  waitForChassis,
  []() {
    Serial.print("dist: ");
    Serial.println(rangefinder.getDistance());
    delay(1000);
    return true;
  },
  []() {
    return motor.moveTo(-3900);
  },
  driveToRoofPlaceSteep,
  openGripper, //placed 4th
  waitForEnter,
  []() {
    chassis.driveFor(10,10,false);
    return true;
  },
  []() {
    return motor.moveTo(-2700) && waitForChassis();
  },
  []() {
    return motor.moveTo(0);
  },
  nullptr
};


bool paused = false;
int leftTarget = 0;
int rightTarget = 0;
int leftCount = 0;
int rightCount = 0;
int leftSpeed = 0;
int rightSpeed = 0;

void loop() {

  if(!paused && commands[idx] && commands[idx]()) { //the command exists and evaluates to true
    idx++; //go to next command    
  }

  switch(decoder.getKeyCode(false)) { //stop / restart
    case STOP_MODE:
      paused = !paused;
      Serial.print("paused?: ");
      Serial.println(paused);
      if (paused) {
        leftTarget = chassis.leftMotor.targetPos;
        rightTarget = chassis.rightMotor.targetPos;
        leftCount = chassis.leftMotor.getCount();
        rightCount = chassis.rightMotor.getCount();
        leftSpeed = chassis.leftMotor.targetSpeed;
        rightSpeed = chassis.leftMotor.targetSpeed;
        chassis.setMotorEfforts(0,0);
        motor.setEffort(0);
        if (digitalRead(A4) == LOW) {
          servo.writeMicroseconds(servoStop);
        }
      } else {
        chassis.leftMotor.moveFor(leftTarget - leftCount);
        chassis.rightMotor.moveFor(rightTarget - rightCount);
      }
      break;
    case NUM_1:
      idx = 43; //halfway point
      Serial.println("skipped");
  }

  if (readBatteryMillivolts() < 6000) {
    Serial.println("battery warning");
  }
  delay(50);
}