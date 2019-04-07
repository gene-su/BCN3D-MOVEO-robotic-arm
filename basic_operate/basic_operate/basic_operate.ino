//-------------------Initial Stepper----------------------
//AccelStepper mystepper(1, pinStep, pinDirection)
#include <AccelStepper.h>
AccelStepper Stepper1(1, 8, 2);
AccelStepper Stepper2(1, 9, 3);
AccelStepper Stepper3(1, 10, 4);
AccelStepper Stepper4(1, 11, 5);
AccelStepper Stepper5(1, 12, 6);
AccelStepper Stepper6(1, 13, 7);

//-------------------Initial Servo------------------------
#include <Servo.h>
Servo myservo;
int release_angle = 50;
int catch_angle = 83;
int pos = 0;

// -------------Motor Funtions---------------

//0. All motor setup
void Motor_setup()
{
  myservo.attach(14);  // 將 servo 物件連接到 pin 14
  myservo.write(release_angle);  // Initial myservo Position
  Stepper1.setMaxSpeed(600);
  Stepper1.setAcceleration(200);
  Stepper2.setMaxSpeed(1200);
  Stepper2.setAcceleration(200);
  Stepper3.setMaxSpeed(1800);
  Stepper3.setAcceleration(200);
  Stepper4.setMaxSpeed(1200);
  Stepper4.setAcceleration(200);
  Stepper5.setMaxSpeed(1200);
  Stepper5.setAcceleration(200);
  Stepper6.setMaxSpeed(1200);
  Stepper6.setAcceleration(200);
}

// ------------------------------------------

void setup() 
{
  // put your setup code here, to run once:
  Motor_setup();
  Serial.begin(9600);
}

int steps[6] = {0, 0, 0, 0, 0, 0};

void loop() 
{
  if (Stepper1.distanceToGo() != 0 || 
      Stepper2.distanceToGo() != 0 ||
      Stepper3.distanceToGo() != 0 ||
      Stepper4.distanceToGo() != 0 ||
      Stepper5.distanceToGo() != 0 ||
      Stepper6.distanceToGo() != 0 ||
      steps[5] != 0)
  {
    Stepper1.run();
    Stepper2.run();
    Stepper3.run();
    Stepper4.run();
    Stepper5.run(); 
    Stepper6.run();
// Let the movement of gripper be the last one. 
    if (Stepper1.distanceToGo() == 0 &&
        Stepper2.distanceToGo() == 0 &&
        Stepper3.distanceToGo() == 0 &&
        Stepper4.distanceToGo() == 0 &&
        Stepper5.distanceToGo() == 0 &&
        Stepper6.distanceToGo() == 0)
    {
      if (steps[5] == 1) // Open
      { 
        for (pos = catch_angle; pos >= release_angle; pos -=1)
        {
          myservo.write(pos);
          delay(30);
        }
        steps[5] = 0;
      }
      if (steps[5] == 2) // Close
      { 
        for (pos = release_angle; pos <= catch_angle; pos +=1)
        {
          myservo.write(pos);
          delay(30);
        }
        steps[5] = 0;
      }
      Serial.println("OK");  // Let we know the robotic arm is ready for the next command.
      delay(1000);
    }        
  }
  else {
// When serial get new command, set new steps(array).
     if (Serial.available())
     {
      for(int i = 0; i < 6; i++) 
      {
        steps[i] = 0;
      }
        
      int i = 0;
      while(Serial.available())
      {
        steps[i] = Serial.readStringUntil(',').toInt();
        i = i+1;
        delay(10);
      }
// Assigning steps which each steppers has to move.
      Stepper1.move(steps[0]);
      Stepper2.move(steps[1]);
      Stepper3.move(steps[2]);
      Stepper4.move(steps[3]);
      Stepper5.move(-steps[3]);
      Stepper6.move(steps[4]);
    }
  }    
}
