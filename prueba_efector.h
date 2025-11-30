#include <Servo.h>

Servo servoSG90;
Servo HK15138;
int angle1 = 0;
int angle2 = 0;


void setup() {
  servoSG90.attach(10);
  HK15138.attach(9);
  servoSG90.write(angle1);
  HK15138.write(angle2);
}

void loop() 
{ 

  for(angle1 = 0; angle1 < 180; angle1++)  
  {                                  
    HK15138.write(angle1);               
    delay(15);                   
  } 

  for(angle2 = 0; angle2 < 90; angle2++)  
  {                                  
    servoSG90.write(angle2);               
    delay(15);                   
  } 

  for(angle1 = 180; angle1 > 0; angle1--)  
  {                                  
    HK15138.write(angle1);               
    delay(15);
  }

  for(angle2 = 90; angle2 > 0; angle2--)  
  {                                  
    servoSG90.write(angle2);               
    delay(15);
  }


}
