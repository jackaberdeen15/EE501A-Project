#include <Car.h>
 system2 s;
  int motorpinR=11;
  int motorpinL=9; //change the pin number
  int directionpinR=12;
  int directionpinL=7;
  int startpin_input=8;
  int stoppin_input=4;
  int step_pin_in=13; //set target speed to 50 percent
  int step_pin_in1=6; //set target speed to 80 percent
  int pin_interruptR=INT0;
  int pin_interruptL=INT1;
  int number_of_magnets=6; //49
  int echo=6;
  int trig=5;
void setup() {
  Serial.begin(9600);
  
s.system_setup(motorpinR,motorpinL,directionpinR,pin_interruptR,pin_interruptL,number_of_magnets,directionpinL, startpin_input,stoppin_input,echo,trig);
 }

void loop() {
  s.system_execute(); 
}
