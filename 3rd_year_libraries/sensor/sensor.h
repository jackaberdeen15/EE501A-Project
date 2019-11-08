

#ifndef Sensor
#define Sensor
#include <InterruptBasedSpeedMeasure.h>

class sensor{

protected:

	InterruptSpeedMeasure_SMA speedometer; //define speed sensor object
    int potentiometer;
 
  public:      
 
 
 //setup takes interrupt pin, number of magnets on circumference, potentiometer pin, direction pin
 void setup_sensing(ArduinoInterruptNames pin_interrupt, int number_poles, double pin_potentiometer=A1){
     
    //setup speedomter interrupt object
 	speedometer.setupSpeedMeasure(pin_interrupt,number_poles);
 	
 	//setup input pins
 	potentiometer =pin_potentiometer;
 //	pinMode(potentiometer,INPUT);
 
  }
 void resetDistanceCount()
 {
 	return speedometer.reset_distancecount();
 }
 bool check_distance_met(int targetCount)
 {
 	 return speedometer.checkDistanceMet(targetCount);
 }
 int GetDistanceCount()
 {
  return speedometer.GetkDistanceCount();
}
 double requestRPM()
 { //returns RPM when interrogated

	double RPM;
	RPM=speedometer.getRPMandUpdate();	

	if(RPM>0)
    {
    //	Serial.println("\n");
    // Serial.print("RPM = ");
    //Serial.println(RPM);
    }  
	 
	return RPM ; 	
}
 
int return_mapped_input_RPM(int input_value){
        	    int input;
				input=analogRead(input_value);
                int mapped_pwm = map(input, 0, 1023, 0, 60);
                return mapped_pwm;
            }

 int return_mapped_input(int input_value){
        	    int input;
				input=analogRead(input_value);
                int mapped_pwm = map(input, 0, 1023, 0, 250);
                return mapped_pwm;
            }
            

	
};
#endif
