#ifndef EXPERIMENT1CONTROL_h
#define EXPERIMENT1CONTROL_h

//The following is useful for use with the Visual Studio Projects provided.
#ifdef COMPILE_CPP_NOT_ARDUINO
#include "..\All_Arduino_or_Cpp_symboldefines\All_Arduino_or_Cpp_symboldefines.h"
#else
#include <Arduino.h>
#endif
//Have included libraries above which include classes of components we will use. (Might not need all libraries, can uncomment if they are needed)
//The following library implements the control unit of the system.


class controller1{
	protected:
        static const unsigned long default_check_interval = 300;    //default time interval between each temperature check (ms)
        unsigned long check_interval;                               //time elapsed since last check,
        unsigned long last_check_time;                              //last check time
		
	public:
		// default constructor:
		controller()
		{
			check_interval = default_check_interval;
			last_check_time = 0;
		}
    
        // constructor with arguments (unsigned long integer input):
        controller(unsigned long in_check_interval)
        {
            setup_controller(in_check_interval);
            last_check_time=0;
        }

    
        // set up with arguments (unsigned long integer input):
        void setup_controller(unsigned long in_check_interval) // in_check interval, gets a value and transfers it to check_interval
        {
            check_interval = in_check_interval;
        }
    
        int getInterCheck() // if its time to measure, takes the value from the InterCheck
        {
            return check_interval;
        }
		
		// verifies if it's time to take a new input measurement
		boolean isTimeToTakeMeasurement()
		{
			unsigned long current_time = millis();	// check current time
            
            if(current_time<last_check_time)
                updateCheckTime();
            
			if((current_time - last_check_time) >= check_interval)
				return true;
			else
				return false;
		}
    
        void updateCheckTime()
        {
            last_check_time = millis();
        }
		
        boolean isTimeToTakeMeasurementAndUpdate()
        {
            boolean istime = isTimeToTakeMeasurement();
            if(istime)
                updateCheckTime();
            return istime;
        }
    
        
    
		// issues an action
        void issueCommand(){ //add return mapped value to here
			
			if( isTimeToTakeMeasurement() ){ 
			     
			                                      	// avoid making decisions if it's too early ti take measurements, defaults time hasnt elapsed
				updateCheckTime();                              // take the current time as the most recent when a successful measurement was done
                
			}
		}  
		
		  //print CSV
        void printCSV(double RPM, double pkVal, double settleMin, double settlingValue){
       
            Serial.print(RPM);
            Serial.print(",");
           
            Serial.print(pkVal);
            Serial.print(",");
           
            Serial.print(settleMin);
            Serial.print(",");
           
            Serial.println(settlingValue);
       
        }
		
};
#endif
