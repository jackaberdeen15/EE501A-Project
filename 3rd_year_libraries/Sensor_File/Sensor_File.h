#ifndef Sensor_File
#define Sensor_File
#include "..\UltrasoundSensor\UltrasoundSensor.h"

#include<Arduino.h>
//class for Sensor Unit

class ultra_sensor{
	protected:
		PingProximitySensor beep;		
		int mindistance=5;
  		int maxdistance=120;  	
  		int measured_distance;  		
  	    bool measurement_success;
  	
  	public:
  		// setup function //
  	    void setup_sensor(int trigger,int echo)
  	    {
  	    	beep.setup_PingProximitySensor(trigger,echo);
		  }
		
		void measurement() {		     	
			  measurement_success=beep.SenseDistance(measured_distance);
			   		}
		// returning measured distance //
  		int current_distance()
  		{ 	
		  measurement();
  		 if(measurement_success)
  		 {
  		 	
        
  			if(mindistance<measured_distance && measured_distance<maxdistance)
  			{
			  return measured_distance;			  
  			}
  			if(measured_distance>maxdistance)
  			{
  				return maxdistance;
  			}
  			if(measured_distance<mindistance)
  		    {
  		    	return mindistance;
  		    }
        }
  	else
  	{ 		 
		//   Serial.println("Measurement Failed");			
  		//return 0;
	  }
       } 
	
};


class ultra_sensor_adv : public ultra_sensor{
public:
	void measurement() {
		   Serial.println("Attempt measuremnt");		
			  int measure1, measure2;
			  int waittime = 100;			  
			  ultra_sensor::measurement();
			  measure1 = measured_distance;
			  delay(waittime);
			  ultra_sensor::measurement();
			  measure2 = measured_distance;
			  measured_distance = ((measure1 + measure2)/2) ;	// getting the average of two measurements		 
		}
};


#endif
