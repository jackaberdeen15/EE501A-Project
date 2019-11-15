// Implemented by Augustinas Zinys 51550490
// This code executes the main task
#ifndef Car
#define Car


#include <stdlib.h> 

#define PI 3.14159265

#include "..\Experiment1Control\Experiment1Control.h"
#include "..\sensor\sensor.h"
#include "..\Act2_motor_action\Act2_motor_action.h"
#include "..\PushButton\PushButton.h"
#include "..\basic_speed_PID\basic_speed_PID.h"
#include "..\Sensor_File\Sensor_File.h"


enum turnSegments_state_enum {
	forward, left_turn, right_turn,stop
};
enum advanced_state {
	GoStraight, semiCircle
};
enum avoidance {
	drive_normal_speed, drive_slower,idle1,turnR,turnL,check_1,check_2
};
enum avoidance_enum {
    normal, check1, turn_clockwise, turn_anticlockwise_segment_one, turn_anticlockwise_segment_two, check2, turn_anticlockwise_segment_three
};

volatile int edge_detected = 0;
void border_detector_int_left()
{
	if (digitalRead(20) == 0) {
		edge_detected = 1;
	}
	//else {
	//	edge_detected = 0;
	//}
}

void border_detector_int_right()
{
	if (digitalRead(21) == 0) {
		edge_detected = 1;
	}
	//else {
	//	edge_detected = 0;
	//}
}


class system2{
	protected:
		
		int circleState = 0;
		#define clockwise 0
		#define changeDirection 1	
		#define anticlockwise 2
		
		int car_reversed;
		int check_intervals_passed;
		int current_segment;
		int distance_array[10]={50,50,50,50};
		int direction_array[10]={0,1,0,1};
		turnSegments_state_enum segments_state;
		avoidance state3;
		avoidance_enum avoidance_state;
		int times_turned;
		advanced_state advanced_S;

		#define idle 0
		#define rotationC 1
		#define rotationAC 2
		#define changeDirAC 3
		#define changeDirA 4
		int state;
		
		//BLUE  , Right motor
		basic_speed_PID PID,PID1;
		double kp=0.28;                  // * (P)roportional Tuning Parameter
		double ki=0.056;                  // * (I)ntegral Tuning Parameter
		double kd=0;
				//RED line ,left motor
		//double kp1=0.25;                 // * (P)roportional Tuning Parameter
		//double ki1=0.051;                  // * (I)ntegral Tuning Parameter
		//double kd1=0;                // * (D)erivative Tuning Parameter

		double kp1 = 0.28;                 // * (P)roportional Tuning Parameter
		double ki1 = 0.056;                  // * (I)ntegral Tuning Parameter
		double kd1 = 0;                // * (D)erivative Tuning Parameter


		double PID_out_min=0; //MIN PID output
		double PID_out_max=255; //MAX PID output
  
		double target_RPM;
		double output; //PWM
		double 	output1;
		double current_RPM;
		double current_RPM1;
		bool started;  // to check if the motor is started 
		bool direction_changed=false; // check if the direction of the wheel has changed    
		bool direction_changed1=false;
		bool forwards;
		bool semiCircle_Finished;


		inputs motor_pushbuttons;
		Motor_Action motor,motor1;
		sensor sense,sense1;
		controller1 c_general,C1,c_hall;
		unsigned long time_interval=350; //time interval to check buttons
		unsigned long time_interval1=100; //task
		unsigned long time_interval2=100;//sensor
		unsigned long initial_stable_time;
  
		int laps_counter;
		bool rotated;
  
		ultra_sensor ultra;
		int distance;


		int max_number = 8000;
		int minimum_number = 5000;
		unsigned long duration = rand() % (max_number + 1 - minimum_number) + minimum_number;
		//int duration = 3000;

		unsigned long spin_stamp = 0;
		unsigned long reverse_stamp = 0;
		int edge_state = 0;

		int tar_speed = 230;


  
public:

void system_setup(int motorpin,int motorpin1,int directionpin, int pin_interrupt,int pin_interrupt1, int number_of_magnets,int directionpin1, int startpin_input, int stoppin_input,int Echo,int trig)
{ 
	motor.setup_motor(motorpin,directionpin);   //Setup motor
	motor1.setup_motor(motorpin1,directionpin1);
	c_hall.setup_controller(time_interval2); //Time checker for Hall effect sensor to get RPM
	sense.setup_sensing(pin_interrupt,number_of_magnets); // Setup for Sensor and potentiometer
	sense1.setup_sensing(pin_interrupt1,number_of_magnets);
		
  
}

void system_init()
{
    motor.start();
    motor1.start();
    target_RPM=200;		//what kind of RPM is this? 0-255 or 0-1023?
    motor.set_speed(target_RPM);
    motor1.set_speed(target_RPM);
    Serial.print("start: ");
    Serial.println(motor.isStarted());
}

void system_execute() 
{
    
	//Check current RPM and update
	if (c_hall.isTimeToTakeMeasurement())
	{
			current_RPM = sense.requestRPM();
			current_RPM1 = sense1.requestRPM();
			Serial.print("m1 ");
            Serial.println(current_RPM);
			c_hall.updateCheckTime(); 			
	}
	
   

}

#endif
