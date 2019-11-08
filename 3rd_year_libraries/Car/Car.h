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
		basic_speed_PID1 PID,PID1;
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
		controller1 C,C1,C2;
		unsigned long time_interval=350; //time interval to check buttons
		unsigned long time_interval1=100; //task
		unsigned long time_interval2=500;//sensor
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
	ultra.setup_sensor(trig,Echo); 
	motor.setup_motor(motorpin,directionpin);   //Setup motor
	motor1.setup_motor(motorpin1,directionpin1);
	C.setup_controller(time_interval);   //Time checker for buttons
	C1.setup_controller(time_interval1); // Time checker for potentiometer 
	C2.setup_controller(time_interval2); //Time checker for Hall effect sensor to get RPM
	sense.setup_sensing(int_0,number_of_magnets); // Setup for Sensor and potentiometer
	sense1.setup_sensing(int_1,number_of_magnets);
     
	// Setup for buttons
	in_push_button start_but(startpin_input,switch_on); // button to turn on the motor and set the target speed to 30 percent
	in_push_button stop_but(stoppin_input,switch_off);
 
  
	// Adds buttons

	motor_pushbuttons.add_in_push_button(start_but);
	motor_pushbuttons.add_in_push_button(stop_but);
    
	PID.basic_speed_PID(kp,ki,kp,PID_out_min,PID_out_max,time_interval2);
	PID1.basic_speed_PID(kp1,ki1,kd1,PID_out_min,PID_out_max, time_interval2);

	const int IR_D0_pin_left = 20;
	const int IR_D0_pin_right = 21;
	pinMode(IR_D0_pin_left, INPUT);
	attachInterrupt(digitalPinToInterrupt(IR_D0_pin_left), border_detector_int_left, CHANGE);
	attachInterrupt(digitalPinToInterrupt(IR_D0_pin_right), border_detector_int_right, CHANGE);

	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(12, OUTPUT);
	digitalWrite(13, HIGH);
	digitalWrite(12, LOW);
	digitalWrite(7, HIGH);
	digitalWrite(8, LOW);
		
  
}

void system_execute() 
{
	if( C.isTimeToTakeMeasurement() )
		{  
			command_list_enum in_smpl_cmd; 
			bool success;
			//success=motor_pushbuttons.check_n_get_command(in_smpl_cmd); //Returns TRUE if the buttons were initialized and sends the label of the button to in_smp_cmd
			success = 1;
			in_smpl_cmd = switch_on;
			if(success)
			{
				//Serial.println("");
				switch (in_smpl_cmd)
				{
				case switch_on:
					//Serial.println("       Start button pressed");
					motor.start();
					motor1.start();
					target_RPM=200;		//what kind of RPM is this? 0-255 or 0-1023?		
				started=true;           
				break;			
				case switch_off:
						Serial.println("      Stop button pressed"); //switch off the motor
						//motor.stopMotor();
						//motor1.stopMotor();
						//started=false;
						rotated=false;	
						break;				
				default:
					Serial.println("Unknown button pressed");
					break;
				}
				C.updateCheckTime();                     
			}					 
	}
	//Check current RPM and update
	/*if (C2.isTimeToTakeMeasurement())
	{
			current_RPM = sense.requestRPM();
			current_RPM1 = sense1.requestRPM();
			//distance=ultra.current_distance();
       //Serial.print("distance = ");
	//	Serial.println(distance);
			C2.updateCheckTime(); 			
	}*/
		
	if (C1.isTimeToTakeMeasurement())
	{
		//Serial.println(duration);
	
		int opp_detected = sanic();
		//turnSegments( distance_array,  direction_array, 4);
		if (edge_detected == 0 && opp_detected == 0)
		{
			forward_dir();
			//driveStraight(tar_speed);
			just_go(tar_speed);
		}

		//else if(opp_detected)
		//	driveStraight(150);

		else if (edge_detected == 1 && opp_detected == 0)
		{
			Serial.println(edge_state);
			switch (edge_state)
			{
				case 0:
					//change direction of motors
					stop_motors();
					backward_dir();
					edge_state = 1;
					reverse_stamp = millis();
					break;

				case 1:
					//move
					//driveStraight(tar_speed); //How long?
					just_go(tar_speed);
					if (millis() - reverse_stamp >= 1000)
						edge_state = 2;
					break;

				case 2:
					//change direction
					forward_dir();
					stop_motors();
					edge_state = 3;
					break;

				case 3:
					//Direction setup
					/*int pick = rand() % (1 + 1 - 0) + 0;
					if (pick % 2 == 0)
						spin_dir_left();
					else
						spin_dir_right();*/
					spin_dir_right();
					edge_state = 4;
					spin_stamp = millis();
					duration = rand() % (max_number + 1 - minimum_number) + minimum_number;

					Serial.println(spin_stamp);
					break;

				case 4:
					//Serial.println(7);
					//edge_detected = 0;
					//Spin
					//driveStraight(tar_speed);
					just_go(tar_speed);
					if (millis() - spin_stamp >= duration)
					{
						forward_dir();
						stop_motors();
						edge_state = 0;
						edge_detected = 0;
					}
					break;
			}
		}
		

		//if (edge_detected == 0)
		//if(opp_detected == 1)
		//	driveStraight(200);
		else
			stop_motors();
		//member_four(350,70,200,35);
		//semiCircle1(20);
		//advance(30,100,20,100,3);
		//C1.printCSV(current_RPM,current_RPM1,0,0);
		//	circleLap(10, 100, 5, 5, 200, 10);		
		//C1.updateCheckTime(); 
		//Serial.println(edge_detected);
		//edge_detected = 0;
		//Serial.println(digitalRead(20));

			
	}	


}

void turn(int direction=0)
{  


	switch(state)
	{
		case idle:
			motor.stopMotor();
            motor1.stopMotor();
            sense.resetDistanceCount();
            sense1.resetDistanceCount();
            
             PID.reset_pidcontrol();
			PID1.reset_pidcontrol();
			if(direction==0&&!rotated)
			{
				state=4;
			}
				if(direction==1&&!rotated)
			{
				state=3;
			}
		   break;
		case rotationC:
			if(!rotated)
			{
				motor.start();
			motor1.start();
		       output1=PID1.ComputePID_output(target_RPM,current_RPM1);				
				output=PID.ComputePID_output(target_RPM,current_RPM);
				motor1.set_speed(output1); 
			    motor.set_speed(output);
			//	Serial.println(sense1.GetDistanceCount());
			if(sense1.check_distance_met(35)) //need to check whether rotated 90 degrees 
          {   	     
   	     rotated=true;	   
   	    directionFW();
   	     state=0;
        	 }
          }
		  break;
		case rotationAC:
			if(!rotated)
			{
			motor.start();
			motor1.start();
				output1=PID1.ComputePID_output(target_RPM,current_RPM1);			
				output=PID.ComputePID_output(target_RPM,current_RPM);
				motor1.set_speed(output1); 
			   motor.set_speed(output);
			   //Serial.println(sense.GetDistanceCount());
			  
			if(sense.check_distance_met(35)) //need to check whether rotated 90 degrees 
          {
   	    
   	     rotated=true;
   	    directionFW();
   	     state=0;
          }
         }
		  break;
		case changeDirAC:
				if(!direction_changed1)
				{
				//	Serial.println("Direction changed");
				motor1.change_direction();
				direction_changed1=true;
				motor1.stopMotor();
				state=2;
		      }
		      else
		       {
		       	motor1.stopMotor();
		      	state=2;
		       }
		     
		 break;	
	     case changeDirA:
			if(!direction_changed)
				{
					//Serial.println("Direction changed");
				motor.change_direction();
				motor.stopMotor();
				direction_changed=true;
				state=1;
		       }
		       else
		       {
		       	motor.stopMotor();
		      	state=1;
		       }
		 break;	
		
	}
}

void driveStraight(int input_speed,bool direction=0)
{
		if(started) //check if the motor is started
			{
				if(direction==0)
				{
				directionFW();
				//target_RPM=input_speed;
				//output=PID.ComputePID_output(input_speed,current_RPM);
				//output1=PID1.ComputePID_output(input_speed,current_RPM1);

				//Serial.print("Output speed: ");
				//Serial.print(output);
				//Serial.print(",");
				//Serial.println(output1);

				Serial.print("Current speed: ");
				Serial.print(current_RPM);
				Serial.print(",");
				Serial.println(current_RPM1);

				//motor.set_speed(output);
				//motor1.set_speed(output1); //Updates PWM in order to change the speed according to potentiometer value
				int sped = 100;
				motor.set_speed(input_speed);
				motor1.set_speed(input_speed);
			 //C1.printCSV(current_RPM,current_RPM1,0,0);
			}
			else{
		
				
				if(direction_changed==true)		
	            {
				//target_RPM=input_speed;
				//output=PID.ComputePID_output(input_speed,current_RPM);
				//output1=PID1.ComputePID_output(input_speed,current_RPM1);

				int sped = 100;
				motor.set_speed(input_speed);
				motor1.set_speed(input_speed);
			}
			else
			{
			motor.change_direction();
				motor1.change_direction();
				direction_changed=true;		
			}
		}
			}
			
		/*	if(!started&&rotated)
			{
				PID.reset_pidcontrol();
				PID1.reset_pidcontrol();
			}*/
			
}
void directionFW()
{
	if(direction_changed)
	{
	//Serial.println("aaa");
			motor.change_direction();
			direction_changed=false;
			
	}
	if(direction_changed1)
	{
	//	Serial.println("aaa");
			motor1.change_direction();
			direction_changed1=false;
	}
	forwards=true;
}

void just_go(int input_speed)
{
	motor.set_speed(input_speed);
	motor1.set_speed(input_speed);
}

void stop_motors()
{
	motor.stopMotor();
	motor1.stopMotor();
}

int sanic()
{
	long duration, cm;
	int pingPin = 5;
	int echoPin = 6;
	pinMode(pingPin, OUTPUT);
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pingPin, LOW);
	pinMode(echoPin, INPUT);
	duration = pulseIn(echoPin, HIGH);
	cm = duration / 29 / 2;
	//Serial.println(cm);

	if (cm < 5)
		return 1;
	else
		return 0;
}

void forward_dir()
{
	digitalWrite(13, LOW);
	digitalWrite(12, HIGH);
	digitalWrite(7, HIGH);
	digitalWrite(8, LOW);
}

void backward_dir()
{
	digitalWrite(13, HIGH);
	digitalWrite(12, LOW);
	digitalWrite(7, LOW);
	digitalWrite(8, HIGH);
}

void spin_dir_left()
{
	digitalWrite(13, HIGH);
	digitalWrite(12, LOW);
	digitalWrite(7, LOW);
	digitalWrite(8, LOW);
}

void spin_dir_right()
{
	digitalWrite(13, LOW);
	digitalWrite(12, LOW);
	digitalWrite(7, LOW);
	digitalWrite(8, HIGH);
}

//BASIC TASK OPTIONAL
void straight_and_reverse(int target_distance_fwd, int target_distance_bwd)
{
	driveStraight(target_RPM);
    if (sense.check_distance_met(target_distance_fwd) & car_reversed == 0)
    {
        if (time_interval1*check_intervals_passed >= 1000)
        {
		sense1.resetDistanceCount();
        	sense.resetDistanceCount();
        	
            motor.change_direction();
            motor1.change_direction();
            car_reversed = 1;
        }
        else
        {
            check_intervals_passed++;
            motor.stopMotor();
            motor1.stopMotor();
        }
    }
    else if (sense.check_distance_met(target_distance_bwd) & car_reversed == 1)
    {
        motor.stopMotor();
        motor1.stopMotor();
    }
    
}
void turnSegments(int distance_array1[10], int direction_array1[10], int number_segments)
{
   	switch (segments_state)
	{
		case forward :
		driveStraight(target_RPM);
		Serial.println("FORWARD");
		if (sense.GetDistanceCount() >= distance_array[current_segment])
		{
			
			if (direction_array[current_segment] == 1)
			{
				//sense.resetDistanceCount();
				segments_state = left_turn;
				
			}
			else
			{
				//sense1.resetDistanceCount();
				segments_state = right_turn;
				
			}
			current_segment++;
		}
		 if(current_segment == number_segments)
			segments_state = stop;
		break;
		case left_turn:
			Serial.println("LEFT TURN");
		turn(1);
		if (rotated)
		{
			//sense1.resetDistanceCount();
		//	sense.resetDistanceCount();
			segments_state = forward;
			rotated=false;
		}
		break;
		case right_turn:
			Serial.println("RIGHT TURN");
		turn(0);
		if (rotated)
		{
			//sense1.resetDistanceCount();
		//	sense.resetDistanceCount();
			segments_state = forward;
				rotated=false;
		}
		break;
		case stop:
			
			 motor.stopMotor();
        motor1.stopMotor();
			break;
	}
}

void member_four(int target_speed, int farDistance,int lowerSpeed, int closeDistance)
{
	 switch(state3)
				 {
				 
				    case drive_normal_speed:
				    	rotated=false;
				    	
				    	Serial.println("drive");
				        driveStraight(target_speed);
				    
				      state3=check_1;
				       break;
				    case drive_slower:
				    	rotated=false;
				    	target_RPM=lowerSpeed;
				    	Serial.println("drive slower");
				        driveStraight(lowerSpeed);
				        state3=check_1;
				    	break;
				    case check_1:
				    	Serial.println("check1");
				    	 if(distance<=closeDistance)
				          {			      			
				     	   	  
				            state3=turnR;
				           }
				           else if(distance<=farDistance&&distance>closeDistance)
				              {
				                   state3=drive_slower;	
				              }    
				           else
				           {
				        
				           	state3=drive_normal_speed;
				           }
				    	break;
				   	case check_2:
				    		rotated=false;
				    		Serial.println("check2");
				    		 if(distance<=closeDistance)
				            {				     	   	  
				            state3=turnL;
				            }
				            else if(distance<=farDistance&&distance>closeDistance)
				            {
				            	times_turned=0;
				            	state3=drive_slower;
				            }
				            else
				            {
				            	times_turned=0;
				           	state3=drive_normal_speed;
				             }
				    		break;
				      
					  case turnR:
				       	Serial.println("turnR");
				        	turn(0);
				      	if(rotated)
				     	 {				 	
				 	   
				     	state3=check_2;				 
				 	
				           }
				  	      break;
				 	case turnL:
				 		Serial.println("turnL");
				 		turn(1);
				 		
				 		if(rotated)
				 		{			            
				       times_turned++;
				       Serial.print("times turned = ");
				       Serial.print(times_turned);
				       if(times_turned==2)
				       {
				       	state3=check_2;
				       }
				       if(times_turned==3)
				       {
				       	times_turned=0;
				       	state3=drive_normal_speed;
				       	
				       }
				       if(times_turned==1)
					    {
				              state3=idle1;
				        }
				        
				       }
				 		break;
				 	case idle1:
				 			Serial.println("idle");
				 			rotated=false;
				 			state3=turnL;
				 			break;
				 		
				 }
}

void semiCircle1(double radius) {
    double ratio, radius1, radius2, RPM2, RPM1;
    double N1, N2,N;
    radius1 = radius + 5;
    radius2 = radius - 5;
    ratio = radius2 / radius1;
    RPM1 = target_RPM;
    RPM2 =  RPM1*ratio;
   
    N1 = (radius1 / (5.7/2))*49.0;
    N2 = (radius2 / (5.7/2)) *49.0;
    
    output = PID.ComputePID_output(RPM2, current_RPM);
    output1 = PID1.ComputePID_output(RPM1, current_RPM1);
    motor.set_speed(output);
    motor1.set_speed(output1);
 
  // Serial.println(sense1.GetDistanceCount());
    if (sense1.check_distance_met((N2/2))) 
	{
        //motor.stopMotor();
       // motor1.stopMotor();
        //sense.resetDistanceCount();
        //sense1.resetDistanceCount();
            
        semiCircle_Finished=true;
    }
}
   
void advance(int distanceStraight,int straight_speed,double radius,int turn_speed,int number_laps)
{

Serial.println("LAPS");
Serial.println(laps_counter);
	if(laps_counter==2*number_laps)
	{
		motor.stopMotor();
		motor1.stopMotor();
	}
	else
	{	
		switch(advanced_S)
	{
		case GoStraight:
			
		
			driveStraight(straight_speed);
			distanceStraight=distanceStraight*49/(3.14*5.7);// convert from centimetres to nuber of pulses of the magnets
			if(sense.check_distance_met(distanceStraight))
			{
				Serial.println("Straight path over");
				laps_counter++;
			//	
				PID.reset_pidcontrol();
				PID1.reset_pidcontrol();
				sense.resetDistanceCount();
        sense1.resetDistanceCount();
			advanced_S=semiCircle;
			}
			break;
		case semiCircle:
			target_RPM=turn_speed;
				semiCircle1(radius);
				if(semiCircle_Finished)
				{
					Serial.println("Circular path over");
						PID.reset_pidcontrol();
				PID1.reset_pidcontrol();
				sense.resetDistanceCount();
        sense1.resetDistanceCount();
					advanced_S=GoStraight;
					semiCircle_Finished=false;
				}
				break;		
	}
}	
}

void circleLap(double radii, double speed, int numberLap1, double radii2, double speed2, int numberLap2) {
	double radii_outer;
	double radii_inner;
	double ratio;
	double set_RPM, set_RPM1;
	double set_RPMAC, set_RPMAC1;
	int n;
	int dist, total_dist, distA, total_distA;

	if (numberLap1 > 10 || numberLap2 > 10) {
		Serial.print("This car cannot go more than 10 laps.");
		motor.stopMotor();
		motor1.stopMotor();
	}
	else {
		switch (circleState)
		{
		case clockwise:
			radii_outer = radii + 5;
			radii_inner = radii - 5;
			ratio = radii_outer / radii_inner;
			set_RPM = speed;
			set_RPM1 = ratio * set_RPM;
			dist = (4.0*PI*radii_inner / (PI*5.7))*49.0;
			total_dist = (4.0*PI*radii_inner / (PI*5.7))*49.0 * numberLap1; // distance travelled by inner motor
			output = PID.ComputePID_output(set_RPM, current_RPM);
			output1 = PID1.ComputePID_output(set_RPM1, current_RPM1);
			motor.set_speed(output);
			motor1.set_speed(output1);
			for (n = 1; n < numberLap1; n++) {
				if (sense.check_distance_met(dist*n)) {
					Serial.print("Lap  ");
					Serial.print(n);
					Serial.print(" completed");
				}
			}
			if (sense.check_distance_met(total_dist)) {
				motor.stopMotor();
				motor1.stopMotor();
			}

			circleState = changeDirection;

			break;
		case changeDirection:
			sense.resetDistanceCount();
			sense1.resetDistanceCount();

			motor.change_direction();
			motor1.change_direction();

			circleState = anticlockwise;
			break;

		case anticlockwise:

			radii_outer = radii2 + 5;
			radii_inner = radii2 - 5;
			ratio = radii_outer / radii_inner;
			set_RPMAC = speed2;
			set_RPMAC1 = ratio * set_RPMAC;
			distA = (4.0*PI*radii_inner / (PI*5.7))*49.0;
			total_distA = (4.0*PI*radii_inner / (PI*5.7))*49.0 * numberLap2;
			output = PID.ComputePID_output(set_RPMAC, current_RPM);
			output1 = PID1.ComputePID_output(set_RPMAC1, current_RPM1);
			motor.set_speed(output);
			motor1.set_speed(output1);

			for (n = 1; n < numberLap2; n++) {
				if (sense.check_distance_met(dist*n)) {
					Serial.print("Lap  ");
					Serial.print(n);
					Serial.print(" completed");
				}
			}
			if (sense.check_distance_met(total_dist)) {
				motor.stopMotor();
				motor1.stopMotor();
			}
			break;
		}
	}


}
};
#endif
