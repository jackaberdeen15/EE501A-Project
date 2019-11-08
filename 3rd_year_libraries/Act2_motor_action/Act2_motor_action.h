//Implemented by Augustinas Zinys 51550490
//This header file execetues the main task given to control the motor (stop,start,reverse)
#ifndef Motor
#define Motor
#include "..\DCmotor\DCmotor.h"
class Motor_Action{
	protected:
	HBridgeDCmotor motor1;	
		
	public:
	
	void setup_motor(int motor_pin,int direction_pin)
	{
		 
    motor1.setup_HBridgeDCmotor(motor_pin,direction_pin);
    
		}
	void stopMotor()
	{
		motor1.stop();
	}
	void start()
	{
		motor1.start();
	}
	void change_direction()
	{
		motor1.changedir();
	}
	void set_speed(int PWM)
	{
		motor1.setSpeedPWM(PWM);
	}
	int get_PWM()
	{
		 motor1.getSpeedPWM();
	}
	bool isStarted()
	{
			motor1.isStarted();
			
	}
};


#endif
