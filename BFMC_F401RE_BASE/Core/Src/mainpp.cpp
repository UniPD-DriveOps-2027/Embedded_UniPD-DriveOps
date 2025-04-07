/*
 * mainpp.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: eugen
 */

#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "pid.h"
#include "imu.h"
//#include "pid_controller.h"

#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

// Constants
#define TRANSMISSION_RATIO 		16				// both wheels placed on the ground are rotating at 1/16th of the motor's speed
#define STEER_DEG2PWM_RATIO 	0.0009505		// [deg] -> [pwm]
#define STEER_DEG2PWM_OFFSET	0.07620			// [pwm]
#define MOTOR_SPEED_OFFSET 		0.075568        // [pwm]: The zero default where motors are stop
#define M_PPI					6.28318530718	//	2*pi
#define WHEEL_RADIUS			0.0335// [m]
#define MAX_FORWARD_SPEED		1.0				// [m/s]
#define MAX_BACKWARD_SPEED		-0.5			// [m/s]
#define MAX_FORWARD_PWM_SPEED   0.009
#define MAX_BACKWARD_PWM_SPEED  0.0119
#define MAX_STEERING_ANGLE		28				// [deg]
#define MOT_DECELERATION		-2				// [m/ss]
#define MOT_ACCELERATION		2				// [m/ss]

//#define SPEED_FFW				0				//
#define EPSILON					0.01			//
//#define EPSILON_DIFF			0.001			//

// timers defined in main.c so get them as extern
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

extern ADC_HandleTypeDef hadc1;


ros::NodeHandle nh;
// REFERENCE variables
double speedTarget=0.0, speedTargetPos=0.0, speedRef=0.0; // [m/s], [m/s]: speed references
double steerRef=0.0;// [deg]: steer references
// MEASUREMENT variables
double wheelSpeed=0.0, motorSpeed=0.0, carSpeed=0.0;// [rps], [rps], [m/s]: speed measurements
double globalDistance=0.0, localDistance=0.0, localDistanceOrigin=0.0, localDistanceRef=0.0;// [m]: position measurements
long encoderCount=0;// [tick]: encoder count
long encoderTotal=0;
// CONTROL variables
double motorPWM=0.0, servoPWM=0.0;// [tick], [tick]: control variables
// STATE of the CAR
bool isSpeedControl 	= true;
bool isPositionControl 	= false;
bool posAckDone=false, reachedPosition=false, pub_flag=false;

// ADC
//uint16_t adcValue = 0;
float adcValue = 0;


// DEBUGGING
uint8_t counter;
float input,output,setpoint;
float KP=0.020001, KI=0.025001, KD=0.0;
uint16_t pFilterWindow = 3000;
float filtered_p = 0;
float raw_p = 0;

uint8_t samplingPID = (uint8_t)((ENC_COUNT_MAX + 1) / 1e3); // [ms]

// CONTROLLERS PID 			 (Input,     Output,    Setpoint) 0.0075, 0.15, 0.00015
PID SpeedController 	= PID(&carSpeed, &motorPWM, &speedRef, 0.02, 0.025, 0.0 ,_PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error
PID PositionController 	= PID(&localDistance, &speedTargetPos, &localDistanceRef, 6, 0.0, 3,_PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error



// ROS STUFF
std_msgs::Float32 carSpeed_msg;
std_msgs::Float32 globalDistance_msg;
std_msgs::Bool posFeedback_msg;


// command callbacks
void command_speed_callback(const std_msgs::Float32& msg);
void command_steer_callback(const std_msgs::Float32& msg);
void command_stop_callback(const std_msgs::Float32& msg);
void command_pos_callback(const std_msgs::Float32& msg);

void stop(float angle);
void steer(float angle);
void speed(float vel);
void position(float dist);
void drive_pwm(float pwm_value);

bool isClose(double A, double B);
double last_diff=0.0;

float readADC(void);


// publishers and subscribers
ros::Publisher carSpeed_pub("automobile/encoder/speed", &carSpeed_msg);
ros::Publisher globalDistance_pub("automobile/encoder/distance", &globalDistance_msg);
ros::Publisher posFeedback_pub("automobile/feedback/position", &posFeedback_msg);
ros::Subscriber<std_msgs::Float32> speed_sub("automobile/command/speed", &command_speed_callback);
ros::Subscriber<std_msgs::Float32> steer_sub("automobile/command/steer", &command_steer_callback);
ros::Subscriber<std_msgs::Float32> stop_sub("automobile/command/stop", &command_stop_callback);
ros::Subscriber<std_msgs::Float32> pos_sub("automobile/command/position", &command_pos_callback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}

void command_speed_callback(const std_msgs::Float32& msg){
	speed(msg.data);
}

void command_steer_callback(const std_msgs::Float32& msg){
	steer(msg.data);
}

void command_stop_callback(const std_msgs::Float32& msg){
	stop(msg.data);
}

void command_pos_callback(const std_msgs::Float32& msg){
	position(msg.data);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim11){//every 1ms, PSC = 83, ARR = 999, f_tim = 84MHz -> (ARR+1*PSC)/f_TIM = 0.001s = 1ms

		double DT = ((ENC_COUNT_MAX + 1) / 1e6); //0.001s  [s]
		// ---------- ENCODER SPEED ----------
		encoderCount = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);	// (int16_t) cast to [-2^15 ; 2^15]
		encoderTotal = encoderTotal + encoderCount;
		motorSpeed = ((double)encoderCount / 2048.0) / DT; 		// [rps]: motor speed
		wheelSpeed = motorSpeed / TRANSMISSION_RATIO;			// [rps]: wheel speed
		carSpeed = wheelSpeed * M_PPI * WHEEL_RADIUS; 			// [m/s]: carSpeed, i.e. v = ω * R , where angular velocity ω = rps * 2π
		globalDistance = globalDistance + carSpeed * DT;		// [m]: signed distance run by the car, i.e. curvilinear abscissa
		localDistance = globalDistance - localDistanceOrigin;	// [m]: signed relative distance run by the car, i.e. curvilinear abscissa

		// ---------- DOUBLE LOOP CONTROL ----------
		// ---------- OUTER LOOP: POSITION CONTROL - PID ----------
		if(isPositionControl){
			if(isClose(localDistance, localDistanceRef) and not posAckDone){
				reachedPosition = true;
				speedTarget = 0.0;
				isPositionControl = false; //just added

			}
			else{
				reachedPosition = false;
				PositionController.Compute();
				speedTarget = speedTargetPos;
			}
		}

		// ---------- INNER LOOP: SPEED CONTROL - PID ----------
		if(isSpeedControl){

			speedRef = speedTarget;
			SpeedController.Compute();// SPEED control action
		}


		// ---------- ACTUATION - SPEED ----------
		//if(abs(motorPWM)<0.002) motorPWM = 0;

		if(abs(speedRef) == 0.00)
			drive_pwm(0);
		else
			drive_pwm(motorPWM);

		//drive_pwm(speedTarget);

		//for plotting on SWM graph
		//input=(float)carSpeed;
		//output=(float)motorPWM * 100;
		//setpoint=(float)speedRef;
		//filtered_p = SpeedController.GetFilteredProportional();
		//raw_p = SpeedController.GetKp() * (speedRef - carSpeed);

		//input=(float)localDistance;
		//output=(float)speedTargetPos;
		//setpoint=(float)localDistanceRef;

		// ---------- RESET ENCODER COUNTER ----------
		__HAL_TIM_SET_COUNTER(&htim4,0);

	}

	if(htim == &htim10){	//every 10ms
		pub_flag = true;
	}

	/*
	if (htim->Instance == TIM1) { // PRESCALER * COUNTER / CLOCK_FREQ = 0.01 s = 10 ms
		imuTimerCallback();
	}
	*/

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// ---------- USER BUTTON EMERGENCY BRAKE (BLUE BUTTON)----------
	if(GPIO_Pin == B1_Pin){

		stop(0.0);

		// Used for scalling the Graph in debugging, press twice the blue button so it gets scalled
		/*
		if (counter % 2 == 0){
			input=-1;
			output=-1;
			setpoint=-1;
		}
		else{
			input=1;
			output=1;
			setpoint=1;
		}
		counter++;
		 */

		//Update PID gains
		//printf("KP = %f , KI = %f , KD = %f \r\n",KP,KI,KD);
		//SpeedController.EnableProportionalFilter(true);
		//SpeedController.SetTunings((double)KP,(double)KI,(double)KD);
		//SpeedController.SetProportionalFilterWindow(pFilterWindow);
		//SpeedController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
		//SpeedController.SetOutputLimits(-0.009, 0.0119);// PWM Output limits
		//SpeedController.SetSampleTime(samplingPID);// set sample time in [ms]

		//PositionController.SetTunings((double)KP,(double)KI,(double)KD);
		//PositionController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
		//PositionController.SetOutputLimits(-0.2, 0.2);// PWM Output limits
		//PositionController.SetSampleTime(samplingPID);// set sample time in [ms]

	}
}

void setup(void)
{
	// Initialize the node ...
	nh.initNode();
	// ... advertise publishers ...
	nh.advertise(carSpeed_pub);
	nh.advertise(globalDistance_pub);
	nh.advertise(posFeedback_pub);

	// ... and declare subscribers
	nh.subscribe(speed_sub);
	nh.subscribe(steer_sub);
	nh.subscribe(stop_sub);
	nh.subscribe(pos_sub);

	// Set PID SPEED controller parameters
	SpeedController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
	SpeedController.SetOutputLimits(-0.009, 0.0119);// PWM Output limits
	SpeedController.SetSampleTime(samplingPID);// set sample time in [ms]
	SpeedController.EnableProportionalFilter(true);
	//SpeedController.SetProportionalFilterWindow(0);

	// Set PID POSITION controller parameters
	PositionController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
	PositionController.SetOutputLimits(-0.2, 0.2);// Speed Output limits [m/s]
	PositionController.SetSampleTime(samplingPID);// set sample time in [ms]
	PositionController.EnableProportionalFilter(false);

	// Start TIMERS
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);// DC MOTOR PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);// SERVO PWM
	HAL_TIM_Base_Start(&htim4);// ENCODER COUNTER
	HAL_TIM_Base_Start_IT(&htim11);// ENCODER SPEED AND SPEED CONTROL
	HAL_TIM_Base_Start_IT(&htim10);// PUBLISH TIMER

	// stop the car
	speedTarget = 0.0;
	speedRef 	= 0.0;
	steerRef 	= 0.0;
	//added
	steer(0);
	drive_pwm(0);

}

void loop(void)
{
	// read line detection sensor
	//adcValue = readADC();

	if (nh.connected())
	{
		if (pub_flag)
		{
			// ---------- PUBLISH ----------
			carSpeed_msg.data = carSpeed;
			carSpeed_pub.publish(&carSpeed_msg);

			globalDistance_msg.data = globalDistance;
			globalDistance_pub.publish(&globalDistance_msg);

			if (reachedPosition and not posAckDone)
			{
				posFeedback_msg.data = reachedPosition;
				posFeedback_pub.publish(&posFeedback_msg);
				posAckDone = true;

			}

			pub_flag = false;
		}
	}
	HAL_Delay(1);
	nh.spinOnce();
}


void stop(float angle){
	// ---------- STATE ----------
	isPositionControl 	= false;
	isSpeedControl 		= false;

	// ---------- STOP references ----------
	speedTarget = 0.0;
	speedTargetPos = 0.0;
	speedRef = 0.0;

	// ---------- STOP ----------
	motorPWM = 0.0;
	steer(angle);
}

void speed(float vel){
	// ---------- SPEED references ----------
	if(vel > MAX_FORWARD_SPEED){
		speedTarget = MAX_FORWARD_SPEED;
	}
	else if (vel < MAX_BACKWARD_SPEED){
		speedTarget = MAX_BACKWARD_SPEED;
	}
	else{
		speedTarget = vel;
	}

	// ---------- STATE ----------
	isSpeedControl 		= true;
	isPositionControl 	= false;

}

void steer(float angle){

	// ---------- STEER references ----------
	if(angle > MAX_STEERING_ANGLE){steerRef = MAX_STEERING_ANGLE;}
	else if (angle < -MAX_STEERING_ANGLE){steerRef = -MAX_STEERING_ANGLE;}
	else{steerRef = angle;}

	// ---------- STEER ----------
	servoPWM = STEER_DEG2PWM_RATIO * steerRef + STEER_DEG2PWM_OFFSET;			// [deg] -> [pwm]
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servoPWM*SERVO_PWM_COUNT_MAX);	// set servo pwm
}

void position(float dist){
	// ---------- POSITION references ----------
	localDistanceOrigin = globalDistance;
	localDistanceRef = dist;

	// ---------- STATE ----------
	isSpeedControl = true;
	isPositionControl = true;
	posAckDone = false;
	reachedPosition = false;
}

void drive_pwm(float pwm_value){
	// ---------- ACTUATION - SPEED ----------

	// is on reverse
	pwm_value = -pwm_value;

	// Range pwm_value - values are calibrated to reach a max of 1.0 m/s forward and 0.5 backwards
	if (pwm_value > 0.009f) pwm_value = 0.009f; 	//Backwards
	if (pwm_value < -0.0119f) pwm_value = -0.0119f; //Forward

	// considering the default zero speed is at 7.5568% duty cycle
	float duty_cycle = (pwm_value + MOTOR_SPEED_OFFSET);

	// Convert duty cycle to pulse width and set the PWM value
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(duty_cycle * MOT_PWM_COUNT_MAX));
}

bool isClose(double A, double B)
{
	double diff = A - B;
	return (abs(diff) < 0.001) ;//&& (abs(last_diff-diff) < EPSILON_DIFF) ;
	//last_diff = diff;
}

float readADC(void)
{
	uint32_t  adc_value;
	float scaled_value;

	HAL_ADC_Start(&hadc1); // Start ADC conversion
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	{ // Wait for conversion
		adc_value = HAL_ADC_GetValue(&hadc1); // Read ADC value
		scaled_value = (float)adc_value;// / 7358.54; // Scale ADC value
	}
	HAL_ADC_Stop(&hadc1); // Stop ADC conversion

	return scaled_value;
}
