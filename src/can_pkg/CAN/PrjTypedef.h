/*
 * PrjTypedef.h
 *
 *  Created on: Feb 9, 2024
 *      Author: najee
 */

#ifndef INC_PRJTYPEDEF_H_
#define INC_PRJTYPEDEF_H_

#include "stdbool.h"


typedef enum
{
  STATUS_OK       = 0x00U,
  STATUS_ERROR    = 0x01U,
  STATUS_BUSY     = 0x02U,
  STATUS_TIMEOUT  = 0x03U
} STATUS_TypeDef;

/*
  Wheel Front Right  = 0x00
  Wheel Front Left   = 0x01
  Wheel Rear Right   = 0x02
  Wheel Rear Left    = 0x03
*/

#define BOARDNUMBER 0x02




typedef enum {
	Ackerman =0x00,
	Crab_mode=0x01
}SteeringMode;

typedef struct
{
	float Wheel_Driving_Velocity;
	float Wheel_Driving_Velocity_Sample_Time;

	float Wheel_Driving_Torque;
	float Wheel_Driving_Torque_Sample_time;
} Car_Driving_Parameter;

typedef struct
{
	float Wheel_Steering_Angle;
	float Wheel_Steering_Angle_Sample_Time;

	float Wheel_Steering_Angular_Velocity;
	float Wheel_Steering_Angular_Velocity_Sample_Time;
	SteeringMode SteeringMode;

} Car_Steering_Parameter;

typedef struct
{
	float Wheel_Brake_Torque;
	float Wheel_Brake_Torque_Sample_Time;
	bool  Wheel_Brake_status;
} Car_Brake_Parameter;

typedef struct
{
	float BatteryVoltage;
	float BatteryCurrent;
	uint8_t BatteryChargePercentage;
	uint8_t BatteryStatus;
} Car_PDU_Battery_Parameter;

typedef struct
{
	Car_PDU_Battery_Parameter BatteryStateOfCharge;
} Car_PDU_Parameter;

typedef struct
{
	Car_Driving_Parameter   Car_Driving;
	Car_Steering_Parameter  Car_Steering;
	Car_Brake_Parameter 	Car_Brake;
	Car_PDU_Parameter		Car_PDU;
} Wheels;




typedef struct
{
float Acceleration_X;
float Acceleration_Y;
float Acceleration_Z;
} Acceleration;

typedef struct
{
float Velocity_X;
float Velocity_Y;
float Velocity_Z;
} Velocity;

typedef struct
{
float AngularVelocity_X;
float AngularVelocity_Y;
float AngularVelocity_Z;
} AngularVelocity;

typedef struct
{
float Angle_X;
float Angle_Y;
float Angle_Z;
} Angle;


typedef struct
{
	Acceleration Acceleration;
	Velocity Velocity;
	AngularVelocity AngularVelocity;
	Angle Angle;
} IMUValues;


typedef struct
{
	float AngleFront;
	float AngularVelocityFront;
	float AngleRear;
	float AngularVelocityRear;
} SteeringCommands;

typedef struct
{
	float BrakeTorqueFrontRight;
	float BrakeTorqueFrontLeft;
	float BrakeTorqueRearRight;
	float BrakeTorqueRearLeft;
} BrakeCommands;

typedef struct
{
	float UltraSonicFrontRight;
	float UltraSonicFrontLeft;
	float UltraSonicRearRight;
	float UltraSonicRearLeft;
} UltraSonic;

typedef struct
{
	float DesiredVelocity;
	float DesiredAcceleration;
	float DesiredSteeringAngle;
	float DesiredSteeringAngularVelocity;
	float DesiredBrake;
	SteeringMode DesiredSteeringMode;

} DesiredDrivingParameters;

typedef struct
{
	IMUValues IMUSensor;
	SteeringCommands SteeringCommand;
	BrakeCommands  BrakeCommand;
	UltraSonic UltraSonicSensors;
	Wheels WheelFrontRight;
	Wheels WheelFrontLeft;
	Wheels WheelRearRight;
	Wheels WheelRearLeft;
	Car_PDU_Parameter Car_PDU;
	DesiredDrivingParameters DesiredCarParameters;
	bool AutonomousMode;
} Car_Satus;



#endif /* INC_PRJTYPEDEF_H_ */
