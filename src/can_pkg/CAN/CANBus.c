#include "CANBus.h"

Car_Satus Vehicle;
uint8_t CANAknowledgeFlag = 0;
uint8_t MessageNumberSent = 0x00;

/*
STATUS_TypeDef SendCANMessageWithRetries(CAN_HandleTypeDef *Can, CANMessage Message, uint32_t timeout, uint8_t trials) {
	STATUS_TypeDef status;
	uint32_t tickstart = HAL_GetTick();
	uint8_t attempts = 0;
	uint8_t tempBoardNumber = 0x00;
	uint8_t tempStatus = 0x00;
	DecodeCANID(Message.CANMessgeID, &tempBoardNumber, &MessageNumberSent, &tempStatus);
	do {
		status = SendCANMessage(Can, Message);

		if (status == STATUS_OK) {
			return HAL_OK;
		} else {
			HAL_Delay(10);
			attempts++;
		}
	} while (((HAL_GetTick() - tickstart < timeout) && (attempts < trials)) || (CANAknowledgeFlag == 0));

	return STATUS_TIMEOUT;
}
*/
STATUS_TypeDef SendCANMessage(CANMessage Message)
{

	char inject_id[25]; // Buffer for 8 chars + null terminator

	sprintf(inject_id, "%x", Message.CANMessgeID);

	if (inject_data_frame(inject_id, Message.CANMessageData, Message.CANMessageLength) != -1)
		return STATUS_OK;
	else
		return STATUS_ERROR;
}

STATUS_TypeDef ReceiveCANMessage(unsigned char *Data, int *len)
{
	int lengthData = frame_recv(Data);
	*len = lengthData;
	if (lengthData != 0 && lengthData != -1)
		return STATUS_OK;
	
	return STATUS_ERROR;
}

STATUS_TypeDef ParserCANMessage(uint16_t ID, unsigned char *RxData)
{
	uint8_t BoardNumber;
	uint8_t Status;
	uint8_t MessageNumber;
	DecodeCANID(ID, &BoardNumber, &MessageNumber, &Status);
	if (Status == 0x1 || Status == 0x02)
		SendAcknowledgmentMessage(MessageNumber);

	switch (MessageNumber)
	{
	case MRCU_ACKNOWLEDGE:
		if (BOARDNUMBER == RxData[0] && MessageNumberSent == RxData[1])
			CANAknowledgeFlag = 1;
		break;
	case MRCU_WHEELS_VELCITY:
		Vehicle.WheelFrontRight.Car_Driving.Wheel_Driving_Velocity = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontLeft.Car_Driving.Wheel_Driving_Velocity = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearRight.Car_Driving.Wheel_Driving_Velocity = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Driving.Wheel_Driving_Velocity = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_WHEELS_TORQUE:
		Vehicle.WheelFrontRight.Car_Driving.Wheel_Driving_Torque = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontLeft.Car_Driving.Wheel_Driving_Torque = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearRight.Car_Driving.Wheel_Driving_Torque = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Driving.Wheel_Driving_Torque = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_IMU_ACCERLERATION:
		Vehicle.IMUSensor.Acceleration.Acceleration_X = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.IMUSensor.Acceleration.Acceleration_Y = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.IMUSensor.Acceleration.Acceleration_Z = float_decode(RxData[4] << 8 | RxData[5]);
		break;
	case MRCU_IMU_VELOCITY:
		Vehicle.IMUSensor.Velocity.Velocity_X = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.IMUSensor.Velocity.Velocity_Y = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.IMUSensor.Velocity.Velocity_Z = float_decode(RxData[4] << 8 | RxData[5]);
		break;
	case MRCU_IMU_ANGULAR_VELOCITY:
		Vehicle.IMUSensor.AngularVelocity.AngularVelocity_X = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Y = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Z = float_decode(RxData[4] << 8 | RxData[5]);
		break;
	case MRCU_IMU_ANGLE:
		Vehicle.IMUSensor.Angle.Angle_X = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.IMUSensor.Angle.Angle_Y = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.IMUSensor.Angle.Angle_Z = float_decode(RxData[4] << 8 | RxData[5]);
		break;

	case MRCU_STEERING_COMMAND:
		Vehicle.SteeringCommand.AngleFront = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.SteeringCommand.AngularVelocityFront = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.SteeringCommand.AngleRear = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.SteeringCommand.AngularVelocityRear = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_BRAKING_COMMAND:
		Vehicle.BrakeCommand.BrakeTorqueFrontRight = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.BrakeCommand.BrakeTorqueFrontLeft = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.BrakeCommand.BrakeTorqueRearRight = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.BrakeCommand.BrakeTorqueRearLeft = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case MRCU_STEERING_ANGLE:
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_STEERING_ANGULAR_VELOCITY:
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_BRAKE_SATUTS:
		Vehicle.WheelFrontRight.Car_Brake.Wheel_Brake_status = RxData[0];
		Vehicle.WheelFrontLeft.Car_Brake.Wheel_Brake_status = RxData[1];
		Vehicle.WheelRearRight.Car_Brake.Wheel_Brake_status = RxData[2];
		Vehicle.WheelRearLeft.Car_Brake.Wheel_Brake_status = RxData[3];
		break;
	case MRCU_ULTRASONIC:
		Vehicle.UltraSonicSensors.UltraSonicFrontRight = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.UltraSonicSensors.UltraSonicFrontLeft = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.UltraSonicSensors.UltraSonicRearRight = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.UltraSonicSensors.UltraSonicRearLeft = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case MRCU_STATE_OF_CHARGE:
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryVoltage = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryCurrent = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryChargePercentage = RxData[4];
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryStatus = RxData[5];
		break;
	case MRCU_POWER_STATUS_CAHNNEL_REQUEST:
		break;
	case MRCU_TEMP_HUMIDITY_FAN_REQUEST:
		break;
	case MRCU_RELAYS_H_BRIGDE_DIGITAL_OUTPUTS_COMMAND:
		break;
	case MRCU_DIGITAL_ANALOG_INPUT_REQUEST:
		break;

	case ORIN_SET_DRIVING_COMMAND:
		Vehicle.DesiredCarParameters.DesiredVelocity = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.DesiredCarParameters.DesiredAcceleration = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.DesiredCarParameters.DesiredSteeringAngle = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case ORIN_SET_STEERING_MODE:
		Vehicle.DesiredCarParameters.DesiredSteeringMode = (RxData[0] == 0x00) ? Ackerman : Crab_mode;
		break;

	case ORIN_SET_LIGHTS:
		break;

	case ORIN_SET_EMERGENCY_BRAKE:
		Vehicle.DesiredCarParameters.DesiredBrake = float_decode(RxData[0] << 8 | RxData[1]);
		break;
	case ORIN_SET_AUTONOMOUS_MODE:
		Vehicle.AutonomousMode = (RxData[0] == 0x00) ? false : true;
		break;

	case STEERING_ANGLE_FRONT:
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case STEERING_ANGLE_REAR:
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case STEERING_ANGULAR_VELOCITY_FRONT:
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angular_Velocity_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angular_Velocity_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case STEERING_ANGULAR_VELOCITY_REAR:
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angular_Velocity_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angular_Velocity_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angular_Velocity = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case BRAKE_APPLIED_TORQUE_FRONT:
		Vehicle.WheelFrontRight.Car_Brake.Wheel_Brake_Torque_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelFrontRight.Car_Brake.Wheel_Brake_Torque = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelFrontLeft.Car_Brake.Wheel_Brake_Torque_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelFrontLeft.Car_Brake.Wheel_Brake_Torque = float_decode(RxData[6] << 8 | RxData[7]);
		break;

	case BRAKE_APPLIED_TORQUE_REAR:
		Vehicle.WheelRearRight.Car_Brake.Wheel_Brake_Torque_Sample_Time = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.WheelRearRight.Car_Brake.Wheel_Brake_Torque = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.WheelRearLeft.Car_Brake.Wheel_Brake_Torque_Sample_Time = float_decode(RxData[4] << 8 | RxData[5]);
		Vehicle.WheelRearLeft.Car_Brake.Wheel_Brake_Torque = float_decode(RxData[6] << 8 | RxData[7]);
		break;
	case BRAKE_BRAKE_STATUS:
		Vehicle.WheelFrontRight.Car_Brake.Wheel_Brake_status = RxData[0];
		Vehicle.WheelFrontLeft.Car_Brake.Wheel_Brake_status = RxData[1];
		Vehicle.WheelRearRight.Car_Brake.Wheel_Brake_status = RxData[2];
		Vehicle.WheelRearLeft.Car_Brake.Wheel_Brake_status = RxData[3];
		break;
	case PDU_BATTERY_STATE_OF_CHARGE:
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryVoltage = float_decode(RxData[0] << 8 | RxData[1]);
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryCurrent = float_decode(RxData[2] << 8 | RxData[3]);
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryChargePercentage = RxData[4];
		Vehicle.Car_PDU.BatteryStateOfCharge.BatteryStatus = RxData[5];
		break;
	case PDU_FAULT_MESSAGE:
		break;
	case PDU_POWER_STATUS_CAHNNEL_RESPONSE:
		break;
	case PDU_TEMP_HUMIDITY_FAN_RESPONSE:
		break;
	case PDU_DIGITAL_ANALOG_INPUT_RESPONSE:
		break;
	}

	return STATUS_OK;
}

uint32_t EncodeCANID(uint8_t boardNumber, uint8_t messageNumber, uint8_t status)
{
	// Ensure the values fit their respective bit ranges
	boardNumber &= 0x07;	  // 3 bits for board number
	messageNumber &= 0x1F; // 5 bits for message number
	status &= 0x03;		  // 2 bits for status

	// Shift and combine the fields to form the CAN ID
	return (boardNumber << 8) | (messageNumber << 2) | status;
}
void DecodeCANID(uint16_t canID, uint8_t *boardNumber, uint8_t *messageNumber, uint8_t *status)
{
	*boardNumber = (canID >> 8) & 0x07;	  // Extract board number
	*messageNumber = (canID >> 2) & 0x1F; // Extract message number
	*status = canID & 0x03;					  // Extract status
}

/*
 compress float number (32Bit) to uint16_t (16Bit)
 @float _number must be in range (-8190 ~ +8190 )
 */
uint16_t float_encode(float _number)
{
#define MAX_int13 0x1FFF

	uint16_t ret_16;
	uint8_t sign = 0;

	if (_number > 0)
		sign = 0;
	else if (_number < 0)
	{
		sign = 1;
		_number *= -1;
	}

	uint32_t exp = 1;
	uint8_t Dec_exp = 0;

	if (_number >= MAX_int13)
		return 0XFFFF; // number can't be encode
	if (_number * 1000 < MAX_int13)
	{
		exp = 1000;
		Dec_exp = 3;
	}
	else if (_number * 100 < MAX_int13)
	{
		exp = 100;
		Dec_exp = 2;
	}
	else if (_number * 10 < MAX_int13)
	{
		exp = 10;
		Dec_exp = 1;
	}
	else
	{
		exp = 1;
		Dec_exp = 0;
	}

	ret_16 = ((uint16_t)(_number * exp) << 2);
	ret_16 |= Dec_exp;
	ret_16 |= (sign << 15);

	return ret_16;
}

/*
 uncompress uint16_t (16Bit) to float number (32Bit)
 @uint16_t _number must be float_encode form
 */
float float_decode(uint16_t _number)
{

	float ret_float;
	int8_t sign = 0;
	if (_number == 0XFFFF)
		return (float)NAN;
	(_number & 0x8000) ? (sign = -1) : (sign = 1); // get sign

	uint32_t exp = 1;
	uint8_t Dec_exp = (uint8_t)(_number & 0x0003);
	switch (Dec_exp)
	{
	case 0:
		exp = 1;
		break;
	case 1:
		exp = 10;
		break;
	case 2:
		exp = 100;
		break;
	case 3:
		exp = 1000;
		break;
	}

	ret_float = (float)((_number & 0x7FFC) >> 2) * (float)sign / (float)exp;

	return ret_float;
}

STATUS_TypeDef SendAcknowledgmentMessage(uint8_t MessageNumber)
{
	/*
	uint32_t TxMailbox;
	CANMessageStatus MessageState = Response;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = EncodeCANID(GetSenderCANID(MessageNumber), MRCU_ACKNOWLEDGE, MessageState);
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x02;
	uint8_t Message[2] = { BOARDNUMBER, MessageNumber };
	if (HAL_CAN_AddTxMessage(Can, &TxHeader, Message, &TxMailbox) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_ERROR;
		*/
}

uint8_t GetSenderCANID(uint8_t MessageNumber)
{

	if (MessageNumber >= MRCU_ACKNOWLEDGE && MessageNumber <= MRCU_RESERVED_MESSAGE_8)
		return 1;
	else if (MessageNumber >= ORIN_SET_DRIVING_COMMAND && MessageNumber <= ORIN_RESERVED_MESSAGE_5)
		return 2;
	else if (MessageNumber >= STEERING_ANGLE_FRONT && MessageNumber <= STEERING_RESERVED_MESSAGE_7)
		return 3;
	else if (MessageNumber >= BRAKE_APPLIED_TORQUE_FRONT && MessageNumber <= BRAKE_RESERVED_MESSAGE_6)
		return 4;
	else if (MessageNumber >= PDU_BATTERY_STATE_OF_CHARGE && MessageNumber <= PDU_RESERVED_MESSAGE_1)
		return 5;
	else
		return 0;
}
