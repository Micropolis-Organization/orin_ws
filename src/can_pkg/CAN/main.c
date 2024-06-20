
#include "CAN.h"
#include "CANBus.h"
#include "PrjTypedef.h"

int lengthData = 0;
unsigned char RecivedData[20] = {0};
uint8_t CAN_Message_Length = 0;
uint16_t CAN_ID = 0;
uint8_t BoardNumber;
uint8_t Status;
uint8_t MessageNumber;
extern Car_Satus Vehicle;
CANMessage cMessage;
int main(int argc, char *argv[])
{
   CANInit();
   memset(RecivedData, 0, sizeof(RecivedData));
   while (1)
   {
      // send msg
      // 0.5
		// Vehicle.WheelFrontRight.Car_Driving.Wheel_Driving_Velocity = float_decode(RxData[0] << 8 | RxData[1]);

      float speed = 0.5;
      float acceleration = 0;
      float steering_angle = 0.35;
      float steering_angular_velocity = 0;
      #define mrcu_id 0x01
      cMessage.CANMessgeID = EncodeCANID(mrcu_id, ORIN_SET_DRIVING_COMMAND, 0x01); // Receiver canID, message type, status
      cMessage.CANMessageLength = 0x08;
      //speed
      cMessage.CANMessageData[0] = (uint8_t)(float_encode(speed) >> 8);
      cMessage.CANMessageData[1] = (uint8_t)(float_encode(speed) & 0x00FF);
      //acceleration
      cMessage.CANMessageData[2] = (uint8_t)(float_encode(acceleration) >> 8);
      cMessage.CANMessageData[3] = (uint8_t)(float_encode(acceleration) & 0x00FF);
      //steering angle
      cMessage.CANMessageData[4] = (uint8_t)(float_encode(steering_angle) >> 8);
      cMessage.CANMessageData[5] = (uint8_t)(float_encode(steering_angle) & 0x00FF);
      //steering angular velocity
      cMessage.CANMessageData[6] = (uint8_t)(float_encode(steering_angular_velocity) >> 8);
      cMessage.CANMessageData[7] = (uint8_t)(float_encode(steering_angular_velocity) & 0x00FF);

      if (SendCANMessage(cMessage)==STATUS_OK)
      {
         fprintf(stdout, "Message sent successfully\n");
      }
      else
      {
         fprintf(stdout, "Message not sent\n");
      }
      usleep(250);

      

      // lengthData = frame_recv(RecivedData);
      // fprintf(stdout,"lengthData=%d\n",lengthData);
      if (ReceiveCANMessage(RecivedData, &lengthData) == STATUS_OK)
      {
         if (RecivedData[0] == 0xaa && RecivedData[lengthData - 1] == 0x55)
         {
            CAN_ID = RecivedData[3] << 8 | RecivedData[2];
            DecodeCANID(CAN_ID, &BoardNumber, &MessageNumber, &Status);
            if (BoardNumber == BOARDNUMBER)
            {

            //    CAN_Message_Length = (uint8_t)(RecivedData[1] & 0x0F);
               ParserCANMessage(CAN_ID, &RecivedData[4]);
            //    fprintf(stdout, "Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle=%.2f\n",
            //            Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle);
            //    fprintf(stdout, "Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle=%.2f\n",
            //            Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle);
            //    fprintf(stdout, "Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle=%.2f\n",
            //            Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle);
            //    fprintf(stdout, "Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle=%.2f\n",
            //            Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle);
            //    fprintf(stdout, "-------------------------------------------------\n");
            //    BoardNumber = 0;
            //    memset(RecivedData, 0, sizeof(RecivedData)); // Initialize buffer to zero before using it.
            }
         }
      }
   }
   return EXIT_SUCCESS;
}
