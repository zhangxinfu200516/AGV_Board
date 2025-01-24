#include "crt_steering_wheel.h"
Class_Steering_Wheel steering_wheel;
void Class_Steering_Wheel::CAN_RxAgvBoardCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{

    switch (CAN_RxMessage->Header.StdId)
    {
    case 0x02A:
    {
        memcpy(&Power_Management.Motor_Data[0], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[1], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02B:
    {
        memcpy(&Power_Management.Motor_Data[2], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[3], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02C:
    {
        memcpy(&Power_Management.Motor_Data[4], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[5], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x02D:
    {
        memcpy(&Power_Management.Motor_Data[6], CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[7], CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x03A:
    {
        memcpy(&Power_Management.Motor_Data[0].torque, CAN_RxMessage->Data, 2);
        memcpy(&Power_Management.Motor_Data[1].torque, CAN_RxMessage->Data + 2, 2);
    }
    break;
    case 0x03B:
    {
        memcpy(&Power_Management.Motor_Data[2].torque, CAN_RxMessage->Data, 2);
        memcpy(&Power_Management.Motor_Data[3].torque, CAN_RxMessage->Data + 2, 2);
    }
    break;
    case 0x03C:
    {
        memcpy(&Power_Management.Motor_Data[4].torque, CAN_RxMessage->Data, 2);
        memcpy(&Power_Management.Motor_Data[5].torque, CAN_RxMessage->Data + 2, 2);
    }
    break;
    case 0x03D:
    {
        memcpy(&Power_Management.Motor_Data[6].torque, CAN_RxMessage->Data, 2);
        memcpy(&Power_Management.Motor_Data[7].torque, CAN_RxMessage->Data + 2, 2);
    }
    break;
    }
}

// 这里要根据帧ID判断是功率数据还是角度速度数据
//float velocity_x, velocity_y, velocity, theta;
//float last_angle=0;
float power;
float power_max = 70;
void Class_Steering_Wheel::CAN_RxChassisCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{   //角度是以编码值为单位的（单圈编码值为8191），速度是以RPM为单位的
    uint16_t tmp_angle_encoder = 0;
    int16_t tmp_velocity_rpm = 0;
    Enum_Steering_Cmd_Status Cmd_Status;

    CanRX_Chassis_Cmd_Flag++;

    if (CAN_RxMessage->Header.StdId == AGV_BOARD_ID)
    {
        #ifdef Forward_Code
        memcpy(&velocity_x, CAN_RxMessage->Data, 4);
        memcpy(&velocity_y, CAN_RxMessage->Data + 4, 4);

        velocity = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
        theta = My_atan(velocity_y, velocity_x) + PI; // 映射到[0,2PI]

        this->Target_Velocity = velocity;
        this->Target_Angle = theta * RAD_TO_DEG;
        #endif
	    memcpy(&tmp_angle_encoder,CAN_RxMessage->Data,2);
        memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+2,2);
        memcpy(&Cmd_Status,CAN_RxMessage->Data+4,1);
        this->Target_Angle = Math_Int_To_Float(tmp_angle_encoder,0,0xffff,0.0f,8191.0f) * 360.0f / 8191.0f;
        this->Target_Omega = (float)tmp_velocity_rpm * 6.0f;
        this->Target_Velocity = this->Target_Omega / 180.0f * PI * Wheel_Diameter / 2.0f;//rpm转为m/s
        this->Steering_Cmd_Status = Cmd_Status;
				
    }

    if (CAN_RxMessage->Header.StdId == 0x01E)
    {
        memcpy(&Power_Management.Max_Power, CAN_RxMessage->Data, 2);
	    memcpy(&Power_Management.Actual_Power,CAN_RxMessage->Data+2,4);
        power = Power_Management.Actual_Power;
			 power_max = 70.0f;
    }
}

void Class_Steering_Wheel::CanRX_Chassis_TIM_Alive_PeriodElapsedCallback()
{
    //判断是否有新的指令
    if(CanRX_Chassis_Cmd_Flag == CanRX_Chassis_Cmd_Pre_Flag)
    {
        //与底盘通信断开
        CanRX_Chassis_Cmd_Status = CanRX_Chassis_Cmd_Status_DISABLE;
    }
    else
    {
        //与底盘通信连接
        CanRX_Chassis_Cmd_Status = CanRX_Chassis_Cmd_Status_ENABLE;
    }
    CanRX_Chassis_Cmd_Pre_Flag = CanRX_Chassis_Cmd_Flag;
}

void Class_Steering_Wheel::CanRX_AGV_Board_TIM_Alive_PeriodElapsedCallback()
{
    //判断是否有新的指令
    if(CanRX_AGV_Board_Flag == CanRX_AGV_Board_Pre_Flag)
    {
        //与其他舵通信断开
        CanRX_AGV_Board_Flag_Status = CanRX_AGV_Board_Status_DISABLE;
    }
    else
    {
        //与其他舵通信连接
        CanRX_AGV_Board_Flag_Status = CanRX_AGV_Board_Status_ENABLE;
    }
    CanRX_AGV_Board_Pre_Flag = CanRX_AGV_Board_Flag;
}
void Class_Steering_Wheel::Init()
{
    memset(&Power_Management, 0, sizeof(Power_Management));

    // todo:待调参

    Motion_Motor.PID_Omega.Init(7.5, 0, 0, 0, 0, 16384);
    Motion_Motor.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, MOT_OUTPUT_TO_ROTOR_RATIO);

    Directive_Motor.PID_Angle.Init(4.5, 0, 0, 0, 0, 16384);
    Directive_Motor.PID_Omega.Init(25, 0, 0, 0, 0, 16384);

#ifdef DEBUG_DIR_SPEED
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 8);
#else
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_ANGLE, DIR_OUTPUT_TO_ROTOR_RATIO);
#endif 

    Encoder.Init(&hcan1, static_cast<Enum_Encoder_ID>(ENCODER_ID));
    Power_Limit.Init();
}
