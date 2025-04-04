#include "dvc_briter_encoder.h"
#include "string.h"
#include "config.h"
/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Encoder_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (A_ENCODER_ID_e):
        {
            tmp_tx_data_ptr = (CAN1_0x0A_Tx_Data);
        }
        break;

        case (B_ENCODER_ID_e):
        {
            tmp_tx_data_ptr = (CAN1_0x0B_Tx_Data);
        }
	break;
        case (C_ENCODER_ID_e):
        {
            tmp_tx_data_ptr = (CAN1_0x0C_Tx_Data);
        }
	  break;
        case (D_ENCODER_ID_e):
        {
            tmp_tx_data_ptr = (CAN1_0x0D_Tx_Data);
        }
        break;
        }
        return (tmp_tx_data_ptr);
    }
}

void Class_Briter_Encoder::output(void)
{
    memcpy(CAN_Tx_Data, &Command, sizeof(Struct_Briter_Encoder_Can_Data));
}

void Class_Briter_Encoder::Init(CAN_HandleTypeDef *hcan, Enum_Encoder_ID __CAN_ID, BRITER_ENCODER_CAN_BAUD_RATE_t __Briter_Encoder_Baud_Rate, uint16_t __Lsbs_Per_Encoder_Round, BRITER_ENCODER_INCREMENT_DIRECTION_t __Increment_Direction)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    this->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);

    Parameter.Baud_Rate = __Briter_Encoder_Baud_Rate;
    Parameter.Lsbs_Per_Encoder_Round = __Lsbs_Per_Encoder_Round;
    Parameter.Increment_Direction = __Increment_Direction;
}

float delta_time;
void Class_Briter_Encoder::Data_Process()
{
    static uint32_t Now_Time = 0;
    static uint32_t Pre_Time = 0;

  
    uint32_t tmp_encoder;
    int16_t tmp_omega;

    Struct_Briter_Encoder_Can_Data *tmp_buffer = (Struct_Briter_Encoder_Can_Data *)CAN_Manage_Object->Rx_Buffer.Data;
    Now_Time = DWT_GetTimeline_us();
    delta_time = Now_Time - Pre_Time;

    memcpy(&Data.Raw_Value, &tmp_buffer->Data, 4);                                                                              // 原始数据
    Data.Now_Multi_Turn_Angle = Data.Raw_Value * 360.0 / Parameter.Lsbs_Per_Encoder_Round;                                      // 求多圈角度
    Data.Now_Angle = fmod(Data.Now_Multi_Turn_Angle, 360.0f*OUTPUT_TO_ENCODER_RATIO);                                                                   // 求单圈角度
    Data.Now_Omega = (float)(Data.Raw_Value - Data.Pre_Raw_Value) / delta_time * 1000000.0 * 360.0 / Parameter.Lsbs_Per_Encoder_Round; // 求角速度，deg/s

    // 存储预备信息
    Data.Pre_Raw_Value = Data.Raw_Value;
    Pre_Time = Now_Time;

    if (Start_Falg == 0)
        Start_Falg = 1;
}

void Class_Briter_Encoder::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断编码器是否在线
    Flag += 1;

    Data_Process();
}

void Class_Briter_Encoder::TIM_PeriodElapsedCallback()
{
}

void Class_Briter_Encoder::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过编码器数据
    if (Flag == Pre_Flag)
    {
        // 编码器断开连接
        Encoder_Status = Encoder_Status_DISABLE;
    }
    else
    {
        // 编码器保持连接
        Encoder_Status = Encoder_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

void Class_Briter_Encoder::Briter_Encoder_Request_Total_Angle(void)
{

    Command.Length = BRITER_ENCODER_DATA_LENGTH_4;
    Command.Encoder_Address = CAN_ID;
    Command.Command_Code = GET_TOTAL_ANGLE;

    uint8_t trans_data[5];
    memcpy(Command.Data, &trans_data, 5);

    output();
}