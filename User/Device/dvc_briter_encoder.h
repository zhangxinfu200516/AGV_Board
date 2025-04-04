#ifndef BRITER
#define BRITER

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"
#include "alg_power_limit.h"
#include "dvc_dwt.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BRITER_ENCODER_DATA_LENGTH_4 = 0x04,
    BRITER_ENCODER_DATA_LENGTH_5 = 0x05,
    BRITER_ENCODER_DATA_LENGTH_7 = 0x07,
} BRITER_ENCODER_TRANS_LENGTH_t;

typedef enum
{
    GET_TOTAL_ANGLE = 0x01,
    SET_CAN_ID = 0x02,
    SET_CAN_BAUD_RATE = 0x03,
    SET_CALLBACK_MODE = 0x04,
    SET_CALLBACK_PERIOD = 0x05,
    SET_CURRENT_POS_ZERO_POS = 0x06,
    SET_INCREMENT_DIRECTION = 0x07,
    SET_CURRENT_POS_MID_POS = 0x0C,
    SET_CURRENT_POS_SPECIFIC_VALUE = 0x0D,
    SET_CURRENT_POS_5ROUND_VALUE = 0x0F

} BRITER_ENCODER_COMMAND_CODE_t;

/**
 * @brief 波特率
 *
 */
typedef enum
{
    BRITER_ENCODER_SET_CAN_BAUD_RATE_500K = 0x00,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_1M,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_250K,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_125K,
    BRITER_ENCODER_SET_CAN_BAUD_RATE_100K

} BRITER_ENCODER_CAN_BAUD_RATE_t;

typedef struct
{
    uint8_t length;
    uint8_t encoder_address;
    uint8_t command_code;
    uint8_t data[5];

} briter_encoder_command_t;

/**
 * @brief 增量方向
 *
 */
typedef enum
{
    BRITER_ENCODER_INCREMENT_DIRECTION_CW = 0x00,
    BRITER_ENCODER_INCREMENT_DIRECTION_CCW

} BRITER_ENCODER_INCREMENT_DIRECTION_t;

/**
 * @brief 编码器canid
 *
 */
typedef enum
{
    A_ENCODER_ID_e = 0x0AU,
    B_ENCODER_ID_e = 0x0BU,
    C_ENCODER_ID_e = 0x0CU,
    D_ENCODER_ID_e = 0x0DU
} Enum_Encoder_ID;

/**
 * @brief 编码器状态
 *
 */
typedef enum
{
    Encoder_Status_DISABLE = 0x00,
    Encoder_Status_ENABLE
} Enum_Briter_Encoder_Status;

// 解析过的布瑞特编码器数据
typedef struct
{
    int32_t Raw_Value;     // 最原始的反馈数据
    int32_t Pre_Raw_Value; // 上一时刻的反馈数据

    float Now_Multi_Turn_Angle; // 编码器当前多圈角度，deg
    float Now_Angle;            // 编码器当前单圈内角度，deg
    float Now_Omega;            // 编码器当前角速度，deg/s

} Struct_Briter_Encoder_Data;

/**
 * @brief 布瑞特电机源数据
 *
 */
struct Struct_Briter_Encoder_Can_Data
{
    uint8_t Length;
    uint8_t Encoder_Address;
    uint8_t Command_Code;
    uint8_t Data[5];
} __attribute__((packed));

typedef struct
{
    uint16_t Lsbs_Per_Encoder_Round;                          // 每圈编码器分辨率
    BRITER_ENCODER_CAN_BAUD_RATE_t Baud_Rate;                 // 编码器波特率
    BRITER_ENCODER_INCREMENT_DIRECTION_t Increment_Direction; // 增量方向

} Birter_Encoder_Parameter_t;

class Class_Briter_Encoder
{
public:
    void Init(CAN_HandleTypeDef *hcan, Enum_Encoder_ID __CAN_ID, BRITER_ENCODER_CAN_BAUD_RATE_t __Briter_Encoder_Baud_Rate = BRITER_ENCODER_SET_CAN_BAUD_RATE_1M, uint16_t __Lsbs_Per_Encoder_Round = 1024, BRITER_ENCODER_INCREMENT_DIRECTION_t __Increment_Direction = BRITER_ENCODER_INCREMENT_DIRECTION_CW);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_PeriodElapsedCallback();
    void TIM_Alive_PeriodElapsedCallback();

    void Briter_Encoder_Request_Total_Angle(void);

    inline float Get_Now_Angle();
    inline float Get_Now_Omega();

    void output(void);

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_Encoder_ID CAN_ID;

    // 编码器参数
    Birter_Encoder_Parameter_t Parameter;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;

    // 当前时刻的接收flag
    uint32_t Flag = 0;
    // 前一时刻的接收flag
    uint32_t Pre_Flag = 0;
    // 编码器上电第一帧标志位
    uint8_t Start_Falg = 0;

    // 状态
    Enum_Briter_Encoder_Status Encoder_Status = Encoder_Status_DISABLE;

    // 经处理过的数据
    Struct_Briter_Encoder_Data Data;

    Struct_Briter_Encoder_Can_Data Command;

    // 内部函数
    void
    Data_Process();
};

float Class_Briter_Encoder::Get_Now_Omega()
{
    return Data.Now_Omega;
}

float Class_Briter_Encoder::Get_Now_Angle()
{
    return Data.Now_Angle;
}

#endif // !BRITER