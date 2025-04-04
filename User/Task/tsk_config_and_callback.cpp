/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**

 *
 */

/**
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "drv_dwt.h"
#include "config.h"
#include "drv_can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h" // ::CMSIS:RTOS2
#include "task.h"

#include "Steering_Wheel_Task.h"
/* Private macros ------------------------------------------------------------*/
//osThreadId_t steering_wheel_taskHandle;
//const osThreadAttr_t steering_wheel_task_attributes = {
//    .name = "steering_wheel_task",
//    .stack_size = 528 * 4,
//    .priority = (osPriority_t)osPriorityRealtime,
//};
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
#ifdef STEERING_WHEEL
/**
 * @brief
 *
 */
void Agv_Board_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        steering_wheel.Directive_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0x202):
    {
        steering_wheel.Motion_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case ENCODER_ID:
    {
        steering_wheel.Encoder.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

/**
 * @brief
 *
 */
void Agv_Board_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (AGV_BOARD_ID):
    case 0x01E:
    {
        steering_wheel.CAN_RxChassisCallback(CAN_RxMessage);
    }
    break;
    case (0x02A):
    case (0x02B):
    case (0x02C):
    case (0x02D):
    case (0x03A):
    case (0x03B):
    case (0x03C):
    case (0x03D):
    {
        steering_wheel.CAN_RxAgvBoardCallback(CAN_RxMessage);
    }
    break;
    #if defined (AGV_BOARD_A) || defined(AGV_BOARD_C) || defined(AGV_BOARD_D)
    case (0x20E):
    {
        float _k1=0,_k2=0; 
        memcpy(&_k1,CAN_RxMessage->Data,sizeof(float));
        memcpy(&_k2,CAN_RxMessage->Data+4,sizeof(float));
        steering_wheel.Power_Limit.Set_K1(_k1);
        steering_wheel.Power_Limit.Set_K2(_k2);
    }
    break;
    case (0x20F):
    {
        float forget_k = 0;
        memcpy(&forget_k,CAN_RxMessage->Data,sizeof(float));
        steering_wheel.Power_Limit.Set_Forget_K(forget_k);
    }
    break;
    #endif
    default:
         break;
    }
    

}

#endif

//void OSTaskInit()
//{
//    steering_wheel_taskHandle = osThreadNew(Steering_Wheel_Task, NULL, &steering_wheel_task_attributes);
//}

/**
 * @brief 初始化任务
 *
 */
extern "C" void
Task_Init()
{

    DWT_Init(72);

    /********************************** 驱动层初始化 **********************************/
#ifdef STEERING_WHEEL
    /**
     * @brief
     *
     */
	
    CAN_Init(&hcan1, Agv_Board_CAN1_Callback);
    CAN_Init(&hcan2, Agv_Board_CAN2_Callback);

#endif // DEBUG

    /********************************* 设备层初始化 *********************************/
    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
    steering_wheel.Init();      

	//OSTaskInit();
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
