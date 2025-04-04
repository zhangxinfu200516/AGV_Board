/**
 * @file alg_power_limit.h
 * @author qyx
 * @brief 自适应功率限制算法
 * @version 1.2
 * @date
 *
 * @copyright ZLLC 2025
 *
 */

#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"
#include "RLS.hpp"

/* Exported macros -----------------------------------------------------------*/
#define RAD_TO_RPM 9.5493f
#define PI 3.14159265354f

/*3508参数*/
#define M3508_REDUATION (3591.f / 187.f)                                                      // 3508标准减速比
#define M3508_TORQUE_CONSTANT 0.3                                                             // 3508带标准减速箱的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f)                                  // Icmd映射到Itorque
#define M3508_Kt (M3508_TORQUE_CONSTANT)                                                        // 3508转子的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)          // 发送的电流控制值（16384）映射到转子扭矩
#define M3508_TORQUE_TO_CMD_CURRENT (1.0f / (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)) // 转子扭矩映射到电流控制值（16384）
/*------------------------------------------------------------------------*/

/*6020参数*/
// 需哨兵组成员补充，最终需要GM6020_CMD_CURRENT_TO_TORQUE，GM6020_TORQUE_TO_CMD_CURRENT
/*----------------------------------------------------------------------------*/

#ifdef AGV

#if defined(INFANTRY) || defined(HERO)
// 转向电机参数选择
#define DIR_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define DIR_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

// 动力电机参数选择
#define MOT_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define MOT_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

#endif

#ifdef SENTRY
// 转向电机参数选择
#define DIR_CMD_CURRENT_TO_TORQUE GM6020_CMD_CURRENT_TO_TORQUE
#define DIR_TORQUE_TO_CMD_CURRENT GM6020_TORQUE_TO_CMD_CURRENT

// 动力电机参数选择
#define MOT_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define MOT_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

#endif

// 根据电机索引选择对应的转换系数
#define GET_CMD_CURRENT_TO_TORQUE(motor_index) ((motor_index % 2 == 0) ? DIR_CMD_CURRENT_TO_TORQUE : MOT_CMD_CURRENT_TO_TORQUE)
#define GET_TORQUE_TO_CMD_CURRENT(motor_index) ((motor_index % 2 == 0) ? DIR_TORQUE_TO_CMD_CURRENT : MOT_TORQUE_TO_CMD_CURRENT)

#endif
/*---------------------------------------------------------------------------------*/

typedef struct
{
    int16_t feedback_omega;  // 反馈的转子转速,rpm
    float feedback_torque; // 反馈的转子转矩,Nm

    float torque;           // pid输出的转子转矩,Nm
    float theoretical_power; // 理论功率
    float scaled_power;      // 功率（收缩后）

    int16_t pid_output; // pid输出的扭矩电流控制值（16384）
    int16_t output;     // 最终输出扭矩电流控制值（16384）
} Struct_Power_Motor_Data;

typedef struct
{
    float Max_Power;            // 最大功率限制
    float Scale_Conffient;         // 功率收缩系数
    float Theoretical_Total_Power; // 理论总功率
    float Scaled_Total_Power;      // 收缩后总功率
    float Actual_Power;            // 实际总功率

    Struct_Power_Motor_Data Motor_Data[8]; // 舵轮底盘八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机

} Struct_Power_Management;

class Class_Power_Limit
{
public:
    float Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index);
    float Calculate_Toque(float omega, float power, float torque, uint8_t motor_index);
    float Calculate_Limit_K(float omega[],float torque[],float power_limit,uint8_t motor_nums);
    void Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data);
    void Power_Task(Struct_Power_Management &power_management);
    void Init();
    //英雄广工方案
    float Calculate_Theoretical_Power(float omega[], float torque[], int number);
    void Power_Allocate(float Power_Limit, float rate,float dir_predict_power,float mot_predict_power,float *dir_power_allocate,float *mot_power_allocate);
    void Power_Limit_Task();

    // 普通模式下的getter/setter
    inline float Get_K1() const { return k1; }
    inline float Get_K2() const { return k2; }
    inline float Get_K3() const { return k3; }
    inline float Get_forget_k() { return forget_k; }
    
    inline void Set_K1(float _k1) { k1 = _k1; }
    inline void Set_K2(float _k2) { k2 = _k2; }
    inline void Set_K3(float _k3) { k3 = _k3; }
    inline void Set_Forget_K(float _forget_k) { forget_k = _forget_k; }

protected:
    float k1 = 1.3f;//0.00005;S
    float k2 = 0.0101f;//487.948486;//477.325562;
    float k3 = 6.6f / 8.0f;
    float forget_k = 0.9999f;
    RLS<2> rls{1e-5f,forget_k};
    float power_predict; //功率预测
    float limit_k; //伸缩因子

};

#endif