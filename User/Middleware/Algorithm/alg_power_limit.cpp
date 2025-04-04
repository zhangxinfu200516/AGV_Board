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

#include "alg_power_limit.h"
#include "math.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }
static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }

void Class_Power_Limit::Init()
{

    float initParams[2] = {k1, k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
}

/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机，舵轮需要传此参数
 * @return float 理论功率值
 */
float Class_Power_Limit::Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
{

    float cmdPower = rpm2av(omega) * torque +
<<<<<<< HEAD
                     fabs(rpm2av(omega)) * fabs(rpm2av(omega))* k1 +
=======
                     fabs(rpm2av(omega)) * fabs(rpm2av(omega)) * k1 +
>>>>>>> 74f5c9f ([测试提交ä中é文])
                     torque * torque * k2 +
                     k3;

    return cmdPower;
}

float Class_Power_Limit::Calculate_Theoretical_Power(float omega[], float torque[], int number)
{
    double tmp_predict = 0;
    for (int i = 0; i < number; i++)
    {
				tmp_predict += fabs(rpm2av(omega[i]) * torque[i]) +
						k1 * torque[i] * torque[i] +
						k2 * rpm2av(omega[i]) * rpm2av(omega[i]) +
						k3;
    }
    return tmp_predict;
}
/**
 * @brief 计算功率系数
 *
 * @param actual_power 实际功率
 * @param motor_data 电机数据数组
 */
void Class_Power_Limit::Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data)
{

    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    float effectivePower = 0;

    samples[0][0] = samples[1][0] = 0;

    if (actual_power > 5)
    {
        for (int i = 0; i < 8; i++)
        {
            if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
            {
                effectivePower += motor_data[i].feedback_torque *
                                  rpm2av(motor_data[i].feedback_omega);
            }
            samples[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega)) * fabsf(rpm2av(motor_data[i].feedback_omega));
            samples[1][0] += motor_data[i].feedback_torque *
                             motor_data[i].feedback_torque;
        }

        params = rls.update(samples, actual_power - effectivePower - 8 * k3);
        k1 = my_fmax(params[0][0], 1e-7f);
        k2 = my_fmax(params[1][0], 1e-7f);
    }
}

float Class_Power_Limit::Calculate_Limit_K(float omega[], float torque[], float power_limit, uint8_t motor_nums)
{
    float limit_k = 1.; // 输出伸缩因子k

    float tmp_predict;

    float torque_square_sum;
    float omega_square_sum;
    float torque_multi_omega_sum;

    float func_a, func_b, func_c;
    float delta;

    for (int i = 0; i < motor_nums; i++)
    {
        torque_square_sum += torque[i] * torque[i];
        omega_square_sum += rpm2av(omega[i]) * rpm2av(omega[i]);
        torque_multi_omega_sum += fabs(rpm2av(omega[i]) * torque[i]);

        tmp_predict += fabs(rpm2av(omega[i]) * torque[i]) +
                       k1 * torque[i] * torque[i] +
                       k2 * rpm2av(omega[i]) * rpm2av(omega[i]) +
                       k3;
    }
    // power_predict = tmp_predict;

    if (tmp_predict > power_limit)
    {
        func_a = k1 * torque_square_sum;
        func_b = torque_multi_omega_sum;
        func_c = k2 * omega_square_sum - power_limit + k3 * motor_nums;

        delta = func_b * func_b - 4 * func_a * func_c; // b*b-4*a*c

        if (delta >= 0)
        {
            limit_k = (-func_b + sqrtf(delta)) / (2 * func_a); // 求根公式
        }
        else
        {
            limit_k = 1;
        }
    }

    return (limit_k > 1) ? 1 : limit_k;
}

/**
 * @brief 计算限制后的扭矩
 *
 * @param omega 转子转速，单位为rpm
 * @param power 限制功率值
 * @param torque 原始扭矩值
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机
 * @return float 限制后的扭矩值
 */
float Class_Power_Limit::Calculate_Toque(float omega, float power, float torque, uint8_t motor_index)
{

    omega = rpm2av(omega);
    float newTorqueCurrent = 0.0f;

    float delta = omega * omega - 4 * (k1 * fabs(omega) * fabs(omega) + k3 - power) * k2;

    if (power <= 0)
    // if (torque * omega <= 0)
    {
        newTorqueCurrent = torque;
    }
    else
    {
        if (floatEqual(delta, 0.0f))
        {
            newTorqueCurrent = -omega / (2.0f * k2);
        }
        else if (delta > 0.0f)
        {

            float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
            float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);

            newTorqueCurrent = (torque > 0) ? solution1 : solution2;
        }
        else
        {
            newTorqueCurrent = -omega / (2.0f * k2);
        }
    }
    return newTorqueCurrent;
}

/**
 * @brief 功率限制主任务
 *
 * @param power_management 功率管理结构体
 */
void Class_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
#ifdef AGV
    float theoretical_sum_mot = 0, theoretical_sum_dir = 0;
    float scaled_sum_mot = 0, scaled_sum_dir = 0;

    /*******************************广工测试**************************************/
    float omega[8];
    float torque[8];
    float power_limit = power_management.Max_Power;
    for (int i = 0; i < 8; i++)
    {
        omega[i] = power_management.Motor_Data[i].feedback_omega;
        torque[i] = power_management.Motor_Data[i].torque;
    }
    Calculate_Limit_K(omega, torque, power_limit, 8);
    /*******************************广工测试**************************************/

    // 分别计算动力和转向电机的理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);

        if (i % 2 == 0) // 转向电机
        {
            if (power_management.Motor_Data[i].theoretical_power > 0)
            {
                theoretical_sum_dir += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                power_management.Motor_Data[i].theoretical_power = 0;
            }
        }
        else // 动力电机
        {
            if (power_management.Motor_Data[i].theoretical_power > 0)
            {
                theoretical_sum_mot += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                power_management.Motor_Data[i].theoretical_power = 0;
            }
        }
    }

    // 新的功率分配逻辑
    float scale_mot = 1.0f, scale_dir = 1.0f;
    float dir_power_limit = power_management.Max_Power * 0.8f; // 转向电机功率上限
    float mot_power_limit;                                     // 动力电机功率上限，动态计算

    // 首先分配转向电机功率
    if (theoretical_sum_dir > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        scale_dir = dir_power_limit / theoretical_sum_dir;
        mot_power_limit = power_management.Max_Power - dir_power_limit;
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        scale_dir = 1.0f;
        // 剩余功率全部分配给动力电机
        mot_power_limit = power_management.Max_Power - theoretical_sum_dir;
    }

    // 然后分配动力电机功率
    if (theoretical_sum_mot > mot_power_limit)
    {
        scale_mot = mot_power_limit / theoretical_sum_mot;
    }
    else
    {
        scale_mot = 1.0f;
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 8; i++)
    {
        float scale = (i % 2 == 0) ? scale_dir : scale_mot;
        power_management.Motor_Data[i].scaled_power =
            power_management.Motor_Data[i].theoretical_power * scale;

        if (i % 2 == 0)
        {
            scaled_sum_dir += power_management.Motor_Data[i].scaled_power;
        }
        else
        {
            scaled_sum_mot += power_management.Motor_Data[i].scaled_power;
        }

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            GET_TORQUE_TO_CMD_CURRENT(i);

        // 限幅处理
        power_management.Motor_Data[i].output =
            (power_management.Motor_Data[i].output > 16384) ? 16384 : (power_management.Motor_Data[i].output < -16384) ? -16384
                                                                                                                       : power_management.Motor_Data[i].output;
    }

    power_management.Theoretical_Total_Power = theoretical_sum_mot + theoretical_sum_dir;
    power_management.Scaled_Total_Power = scaled_sum_mot + scaled_sum_dir;

#ifdef AGV_BOARD_B
// Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
#endif
#else
    float theoretical_sum = 0;
    float scaled_sum = 0;

    // 计算理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);
        if (power_management.Motor_Data[i].theoretical_power > 0)
        {
            theoretical_sum += power_management.Motor_Data[i].theoretical_power;
        }
    }

    power_management.Theoretical_Total_Power = theoretical_sum;

    // 计算收缩系数
    if (power_management.Max_Power < power_management.Theoretical_Total_Power)
    {
        power_management.Scale_Conffient =
            power_management.Max_Power / power_management.Theoretical_Total_Power;
    }
    else
    {
        power_management.Scale_Conffient = 1.0f;
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].scaled_power =
            power_management.Motor_Data[i].theoretical_power *
            power_management.Scale_Conffient;

        scaled_sum += power_management.Motor_Data[i].scaled_power;

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            TORQUE_TO_CMD_CURRENT;

        if (abs(power_management.Motor_Data[i].output) >= 16384)
        {
            power_management.Motor_Data[i].output = 0;
        }
    }

    power_management.Scaled_Total_Power = scaled_sum;

    Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
#endif
}

void Class_Power_Limit::Power_Allocate(float Power_Limit, float rate,float dir_predict_power,float mot_predict_power,float *dir_power_allocate,float *mot_power_allocate)
{
    // 新的功率分配逻辑
    float dir_power_limit = Power_Limit * rate; // 转向电机功率上限
    float mot_power_limit = Power_Limit * (1.f-rate);      // 动力电机功率上限，动态计算

    //实际分配到的功率
    float dir_allocate,mot_allocate;

    // 首先分配转向电机功率
    if (dir_predict_power > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        dir_allocate = dir_power_limit; 
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        dir_allocate = dir_predict_power;
    }
    // 优先分配转向舵
    mot_power_limit = Power_Limit - dir_allocate;
    // 然后分配动力电机功率
    if (mot_predict_power > mot_power_limit)
    {
        mot_allocate = mot_power_limit;
    }
    else
    {
        mot_allocate = mot_predict_power;
    }
    //赋值给成员变量
    *dir_power_allocate = dir_allocate;
    *mot_power_allocate = mot_allocate;
}


void Class_Power_Limit::Power_Limit_Task()
{
    
}