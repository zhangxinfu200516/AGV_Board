#include "Steering_Wheel_Task.h"

#ifdef DEBUG_DIR_SPEED
float test_speed = 0.0;
#endif // DEBUG

// 应该要加互斥锁
void State_Update(Class_Steering_Wheel *steering_wheel)
{
    // 由于齿轮传动使得编码器转动方向为CW时，舵转动方向为CCW，反之亦然。所以要对称处理
    /*
        编码器顺时针旋转：     0° -> 90° -> 180°
        舵轮实际逆时针旋转：  360° -> 270° -> 180°

        所以需要：新角度 = 360° - 编码器角度
    */
    steering_wheel->Now_Angle = 360.0 - steering_wheel->Encoder.Get_Now_Angle() * ENCODER_TO_OUTPUT_RATIO;
    steering_wheel->Now_Omega = steering_wheel->Directive_Motor.Get_Now_Omega_Angle();                       // 由于转子反馈角速度的精度远大于编码器精度，所以用转子反馈的数据，并且舵转动方向即为转子转动方向，反馈的数据已经带有减速箱
    steering_wheel->Now_Velocity = steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * Wheel_Diameter / 2; // 这里有点怪异，因为Get_Now_Omega_Radian和Set_Target_Omega_Radian返回的数据不一样

// 更新功率控制所需的数据
#if defined(AGV_BOARD_A) || defined(AGV_BOARD_B) || defined(AGV_BOARD_C) || defined(AGV_BOARD_D)
    // 转向电机数据
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].feedback_omega = 
        steering_wheel->Directive_Motor.Get_Now_Omega_Radian() * 
        steering_wheel->Directive_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].feedback_torque = 
        steering_wheel->Directive_Motor.Get_Now_Torque() * 
        GET_CMD_CURRENT_TO_TORQUE(MOTOR_BASE_INDEX);

    // 动力电机数据
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].feedback_omega = 
        steering_wheel->Motion_Motor.Get_Now_Omega_Radian() * 
        steering_wheel->Motion_Motor.Get_Gearbox_Rate() * RAD_TO_RPM;
			steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].feedback_torque = 
        steering_wheel->Motion_Motor.Get_Now_Torque() * 
        GET_CMD_CURRENT_TO_TORQUE(MOTOR_BASE_INDEX + 1);

    // CAN数据打包
    // 反馈数据
    int16_t motion_motor_torque = steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].feedback_torque*1000.f;
    int16_t direct_motor_torque = steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].feedback_torque*1000.f;
    memcpy(AGV_BOARD_CAN_DATA_1, 
           &steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].feedback_omega, 2);
    memcpy(AGV_BOARD_CAN_DATA_1 + 2, &motion_motor_torque, 2);
    memcpy(AGV_BOARD_CAN_DATA_1 + 4, 
           &steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].feedback_omega, 2);
    memcpy(AGV_BOARD_CAN_DATA_1 + 6, 
           &direct_motor_torque, 2);

    // 控制数据
    memcpy(AGV_BOARD_CAN_DATA_2, 
           &steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].torque, 4);
    memcpy(AGV_BOARD_CAN_DATA_2 + 4, 
           &steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].torque, 4);
#endif

}

void Command_Update(Class_Steering_Wheel *steering_wheel)
{
}

/*
初始状态：
Now_Angle = 0°
Target_Angle = 135°
invert_flag = 0（不反转）

1. 角度优化：
   temp_err = 135° - 0° - (0 * 180°) = 135°

   比较路径：
   - 直接路径 = |135°| = 135°
   - 绕行路径 = 360° - |135°| = 225°
   temp_min = min(135°, 225°) = 135°

   因为 temp_min(135°) > 90°：
   - invert_flag 切换为 1（反转）
   - 重新计算误差：
     temp_err = 135° - 0° - (1 * 180°) = -45°

2. 优劣弧优化：
   temp_err = -45° 已经在[-180°, 180°]范围内，无需调整

3. 最终结果：
   - 电机反转
   - 逆时针转动45°
*/
        float power_limit;
        float dir_omega[4],dir_torque[4],mot_omega[4],mot_torque[4];
        float dir_predict_power,mot_predict_power;
        float dir_allocate_power,mot_allocate_power;

float temp_err = 0.0f;
float temp_target_angle = 0.0f;

float omega_k = 20.f;
float limit_k;
float limit_dir_k,limit_mot_k;
void Control_Update(Class_Steering_Wheel *steering_wheel)
{
    // 由于can中断会修改Power_Management的内容，所以这里进出临界区以保护数据
    portENTER_CRITICAL();


    //以下部分考虑后续整合为一个函数
    // 1. 角度优化
    if (steering_wheel->deg_optimization == ENABLE_MINOR_DEG_OPTIMIZEATION)
    {
        float temp_min;

        // 计算误差，考虑当前电机状态
        temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;

        // 标准化到[0, 360)范围
        while (temp_err > 360.0f)
            temp_err -= 360.0f;
        while (temp_err < 0.0f)
            temp_err += 360.0f;

        // 比较路径长度
        if (fabs(temp_err) < (360.0f - fabs(temp_err)))
            temp_min = fabs(temp_err);
        else
            temp_min = 360.0f - fabs(temp_err);

        // 判断是否需要切换方向
        if (temp_min > 90.0f)
        {
            steering_wheel->invert_flag = !steering_wheel->invert_flag;
            // 重新计算误差
            temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;
        }
    }
    else
    {
        temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle;
    }

    // 2. 优劣弧优化，实际上角度优化那里已经完成了
    if (steering_wheel->arc_optimization == ENABLE_MINOR_ARC_OPTIMIZEATION)
    {
        if (temp_err > 180.0f)
        {
            temp_err -= 360.0f;
        }
        else if (temp_err < -180.0f)
        {
            temp_err += 360.0f;
        }
    }
    temp_target_angle = steering_wheel->Now_Angle + temp_err;

    // PID参数更新
    steering_wheel->Motion_Motor.Set_Target_Omega_Angle(
        (steering_wheel->invert_flag ? -1 : 1) * steering_wheel->Target_Velocity / Wheel_Diameter * 2 * RAD_TO_DEG);
    steering_wheel->Motion_Motor.Set_Now_Omega_Angle(steering_wheel->Now_Velocity / Wheel_Diameter * 2 * RAD_TO_DEG); // Get_Now_Omega_Radian获得的是电机输出轴的角速度，这个角速度放在了data里，Set_Now_Omega_Radian设置的是电机类直属的Now_Omega_Radian

    steering_wheel->Directive_Motor.Set_Target_Angle(temp_target_angle);
    steering_wheel->Directive_Motor.Set_Now_Angle(steering_wheel->Now_Angle);       // 转向轮的当前数据不直接来自于电机
    steering_wheel->Directive_Motor.Set_Now_Omega_Angle(steering_wheel->Now_Omega); // 转向轮的当前数据不直接来自于电机

#ifdef DEBUG_DIR_SPEED
    steering_wheel->Directive_Motor.Set_Target_Omega_Angle(test_speed);

#endif 

    // PID计算
    steering_wheel->Motion_Motor.TIM_PID_PeriodElapsedCallback();
    steering_wheel->Directive_Motor.TIM_PID_PeriodElapsedCallback();

#if POWER_CONTROL == 1

#if defined(AGV_BOARD_A) || defined(AGV_BOARD_B) || defined(AGV_BOARD_C) || defined(AGV_BOARD_D)
    // 获取PID输出
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].pid_output =
        steering_wheel->Directive_Motor.Get_Out();
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].pid_output =
        steering_wheel->Motion_Motor.Get_Out();

    // 更新pid输出扭矩
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].torque =
        steering_wheel->Directive_Motor.Get_Out() * GET_CMD_CURRENT_TO_TORQUE(MOTOR_BASE_INDEX);
    steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].torque =
        steering_wheel->Motion_Motor.Get_Out() * GET_CMD_CURRENT_TO_TORQUE(MOTOR_BASE_INDEX + 1);

     // 设置输出
    if (steering_wheel->Get_Steering_Cmd_Status() == Steering_Cmd_Status_DISABLE) // 舵轮失能保护(即失能两个电机)
    {
        steering_wheel->Directive_Motor.Set_Out(0.0f);
        steering_wheel->Motion_Motor.Set_Out(0.0f);
    }
    else
    {
        // 运行功率限制任务
        // steering_wheel->Power_Limit.Power_Task(steering_wheel->Power_Management);
        // float omega[8]; 
        // float torque[8];
        // float power_limit = steering_wheel->Power_Management.Max_Power;
        // for(int i=0;i<8;i++){
        //     omega[i] = (float)steering_wheel->Power_Management.Motor_Data[i].feedback_omega/omega_k;
        //     torque[i] = steering_wheel->Power_Management.Motor_Data[i].torque;
        // }
        // limit_k = steering_wheel->Power_Limit.Calculate_Limit_K(omega,torque,power_limit,8);

        power_limit = steering_wheel->Power_Management.Max_Power;
        //收集功率
        for(int i=0;i<8;i++)
        {
            if(i % 2 == 0)
            {
                dir_omega[i/2] = (float)steering_wheel->Power_Management.Motor_Data[i].feedback_omega/omega_k;
                dir_torque[i/2] = steering_wheel->Power_Management.Motor_Data[i].torque;
            }
            else
            {
                mot_omega[i/2] = (float)steering_wheel->Power_Management.Motor_Data[i].feedback_omega/omega_k;
                mot_torque[i/2] = steering_wheel->Power_Management.Motor_Data[i].torque;
            }
        }
        
        //预测转向和动力功率
        dir_predict_power = steering_wheel->Power_Limit.Calculate_Theoretical_Power(dir_omega,dir_torque,4);
        mot_predict_power = steering_wheel->Power_Limit.Calculate_Theoretical_Power(mot_omega,mot_torque,4);
        // 分配转向和动力功率
        steering_wheel->Power_Limit.Power_Allocate(power_limit,0.8,dir_predict_power,mot_predict_power,&dir_allocate_power,&mot_allocate_power);
        // // 分别跑功率限制
        limit_dir_k = steering_wheel->Power_Limit.Calculate_Limit_K(dir_omega,dir_torque,dir_allocate_power,4);
        limit_mot_k = steering_wheel->Power_Limit.Calculate_Limit_K(mot_omega,mot_torque,mot_allocate_power,4);
        // // 设置输出
        steering_wheel->Directive_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX].pid_output*limit_dir_k);
        steering_wheel->Motion_Motor.Set_Out(steering_wheel->Power_Management.Motor_Data[MOTOR_BASE_INDEX + 1].pid_output*limit_mot_k);
    }
#endif



#endif 
    portEXIT_CRITICAL();
}
float k1, k2,forget_k;
uint8_t test_data[8];
uint8_t test_test_data[8];
void Command_Send(Class_Steering_Wheel *steering_wheel)
{
#ifdef AGV_BOARD_B
    /*测试用*/
    k1 = steering_wheel->Power_Limit.Get_K1();
    k2 = steering_wheel->Power_Limit.Get_K2();
    forget_k = steering_wheel->Power_Limit.Get_forget_k();
//    memcpy(test_data, &steering_wheel->Power_Management.Theoretical_Total_Power, sizeof(float));
    memcpy(test_data, &k1, sizeof(float));
    memcpy(test_data+4, &k2, sizeof(float));
    memcpy(test_test_data, &forget_k, sizeof(float));
    CAN_Send_Data(&hcan2, 0x20E, test_data, 8);
    CAN_Send_Data(&hcan2, 0x20F, test_test_data, 8);
#endif

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);                  // 发送本轮组电机指令
    static uint8_t mod5 = 0;
    mod5++;
    if(mod5 == 5)
    {
        mod5 = 0;
        CAN_Send_Data(&hcan2, BOARD_TO_BOARDS_ID_1, AGV_BOARD_CAN_DATA_1, 8); // 发送本轮组电机的转速和扭矩
        CAN_Send_Data(&hcan2, BOARD_TO_BOARDS_ID_2, AGV_BOARD_CAN_DATA_2, 8); // 发送本轮组电机的pid out 的值
    }
    steering_wheel->Encoder.Briter_Encoder_Request_Total_Angle();
    CAN_Send_Data(&hcan1, ENCODER_ID, ENCODER_CAN_DATA, 8); // 发送请求编码器数据
}

int steering_wheel_dt;
static int steering_wheel_start;
extern "C" void Steering_Wheel_Task(void *argument)
{

    while (1)
    {
        steering_wheel_start = DWT_GetTimeline_us();

        State_Update(&steering_wheel);
        Command_Update(&steering_wheel);
        Control_Update(&steering_wheel);
        Command_Send(&steering_wheel);
        steering_wheel_dt = DWT_GetTimeline_us() - steering_wheel_start;
        osDelay(1);
    }
}
