#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"
#include "main.h"

typedef __PACKED_STRUCT
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

typedef __PACKED_STRUCT
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num;       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

//卡尔曼滤波器
typedef struct 
{
    float X_k; //先验
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
    float A;
    float B[2];
}KFP;//Kalman Filter parameter




//快速开方
extern fp32 invSqrt(fp32 num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,fp32 num);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);

extern float KalmanFilter(KFP *kfp,float Zk , float current_1,float current_2);
extern void KalmanFilter_init(KFP *kfp,float A ,float B_1 ,float B_2,float LastP,float Q,float R);
extern float Fabs(float a);
extern fp32 Calc(double x);
extern fp32 sig(fp32 x);


//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

extern fp32 fast_atan2f(fp32 x, fp32 y);

#endif
