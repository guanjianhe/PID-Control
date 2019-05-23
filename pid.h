#ifndef PID_H
#define PID_H
#include "stdint.h"


enum PID_MODE
{
    PID_INC = 0,    //增量式
    PID_POS,        //位置式
};

struct tagPIDInPara
{
    int16_t OutputUpperLimit;   //输出上限
    int16_t OutputLowerLimit;   //输出下限
    float TargetPoint;
    float Kp;
    float Ki;
    float Kd;
};

struct tagPIDInFunc
{
    float (*PID_Calc) (struct tagPIDCB *cb, float samplepoint);
};

struct tagPIDOutput
{
    float Out;
};

struct tagPIDPrv
{
    float NowError;
    float LastError;
    float PrevError;
    float IntegralSum;
};


struct tagPIDCB
{
    struct tagPIDInPara InPara;
    struct tagPIDOutput Output;
    struct tagPIDPrv    Prv;
    struct tagPIDInFunc InFunc;
};


extern void PID_InitPara (struct tagPIDCB *cb,
                          enum PID_MODE mode,       //pid模式
                          float targetpoint,        //目标点
                          int16_t outputupperlimit, //输出上限
                          int16_t outputlowerlimit, //输出下限
                          float kp,
                          float ki,
                          float kd);

extern void PID_SetPIDPara (struct tagPIDCB *cb,
                            float kp,
                            float ki,
                            float kd);
extern void PID_GetPIDPara (struct tagPIDCB *cb, float *pidpara);

extern float PID_GetOutput (struct tagPIDCB *cb);
extern float PID_Calc (struct tagPIDCB *cb, float samplepoint);

extern float PID_GetTargetValue (struct tagPIDCB *cb);
extern void PID_SetTargetValue (struct tagPIDCB *cb, float targetpoint);

#endif
