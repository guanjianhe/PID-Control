#include "pid.h"

#define ABS(tmp) ((tmp<0)?(-tmp):(tmp))

//增量式
static float PID_IncCalculate (struct tagPIDCB *cb, float samplepoint)
{
    cb->Prv.NowError = cb->InPara.TargetPoint - samplepoint;

    cb->Output.Out += (cb->InPara.Kp * (cb->Prv.NowError - cb->Prv.LastError)
                       + cb->InPara.Ki * cb->Prv.NowError
                       + cb->InPara.Kd * (cb->Prv.NowError - 2 * cb->Prv.LastError + cb->Prv.PrevError));

    cb->Prv.LastError = cb->Prv.NowError;
    cb->Prv.PrevError = cb->Prv.LastError;

    cb->Output.Out = (cb->Output.Out >= cb->InPara.OutputUpperLimit) ? (cb->InPara.OutputUpperLimit) : (cb->Output.Out);
    cb->Output.Out = (cb->Output.Out <= cb->InPara.OutputLowerLimit) ? (cb->InPara.OutputLowerLimit) : (cb->Output.Out);

    return cb->Output.Out;
}

//位置式
static float PID_PosCalculate (struct tagPIDCB *cb, float samplepoint)
{

    #if 0
    cb->Prv.NowError = cb->InPara.TargetPoint - samplepoint;

    cb->Prv.IntegralSum += cb->Prv.NowError;

    cb->Output.Out = cb->InPara.Kp * cb->Prv.NowError
                     + cb->InPara.Ki * cb->Prv.IntegralSum
                     + cb->InPara.Kd * (cb->Prv.NowError - cb->Prv.LastError);

    cb->Prv.LastError = cb->Prv.NowError;

    cb->Output.Out = (cb->Output.Out >= cb->InPara.OutputUpperLimit) ? (cb->InPara.OutputUpperLimit) : (cb->Output.Out);
    cb->Output.Out = (cb->Output.Out <= cb->InPara.OutputLowerLimit) ? (cb->InPara.OutputLowerLimit) : (cb->Output.Out);

    #else

    int index = 0;
    cb->Prv.NowError = cb->InPara.TargetPoint - samplepoint;

    if (cb->Output.Out >= cb->InPara.OutputUpperLimit)   //抗饱和积分分离
    {
        if (ABS (cb->Prv.NowError) < 200)
        {
            index = 1;

            if (cb->Prv.NowError < 0)
            {
                cb->Prv.IntegralSum += cb->Prv.NowError;
            }
        }
    }
    else if (cb->Output.Out <= cb->InPara.OutputLowerLimit)   //抗饱和积分分离
    {
        if (ABS (cb->Prv.NowError) < 200)
        {
            index = 1;

            if (cb->Prv.NowError > 0)
            {
                cb->Prv.IntegralSum += cb->Prv.NowError;
            }
        }
    }
    else
    {
        if (ABS (cb->Prv.NowError) < 200)     //积分分离
        {
            index = 1;
            cb->Prv.IntegralSum += cb->Prv.NowError;
        }
    }

    cb->Output.Out = cb->InPara.Kp * cb->Prv.NowError
                     + index * cb->InPara.Ki * cb->Prv.IntegralSum
                     + cb->InPara.Kd * (cb->Prv.NowError - cb->Prv.LastError);

    cb->Prv.LastError = cb->Prv.NowError;

    cb->Output.Out = (cb->Output.Out >= cb->InPara.OutputUpperLimit) ? (cb->InPara.OutputUpperLimit) : (cb->Output.Out);
    cb->Output.Out = (cb->Output.Out <= cb->InPara.OutputLowerLimit) ? (cb->InPara.OutputLowerLimit) : (cb->Output.Out);

    #endif

    return cb->Output.Out;
}

void PID_InitPara (struct tagPIDCB *cb,
                   enum PID_MODE mode,       //pid模式
                   float targetpoint,        //目标点
                   int16_t outputupperlimit, //输出上限
                   int16_t outputlowerlimit, //输出下限
                   float kp,
                   float ki,
                   float kd)
{
    cb->InPara.OutputUpperLimit = outputupperlimit;
    cb->InPara.OutputLowerLimit = outputlowerlimit;
    cb->InPara.TargetPoint = targetpoint;
    cb->InPara.Kp = kp;
    cb->InPara.Ki = ki;
    cb->InPara.Kd = kd;

    cb->Prv.NowError = 0.0f;
    cb->Prv.LastError = 0.0f;
    cb->Prv.PrevError = 0.0f;
    cb->Prv.IntegralSum = 0.0f;

    cb->Output.Out = 0.0f;

    if (PID_INC == mode)    //增量式
    {
        cb->InFunc.PID_Calc = PID_IncCalculate;
    }
    else if (PID_POS == mode)   //位置式
    {
        cb->InFunc.PID_Calc = PID_PosCalculate;
    }
    else
    {

    }
}

float PID_Calc (struct tagPIDCB *cb, float samplepoint)
{
    return (cb->InFunc.PID_Calc (cb, samplepoint));
}

void PID_SetTargetValue (struct tagPIDCB *cb, float targetpoint)
{
    cb->InPara.TargetPoint = targetpoint;
}

float PID_GetTargetValue (struct tagPIDCB *cb)
{
    return cb->InPara.TargetPoint;
}

float PID_GetOutput (struct tagPIDCB *cb)
{
    return cb->Output.Out;
}



void PID_SetPIDPara (struct tagPIDCB *cb,
                     float kp,
                     float ki,
                     float kd)
{
    cb->InPara.Kp = kp;
    cb->InPara.Ki = ki;
    cb->InPara.Kd = kd;
}

void PID_GetPIDPara (struct tagPIDCB *cb, float *pidpara)
{
    pidpara[0] = cb->InPara.Kp;
    pidpara[1] = cb->InPara.Ki;
    pidpara[2] = cb->InPara.Kd;
}
