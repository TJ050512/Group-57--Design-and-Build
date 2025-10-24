#include "PID.h"

void PID_Update(PID_t *p)
{
	// 保存上一次误差
	p->Error1 = p->Error0;
	// 计算当前误差
	p->Error0 = p->Target - p->Actual;
	
	// 积分项处理 - 改进：避免积分饱和
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		// 积分限幅，防止积分饱和
		if (p->ErrorInt > 1000.0f) p->ErrorInt = 1000.0f;
		if (p->ErrorInt < -1000.0f) p->ErrorInt = -1000.0f;
	}
	
	// PID计算
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	// 输出限幅
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
} 