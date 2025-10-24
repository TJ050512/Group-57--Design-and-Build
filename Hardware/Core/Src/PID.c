#include "PID.h"

void PID_Update(PID_t *p)
{
	// ������һ�����
	p->Error1 = p->Error0;
	// ���㵱ǰ���
	p->Error0 = p->Target - p->Actual;
	
	// ������� - �Ľ���������ֱ���
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		// �����޷�����ֹ���ֱ���
		if (p->ErrorInt > 1000.0f) p->ErrorInt = 1000.0f;
		if (p->ErrorInt < -1000.0f) p->ErrorInt = -1000.0f;
	}
	
	// PID����
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	// ����޷�
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
} 