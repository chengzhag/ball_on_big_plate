#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "ebox.h"
#include <limits>
#include <math.h>
#define M_PI		3.14159265358979323846
//#include <limits.h>
//#include <float.h>
 
//����һ�����������е�ĳλ������
//dst: Ŀ���������
//bits: ����ֵ���Ҷ���
//high: ���������λ
//low: ���������λ
template<typename T>
void replaceBits(T& dst, T bits, uint8_t high, uint8_t low)
{
	T mask = 1;//����high=4,low=2,bits='b0000_0011,dst='b1011_0100,mask='b0000_0001
	mask <<= (high - low + 1);//mask='b0000_1000
	mask -= 1;//mask='b0000_0111
	mask <<= low;//mask='b0001_1100
	bits <<= low;//bits='b0000_1100
	bits &= mask;//ȷ��bits������λΪ0
	mask = ~mask;//mask='b1110_0011
	dst &= mask;//���dst��Ӧλ��dst='b1010_0000
	dst |= bits;//����dst�Ķ�Ӧλ��dst='b1010_1100
}

//����ĳ�������½�
template<typename T>
void limitLow(T &num, T limL)
{
	if (num < limL)
	{
		num = limL;
	}
}

//����ĳ�������Ͻ�
template<typename T>
void limitHigh(T &num, T limH)
{
	if (num > limH)
	{
		num = limH;
	}
}


//����ĳ���������½�
template<typename T>
void limit(T &num, T limL, T limH)
{
	if (num > limH)
	{
		num = limH;
	}
	else if (num < limL)
	{
		num = limL;
	}
}

//��������ģ��matlab��tictoc�࣬��λms
class TicToc
{
	unsigned long ticTime;
public:
	TicToc();
	//��ʼ��ʱ
	void tic();

	//���ش��ϴο�ʼ��ʱ�����ڵ�ʱ���
	unsigned long toc();

};


//֡�ʼ�����
class FpsCounter:private TicToc
{
	float fps;
public:
	FpsCounter();

	//��ʼ��ʱ
	void begin();

	//����֡��
	float getFps();

	//��ȡ�ϴ�֡��
	float getOldFps();
};








#endif