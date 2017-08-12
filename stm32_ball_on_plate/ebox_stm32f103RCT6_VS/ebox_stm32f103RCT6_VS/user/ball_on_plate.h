#pragma once
#include "ebox.h"
#include "uart_num.h"
#include "my_math.h"
#include "signal_stream.h"
#include "PID.h"
#include "servo.h"
#include "ws2812.h"
#include <math.h>

//#define PIDBallOnPlate_DEBUG


typedef enum
{
	PIDBallOnPlate_Mode_Task,
	PIDBallOnPlate_Mode_Cali
}PIDBallOnPlate_Mode;

//Ϊľ��ƽ��С����Ƶ�����PID
//ǰ���������ٻ��ֲ���ȫ΢��PID
//������˲�������Ч��΢������PID
//������deadzone�У�����ٶ�С��vLim������I����
class PIDBallOnPlate :public PIDGshifIntIncDiff, public PIDDeadzone
{
	float vLim;
	PIDBallOnPlate_Mode mode;
public:
	PIDBallOnPlate(float kp = 0, float ki = 0, float kd = 0,
		float interval = 0.01, float stopFrq = 50, float deadzone = 0, float vLim = 0) :
		PIDPosition(kp, ki, kd, interval),
		PIDGshifIntIncDiff(kp, ki, kd, interval, stopFrq),
		PIDDeadzone(kp, ki, kd, interval, deadzone),
		vLim(vLim)
	{

	}

	virtual float refresh(float feedback)
	{
		float err;
		err = target - feedback;

		//��ʼʱʹ΢��Ϊ0������ͻȻ�ľ޴����΢��
		if (isBegin)
		{
			errOld = err;
			isBegin = false;
		}

		//PID�㷨��ʼ
			//���������deadzone�У����ٶ�С��vLim������I���ƣ�����P����
		if ((abs(err) < deadzone) && abs(err - errOld) < vLim)
		{
			output = 1.8*kp*err + integral + filter.getFilterOut(1.9*kd*(err - errOld));
		}
		//����ڣ�����PD����
		else if (abs(err) < 25)
		{
			//���������Χֹͣ���ּ�������
			//�������Ŀ����ֹͣ���ּ�������
			if (((output > outputLimL && output < outputLimH) ||
				(output == outputLimH && err < 0) ||
				(output == outputLimL && err > 0))
				&& err - errOld < 1)
			{
				float ek = (err + errOld) / 2;
				integral += 1.2*ki*fek(ek)*ek;
			}
			if (err > gearshiftPointL + gearshiftPointH)
			{
				integral = 0;
			}
			output = 2.0*kp*err + integral + filter.getFilterOut(1.9*kd*(err - errOld));
		}
		//����ڣ�����PD����
		else if (abs(err) < 50)
		{
			//���������Χֹͣ���ּ�������
			//�������Ŀ����ֹͣ���ּ�������
			if (((output > outputLimL && output < outputLimH) ||
				(output == outputLimH && err < 0) ||
				(output == outputLimL && err > 0))
				&& err - errOld < -3)
			{
				float ek = (err + errOld) / 2;
				integral += ki*fek(ek)*ek;
			}
			if (err > gearshiftPointL + gearshiftPointH)
			{
				integral = 0;
			}
			output = 1.5*kp*err + integral + filter.getFilterOut(1.9*kd*(err - errOld));
		}
		else
		{
			//���������Χֹͣ���ּ�������
			//�������Ŀ����ֹͣ���ּ�������
			if (((output > outputLimL && output < outputLimH) ||
				(output == outputLimH && err < 0) ||
				(output == outputLimL && err > 0))
				&& err - errOld < 0)
			{
				float ek = (err + errOld) / 2;
				integral += ki*fek(ek)*ek;
			}
			if (err > gearshiftPointL + gearshiftPointH)
			{
				integral = 0;
			}
			output = kp*err + integral + filter.getFilterOut(kd*(err - errOld));
		}

		errOld = err;

		limit<float>(output, outputLimL, outputLimH);
		return output;
	}

	float getIntegral()
	{
		return integral;
	}

	void setMode(PIDBallOnPlate_Mode mode)
	{
		this->mode = mode;

	}
};

//ƽ��С����࣬���Ϊ������
//�̶�io�����ã������led��
//��������PID���ƽӿڣ���ʹ�ܣ������ָ�ˮƽ��������Ŀ��ֵ��PID�����̶�
//�����������ݻ�ȡ�ӿڣ���ǰ���ꡢPID��ر��������ֵ��ˢ������
//���������жϣ�PID�����жϣ��ص������󶨽ӿ�
//��������״̬��ȡ�ӿڣ�С���Ƿ��ڰ��ϣ���ݮ���Ƿ��ڷ�����Ϣ
//������ݮ�ɻ��������ӿڣ��ػ���ֹͣ����
//���������þ�̬����
class BallOnPlateBase
{
protected:
	//��λ
	UartNum<float, 2> uartNum;
	static const float maxPos;
	float posX;
	float posY;
	bool isBallOnPlate;
	uint64_t lastPosReceiveTime;
	//PID
	static const float ratePID;
	static const float intervalPID;
	FpsCounter fpsPID;
	//ǰ��ϵͳ
	static const float gzDenominator;
	static const float feedforwardSysH[3];
	SysWithOnlyZero feedforwardSysX, feedforwardSysY;
	//pid����
	float targetXFiltered, targetYFiltered, targetXraw, targetYraw;
	static const float factorPID;
	PIDBallOnPlate pidX, pidY;
	RcFilter filterOutX, filterOutY, filterTargetX, filterTargetY;
	float outX, outY;
	//pid�ж��лص��ĺ���ָ��
	FunctionPointerArg1<void, void> afterPIDEvent;
	//����
	Servo servoX, servoY;
	//����
	WS2812 ws2812;
	//ƽ��͹��ˮƽУ׼
	float offsetX, offsetY, factorX, factorY;


protected:
	//�յ���λ������������PID����
	virtual void posReceiveEvent(UartNum<float, 2>* uartNum)
	{
		lastPosReceiveTime = millis();

		//�Զ�λPID��Ŀ����������˲�
		targetXFiltered = filterTargetX.getFilterOut(targetXraw);
		targetYFiltered = filterTargetY.getFilterOut(targetYraw);
		pidX.setTarget(targetXFiltered);
		pidY.setTarget(targetYFiltered);

		if (uartNum->getLength() == 2)
		{
			posX = uartNum->getNum()[0];
			posY = uartNum->getNum()[1];

			if (!isnan(posX) && !isnan(posY))
			{
				if (isBallOnPlate == false)
				{
					pidX.resetState();
					pidY.resetState();

					isBallOnPlate = true;
				}

				outX = 0, outY = 0;
				outX += pidX.refresh(posX);
				outY -= pidY.refresh(posY);

				//���ƽ���´��ĽǶ��ʵ�����ƫ��
				float increaseX = -(posX - 300)*factorX / 200;
				float increaseY;
				//���������������ϴ������
				if (posY > 300)
				{
					increaseY = (posY - 300)*factorY*0.6 / 200;
				}
				else
				{
					increaseY = (posY - 300)*factorY*1.2 / 200;
				}

				outX += increaseX + offsetX;
				outY += increaseY - offsetY;
				limit<float>(outX, -100.f, 100.f);
				limit<float>(outY, -100.f, 100.f);

				outX = filterOutX.getFilterOut(outX);
				outY = filterOutY.getFilterOut(outY);
			}
			else
			{
				outX = 0, outY = 0;
				outX += offsetX;
				outY += offsetY;
				isBallOnPlate = false;
			}
		}
		servoX.setPct(outX);
		servoY.setPct(outY);

		fpsPID.getFps();

		afterPIDEvent.call();
	}

public:
	BallOnPlateBase() :
		//��λ
		uartNum(&uart2),
		posX(0.0 / 0.0), posY(0.0 / 0.0),
		isBallOnPlate(true),
		lastPosReceiveTime(0),
		//ǰ��ϵͳ
		feedforwardSysX((float*)feedforwardSysH, 3), feedforwardSysY((float*)feedforwardSysH, 3),
		targetXFiltered(maxPos / 2), targetYFiltered(maxPos / 2), targetXraw(targetXFiltered), targetYraw(targetYFiltered),
		//pid����
		pidX(0.2f*factorPID, 0.3f*factorPID, 0.15f*factorPID, 1.f / ratePID, 4, 5, 2),//I=0.5, deadzone=6, vLim=2
		pidY(0.2f*factorPID, 0.3f*factorPID, 0.15f*factorPID, 1.f / ratePID, 4, 5, 2),
		filterOutX(ratePID, 12), filterOutY(ratePID, 12), filterTargetX(ratePID, 0.3), filterTargetY(ratePID, 0.3),
		servoX(&PB9, 210, 0.90, 2.00), servoY(&PB8, 210, 0.95, 2.05),
		//����
		ws2812(&PB0),
		//ƽ��У׼
		offsetX(0), offsetY(0), factorX(0), factorY(0)
	{

	}

	//����ƽ��У׼����
	void setCaliParams(float offsetX, float offsetY, float factorX, float factorY)
	{
		this->offsetX = offsetX;
		this->offsetY = offsetY;
		this->factorX = factorX;
		this->factorY = factorY;
	}

	//��ʼ��PID����������λ������
	virtual void begin()
	{
		//PID
		fpsPID.begin();
		pidX.setTarget(maxPos / 2);
		pidX.setOutputLim(-100, 100);
		pidX.setGearshiftPoint(10, 40);

		pidY.setTarget(maxPos / 2);
		pidY.setOutputLim(-100, 100);
		pidY.setGearshiftPoint(10, 40);

		//����
		servoX.begin();
		servoX.setPct(0);
		servoY.begin();
		servoY.setPct(0);

		//��λ
		uartNum.begin(9600);

		//����
		ws2812.begin();
		ws2812.setAllDataHSV(60, 0, 0.7);

		uartNum.attach(this, &BallOnPlateBase::posReceiveEvent);
	}

	//��ȡλ�õ����ֵ���������������εĳ���
	float getPosMax()
	{
		return maxPos;
	}

	//��ȡPIDˢ�¼��
	float getIntervalPID()
	{
		return intervalPID;
	}

	//����Ŀ��ֵ
	void setTarget(float x, float y)
	{
		targetXraw = x;
		targetYraw = y;
	}
	//��ȡ�˲���Ŀ��ֵ
	void getTargetFiltered(float *x, float *y)
	{
		*x = targetXFiltered;
		*y = targetYFiltered;
	}
	//��ȡԭʼĿ��ֵ
	void getTargetRaw(float *x, float *y)
	{
		*x = targetXraw;
		*y = targetYraw;
	}

	//����Ŀ��X
	void setTargetX(float x)
	{
		targetXraw = x;
	}
	//����Ŀ��Y
	void setTargetY(float y)
	{
		targetYraw = y;
	}
	//��ȡԭʼĿ��X
	float getTargetXRaw()
	{
		return targetXraw;
	}
	//��ȡԭʼĿ��Y
	float getTargetYRaw()
	{
		return targetYraw;
	}
	//��ȡ�˲���Ŀ��X
	float getTargetXFiltered()
	{
		return targetXFiltered;
	}
	//��ȡ�˲���Ŀ��Y
	float getTargetYFiltered()
	{
		return targetYFiltered;
	}

	//��ȡ��ǰ����
	void getPos(float* x, float *y)
	{
		*x = posX;
		*y = posY;
	}
	//��ȡ��ǰX����
	float getPosX()
	{
		return posX;
	}
	//��ȡ��ǰY����
	float getPosY()
	{
		return posY;
	}

	//��ȡ������
	void getOut(float* x, float* y)
	{
		*x = outX;
		*y = outY;
	}
	//��ȡ��ǰX����
	float getOutX()
	{
		return outX;
	}
	//��ȡ��ǰY����
	float getOutY()
	{
		return outY;
	}

	////��ȡǰ��������
	//void getFeedforward(float* x, float* y)
	//{
	//	*x = pidX.getFeedforward();
	//	*y = pidY.getFeedforward();
	//}
	////��ȡXǰ��������
	//float getFeedforwardX()
	//{
	//	return pidX.getFeedforward();
	//}
	////��ȡYǰ��������
	//float getFeedforwardY()
	//{
	//	return pidY.getFeedforward();
	//}

	//С���Ƿ��ڰ���
	bool getIsBallOn()
	{
		return isBallOnPlate;
	}

	//��ݮ���Ƿ��ڷ�����Ϣ
	bool getIsPosReceiving()
	{
		if (millis() - lastPosReceiveTime < 1000)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//��pid�ж��лص��ĺ���
	void attachAfterPIDEvent(void(*afterPIDEvent)(void))
	{
		this->afterPIDEvent.attach(afterPIDEvent);
	}

	//��pid�ж��лص��ĺ���
	template<typename T>
	void attachAfterPIDEvent(T *pObj, void (T::*afterPIDEvent)(void))
	{
		this->afterPIDEvent.attach(pObj, afterPIDEvent);
	}

	//��ȡˢ����
	float getFps()
	{
		return fpsPID.getOldFps();
	}

	//�ر���ݮ��
	void shutdownRasp()
	{
		uartNum.printf("s");
	}

};

//��λ
const float BallOnPlateBase::maxPos = 600;
//PID
const float BallOnPlateBase::ratePID = 32;
const float BallOnPlateBase::intervalPID = 1 / ratePID;
//ǰ��ϵͳ
const float BallOnPlateBase::gzDenominator =
0.6125* intervalPID *intervalPID / (4 / 3 * M_PI / 200);//����һ������ٷֱȵ�������ȵĵ�λϵ��
const float BallOnPlateBase::feedforwardSysH[3] =
{ 1 / gzDenominator,-2 / gzDenominator,1 / gzDenominator };
//pid����
const float BallOnPlateBase::factorPID = 3.2;


//·�����±�����
class BallOnPlatePathIndex
{
protected:
	int pathLength;//·���ĳ���
public:
	int pathPointIndex[128];//�洢·�����±�

	BallOnPlatePathIndex()
	{
	}

	//��ȡ��src��dstԲ��������·��
	//pathPointIndex����ָ��ָ��·���ı��
	//0~8����1~9��Բ��9~12����1~4�Ű�ȫ����
	//pathLength����·���ĳ���
	void computePathPoint(int src, int dst)
	{
		//Ϊ7���������
		if ((src == 12 && dst == 8) || (src == 3 && dst == 11))
		{
			pathPointIndex[0] = src;
			pathPointIndex[1] = dst;
			pathLength = 2;
			return;
		}

		pathPointIndex[0] = src;

		int srcX = src % 3, srcY = src / 3;
		int dstX = dst % 3, dstY = dst / 3;
		int disX = abs(srcX - dstX), disY = abs(srcY - dstY);

		//���src��dst���ڣ������Խ��ߣ�����ͬ
		if (disX <= 1 && disY <= 1)
		{
			pathPointIndex[1] = dst;
			pathLength = 2;
		}
		//��������������Ϊ2
		else if (disX == 2 && disY == 2)
		{
			//�����0��ʼ
			if (src == 0)
			{
				pathPointIndex[1] = 9;
				pathPointIndex[2] = 11;
				pathPointIndex[3] = 12;
			}
			//�����6��ʼ
			else if (src == 6)
			{
				pathPointIndex[1] = 11;
				pathPointIndex[2] = 12;
				pathPointIndex[3] = 10;
			}
			//�����8��ʼ
			else if (src == 8)
			{
				pathPointIndex[1] = 12;
				pathPointIndex[2] = 10;
				pathPointIndex[3] = 9;
			}
			//�����2��ʼ
			else if (src == 2)
			{
				pathPointIndex[1] = 10;
				pathPointIndex[2] = 9;
				pathPointIndex[3] = 11;
			}
			pathPointIndex[4] = dst;
			pathLength = 5;
		}
		//���ֻ��һ���������Ϊ2
		else
		{
			//���·������ˮƽ
			if (disY <= 1)
			{
				int safeY = (srcY + dstY) / 2;
				limit(safeY, 0, 1);
				//����������
				if (srcX < dstX)
				{
					pathPointIndex[1] = 9 + safeY * 2;
					pathPointIndex[2] = 9 + safeY * 2 + 1;
				}
				//����������
				else
				{
					pathPointIndex[1] = 9 + safeY * 2 + 1;
					pathPointIndex[2] = 9 + safeY * 2;
				}
			}
			//���·��������ֱ
			else
			{
				int safeX = (srcX + dstX) / 2;
				limit(safeX, 0, 1);
				//����������
				if (srcY < dstY)
				{
					pathPointIndex[1] = 9 + safeX;
					pathPointIndex[2] = 9 + safeX + 2;
				}
				//����������
				else
				{
					pathPointIndex[1] = 9 + safeX + 2;
					pathPointIndex[2] = 9 + safeX;
				}
			}
			pathPointIndex[3] = dst;
			pathLength = 4;
		}

	}


	//��ȡ·������
	int getPathLength()
	{
		return pathLength;
	}

	//��ȡ·�������
	int getSrc()
	{
		return pathPointIndex[0];
	}

	//��ȡ·�����յ�
	int getDst()
	{
		return pathPointIndex[pathLength - 1];
	}

};

//һά·��������
//���ɹ涨ʱ���ڴ�x��y������·��
class SpeedControl
{
	//����ʼĩ��ֵ
	void setBeginEnd(float x, float y)
	{
		this->x = x;
		this->y = y;
	}

	//����ʱ�������ٶ�
	void setSpeed(float v)
	{
		v = abs(v);
		setTime(abs(y - x) / v);
	}

	//����ʱ������·����ʱ��
	void setTime(float time)
	{
		stepAll = time / interval;
		stepNow = 0;
	}

protected:
	float x, y, interval;
	int stepAll, stepNow;

public:
	SpeedControl(float interval) :
		interval(interval),
		stepAll(0),
		stepNow(0)
	{

	}

	//����·�����յ�����ʱ��
	virtual void setPathTime(float x, float y, float time)
	{
		setBeginEnd(x, y);
		setTime(time);
	}

	//����·�����յ������ٶ�
	virtual void setPathSpeed(float x, float y, float speed)
	{
		setBeginEnd(x, y);
		setSpeed(speed);
	}

	//��ȡ·������һ��
	//����true��ʾ���·��
	virtual bool getNext(float *next)
	{
		bool isEnd = false;
		stepNow += 1;
		if (stepNow > stepAll)
		{
			stepNow = stepAll;
			isEnd = true;
			*next = y;
		}
		else
		{
			*next = x + (y - x)*stepNow / stepAll;
		}
		return isEnd;
	}

	//�Ƿ��յ�
	virtual bool isEnd()
	{
		if (stepNow >= stepAll)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//ֹͣ·�����ɣ�����
	void stop()
	{
		stepNow = stepAll;
	}
};

class SpeedControlSoft :public SpeedControl
{
protected:
	RcFilter filter;
public:
	SpeedControlSoft(float interval) :
		SpeedControl(interval),
		filter(1 / interval, 1)
	{

	}

	//����·�����յ�����ʱ��
	virtual void setPathTime(float x, float y, float time)
	{
		filter.setInit(x);
		SpeedControl::setPathTime(x, y, time);
	}

	//����·�����յ������ٶ�
	virtual void setPathSpeed(float x, float y, float speed)
	{
		filter.setInit(x);
		SpeedControl::setPathSpeed(x, y, speed);
	}

	//��ȡ·������һ��
	//����true��ʾ���·��
	virtual bool getNext(float *output)
	{
		bool isEnd = SpeedControl::getNext(output);
		*output = filter.getFilterOut(*output);
		return isEnd;
	}

	//�����˲�����ֹƵ��
	void setStopFre(float fre)
	{
		filter.setStopFrq(fre);
	}
};

//��������·����ȡ������
class BallOnPlatePath :protected BallOnPlatePathIndex
{
	//�л�����һ��·����
	//����true��ʾ����
	bool moveOn()
	{
		//�����򷵻�
		if (pathIndex >= getPathLength() - 1)
		{
			return true;
		}

		//��ȡʵ������ֵ
		float srcX, srcY, dstX, dstY;
		int srcInd = pathIndex, dstInd = pathIndex + 1;
		int srcPoint = pathPointIndex[srcInd]
			, dstPoint = pathPointIndex[dstInd];
		if (srcPoint < 9)
		{
			srcX = circleX[srcPoint]; srcY = circleY[srcPoint];
		}
		else
		{
			srcX = safeX[srcPoint % 9]; srcY = safeY[srcPoint % 9];
		}
		if (dstPoint < 9)
		{
			dstX = circleX[dstPoint]; dstY = circleY[dstPoint];
		}
		else
		{
			dstX = safeX[dstPoint % 9]; dstY = safeY[dstPoint % 9];
		}
		float disY = abs(dstY - srcY), disX = abs(dstX - srcX);
		float speedRatio = sqrt(disX*disX + disY*disY);
		float speedX = disX / speedRatio*speed;
		float speedY = disY / speedRatio*speed;
		generatorX.setPathSpeed(srcX, dstX, speedX);
		generatorY.setPathSpeed(srcY, dstY, speedY);

		pathIndex++;

		return false;
	}

protected:
	float safeX[4], safeY[4];//4����ȫԲ������
	SpeedControl generatorX, generatorY;//����xy����һάĿ������
	int pathIndex;//��־��ǰ��BallOnPlatePathIndex��·�����±꣬��Χ0 ~ getPathLength()-2
	float speed;

public:
	static const float circleX[9], circleY[9];//9��Բ������

	BallOnPlatePath(float interval) :
		BallOnPlatePathIndex(),
		generatorX(interval), generatorY(interval),
		pathIndex(0),
		speed(1)
	{
		//���㰲ȫԲ������
		for (int i = 0; i < 4; i++)
		{
			int leftTopIndex = i / 2 * 3 + i % 2;
			int rightTopIndex = leftTopIndex + 1;
			int leftBotomIndex = leftTopIndex + 3;
			int rightBotomIndex = leftTopIndex + 4;
			safeX[i] = (circleX[leftTopIndex] + circleX[rightTopIndex]
				+ circleX[leftBotomIndex] + circleX[rightBotomIndex]) / 4;
			safeY[i] = (circleY[leftTopIndex] + circleY[rightTopIndex]
				+ circleY[leftBotomIndex] + circleY[rightBotomIndex]) / 4;
		}
	}

	//����·��������յ���ٶ�
	virtual void setPathSpeed(int src, int dst, float speed)
	{
		this->speed = speed;
		generatorX.stop();
		generatorY.stop();
		pathIndex = 0;
		computePathPoint(src, dst);
	}

	//��ȡ��һ������꣬�����Ƿ����
	virtual bool getNext(float *x, float *y)
	{
		//���xy���򶼵����յ㣬������һ��·��
		if (generatorX.isEnd() && generatorY.isEnd())
		{
			if (moveOn())
			{
				//��ȡ�յ�ʵ������ֵ
				float dstX, dstY;
				getCircleXY(getDst(), &dstX, &dstY);

				*x = dstX;
				*y = dstY;
				return true;
			}
		}

		generatorX.getNext(x);
		generatorY.getNext(y);


		return false;
	}

	bool isEnd()
	{
		if (pathIndex >= getPathLength() - 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void stop()
	{
		pathIndex = getPathLength() - 1;
		generatorX.stop();
		generatorY.stop();
	}

	//��ȡС��Բ����
	void getCircleXY(int index, float *x, float *y)
	{
		if (index < 9)
		{
			*x = circleX[index];
			*y = circleY[index];
		}
		else
		{
			index -= 9;
			*x = safeX[index];
			*y = safeY[index];
		}
	}

};
class BallOnPlatePathSoft :public BallOnPlatePath
{
protected:
	RcFilter filterX, filterY;
public:
	BallOnPlatePathSoft(float interval, float stopFre = 0.5) :
		BallOnPlatePath(interval),
		filterX(1 / interval, stopFre), filterY(1 / interval, stopFre)
	{

	}

	//�����˲����Ľ�ֹƵ��
	void setStopFre(float stopFre)
	{
		filterX.setStopFrq(stopFre);
		filterY.setStopFrq(stopFre);
	}

	//����·��������յ���ٶ�
	virtual void setPathSpeed(int src, int dst, float speed)
	{
		BallOnPlatePath::setPathSpeed(src, dst, speed);

		//��ȡ���ʵ������ֵ
		float srcX, srcY;
		srcX = circleX[src]; srcY = circleY[src];

		//�����˲�����ʼֵ
		filterX.setInit(srcX);
		filterY.setInit(srcY);
	}

	//��ȡ��һ������꣬�����Ƿ����
	virtual bool getNext(float *x, float *y)
	{
		float xRaw, yRaw;
		bool isEnd = BallOnPlatePath::getNext(&xRaw, &yRaw);
		*x = filterX.getFilterOut(xRaw);
		*y = filterY.getFilterOut(yRaw);

		return isEnd;
	}
};

const float BallOnPlatePath::circleX[9] = {
	94.8847,298.129,498.131,
	93.1718,299.301,507.667,
	97.8807,300.759,504.591
};
const float BallOnPlatePath::circleY[9] = {
	106.717,98.0838,98.9344,
	303.629,301.297,298.753,
	504.300,505.724,495.129
};


typedef enum
{
	BallOnPlate_Pos_Mode_Circle,
	BallOnPlate_Pos_Mode_Round
}BallOnPlate_Pos_Mode;

class BallOnPlate :public BallOnPlateBase
{
protected:
	BallOnPlatePath path;
	int targetCircle;
	BallOnPlate_Pos_Mode mode;
	float theta;
	float roundR, fre;

	//PID�жϺ��������·��ˢ�¶�Ŀ��������
	virtual void posReceiveEvent(UartNum<float, 2>* uartNum)
	{
#ifndef PIDBallOnPlate_DEBUG
		if (getIsBallOn())
		{
#endif // PIDBallOnPlate_DEBUG
			refreshPath();
#ifndef PIDBallOnPlate_DEBUG
		}
#endif // PIDBallOnPlate_DEBUG

		BallOnPlateBase::posReceiveEvent(uartNum);
	}

public:
	BallOnPlate() :
		path(getIntervalPID()),
		targetCircle(4),
		mode(BallOnPlate_Pos_Mode_Circle),
		roundR(75), fre(0.2)
	{

	}

	//У׼ƽ�壬�ɹ�����true
	bool calibrate()
	{
		float offsetX = 0, offsetY = 0, factorX = 0, factorY = 0;
		setCaliParams(0, 0, 0, 0);
		pidX.setMode(PIDBallOnPlate_Mode_Cali);
		pidY.setMode(PIDBallOnPlate_Mode_Cali);

		const float maxTime = 10000;
		const int caliCircle[] = { 4,0,2,8,6 };
		float intergralX[5], intergralY[5];

		for (int i = 0; i < 5; i++)
		{
			//У׼i��
			setPath(caliCircle[i], 100);
			if (!isBallInCircleFor(caliCircle[i], 30, 1000, maxTime))
			{
				return false;
			}
			intergralX[i] = pidX.getIntegral();
			intergralY[i] = pidY.getIntegral();
		}

		//����offset
		for (int i = 0; i < 5; i++)
		{
			offsetX += intergralX[i];
			offsetY += intergralY[i];
		}
		offsetX /= 5;
		offsetY /= 5;

		//����factor
		for (int i = 1; i < 5; i++)
		{
			factorX += abs(intergralX[i]);
			factorY += abs(intergralY[i]);
		}
		factorX /= 4;
		factorY /= 4;


		setCaliParams(offsetX, offsetY, factorX, factorY);
		pidX.setMode(PIDBallOnPlate_Mode_Task);
		pidY.setMode(PIDBallOnPlate_Mode_Task);
	}

	//����С�����ģʽ
	//��Բ��ģʽ�£���theta=pi/4*5��ʼ������5��תnȦ����theta=pi/2*3
	void setMode(BallOnPlate_Pos_Mode mode)
	{
		this->mode = mode;
		if (mode == BallOnPlate_Pos_Mode_Circle)
		{
		}
		else if (mode == BallOnPlate_Pos_Mode_Round)
		{
			theta = M_PI / 4 * 6;
		}
	}

	//��ȡС��Բ����
	void getCircleXY(int index, float *x, float *y)
	{
		*x = path.circleX[index];
		*y = path.circleY[index];
	}

	//��ȡС��ԲX����
	float getCircleX(int index)
	{
		return path.circleX[index];
	}

	//��ȡС��ԲY����
	float getCircleY(int index)
	{
		return path.circleY[index];
	}


	//���Ƿ����Ŀ��Բ
	bool isBallInCircle(int index, float dis = 10)
	{
		if (abs(path.circleX[index] - getPosX()) < dis
			&& abs(path.circleY[index] - getPosY()) < dis)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//���Ƿ����Ŀ��Բtime ms����ʱʱ��maxTime
	bool isBallInCircleFor(int index, float dis = 10, float time = 500, float maxTime = 50000)
	{
		TicToc timerAll, timerIn;
		timerAll.tic();
		timerIn.tic();
		bool start = false;
		while (1)
		{
			if (start == false && isBallInCircle(index, dis))
			{
				start = true;
				timerIn.tic();
			}
			else if (start == true && !isBallInCircle(index, dis))
			{
				start = false;
			}
			else if (start == true && isBallInCircle(index, dis))
			{
				if (timerIn.toc() > 500)
				{
					return true;
				}
			}
			if (timerAll.toc() > maxTime)
			{
				return false;
			}
		}
	}

	//����Ŀ��Բ���ӵ�ǰĿ��Բ�ƽ�
	void setPath(int dst, float speed)
	{
		setPath(targetCircle, dst, speed);

	}

	//����·��
	void setPath(int src, int dst, float speed)
	{
		if (mode == BallOnPlate_Pos_Mode_Round)
		{
			setMode(BallOnPlate_Pos_Mode_Circle);
		}
		path.setPathSpeed(src, dst, speed);
		targetCircle = dst;

	}

	//����Բ���˶��뾶������
	void setRoundParams(float r, float fre)
	{
		this->roundR = r;
		this->fre = fre;
	}

	//ˢ��·��
	bool refreshPath()
	{
		if (mode == BallOnPlate_Pos_Mode_Circle)
		{
			float x, y;
			bool isEnd = path.getNext(&x, &y);
			setTarget(x, y);
			return isEnd;
		}
		else if (mode == BallOnPlate_Pos_Mode_Round)
		{
			theta += 2 * M_PI*getIntervalPID()*fre;//תȦƵ��
			float x = roundR*cos(theta) + getCircleX(4), y = -roundR*sin(theta) + getCircleY(4);
			setTarget(x, y);
			if (theta > 4 * 2 * M_PI + M_PI / 4 * 7)
			{
				setMode(BallOnPlate_Pos_Mode_Circle);
				path.setPathSpeed(12, 8, 50);
			}
			return false;
		}
	}

	//ֹͣ·��
	void stopPath()
	{
		path.stop();
	}

	//��ȡ��ǰĿ��Բ
	int getTargetCircle()
	{
		return targetCircle;
	}
};
