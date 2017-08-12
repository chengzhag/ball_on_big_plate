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

//为木板平板小球设计的死区PID
//前馈补偿变速积分不完全微分PID
//输入带滤波器，等效于微分先行PID
//在死区deadzone中，如果速度小于vLim，禁用I控制
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

		//初始时使微分为0，避免突然的巨大错误微分
		if (isBegin)
		{
			errOld = err;
			isBegin = false;
		}

		//PID算法开始
			//如果在死区deadzone中，且速度小于vLim，禁用I控制，增大P控制
		if ((abs(err) < deadzone) && abs(err - errOld) < vLim)
		{
			output = 1.8*kp*err + integral + filter.getFilterOut(1.9*kd*(err - errOld));
		}
		//如果在，增大PD控制
		else if (abs(err) < 25)
		{
			//超过输出范围停止积分继续增加
			//如果朝向目标则停止积分继续增加
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
		//如果在，增大PD控制
		else if (abs(err) < 50)
		{
			//超过输出范围停止积分继续增加
			//如果朝向目标则停止积分继续增加
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
			//超过输出范围停止积分继续增加
			//如果朝向目标则停止积分继续增加
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

//平板小球基类，板材为正方形
//固定io口配置：舵机、led灯
//包含基本PID控制接口：（使能）、（恢复水平）、设置目标值。PID参数固定
//包含基本数据获取接口：当前坐标、PID相关变量、输出值、刷新速率
//包含串口中断（PID处理中断）回调函数绑定接口
//包含基本状态获取接口：小球是否在板上，树莓派是否在发送信息
//包含树莓派基本操作接口：关机、停止进程
//基本参数用静态常量
class BallOnPlateBase
{
protected:
	//定位
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
	//前馈系统
	static const float gzDenominator;
	static const float feedforwardSysH[3];
	SysWithOnlyZero feedforwardSysX, feedforwardSysY;
	//pid参数
	float targetXFiltered, targetYFiltered, targetXraw, targetYraw;
	static const float factorPID;
	PIDBallOnPlate pidX, pidY;
	RcFilter filterOutX, filterOutY, filterTargetX, filterTargetY;
	float outX, outY;
	//pid中断中回调的函数指针
	FunctionPointerArg1<void, void> afterPIDEvent;
	//动力
	Servo servoX, servoY;
	//照明
	WS2812 ws2812;
	//平板凸起、水平校准
	float offsetX, offsetY, factorX, factorY;


protected:
	//收到定位坐标立即进行PID运算
	virtual void posReceiveEvent(UartNum<float, 2>* uartNum)
	{
		lastPosReceiveTime = millis();

		//对定位PID的目标坐标进行滤波
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

				//针对平板下垂的角度适当增加偏置
				float increaseX = -(posX - 300)*factorX / 200;
				float increaseY;
				//消除两边下塌差别较大的因素
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
		//定位
		uartNum(&uart2),
		posX(0.0 / 0.0), posY(0.0 / 0.0),
		isBallOnPlate(true),
		lastPosReceiveTime(0),
		//前馈系统
		feedforwardSysX((float*)feedforwardSysH, 3), feedforwardSysY((float*)feedforwardSysH, 3),
		targetXFiltered(maxPos / 2), targetYFiltered(maxPos / 2), targetXraw(targetXFiltered), targetYraw(targetYFiltered),
		//pid参数
		pidX(0.2f*factorPID, 0.3f*factorPID, 0.15f*factorPID, 1.f / ratePID, 4, 5, 2),//I=0.5, deadzone=6, vLim=2
		pidY(0.2f*factorPID, 0.3f*factorPID, 0.15f*factorPID, 1.f / ratePID, 4, 5, 2),
		filterOutX(ratePID, 12), filterOutY(ratePID, 12), filterTargetX(ratePID, 0.3), filterTargetY(ratePID, 0.3),
		servoX(&PB9, 210, 0.90, 2.00), servoY(&PB8, 210, 0.95, 2.05),
		//照明
		ws2812(&PB0),
		//平板校准
		offsetX(0), offsetY(0), factorX(0), factorY(0)
	{

	}

	//设置平板校准参数
	void setCaliParams(float offsetX, float offsetY, float factorX, float factorY)
	{
		this->offsetX = offsetX;
		this->offsetY = offsetY;
		this->factorX = factorX;
		this->factorY = factorY;
	}

	//初始化PID、动力、定位、照明
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

		//动力
		servoX.begin();
		servoX.setPct(0);
		servoY.begin();
		servoY.setPct(0);

		//定位
		uartNum.begin(9600);

		//照明
		ws2812.begin();
		ws2812.setAllDataHSV(60, 0, 0.7);

		uartNum.attach(this, &BallOnPlateBase::posReceiveEvent);
	}

	//获取位置的最大值（可视区域正方形的长宽）
	float getPosMax()
	{
		return maxPos;
	}

	//获取PID刷新间隔
	float getIntervalPID()
	{
		return intervalPID;
	}

	//设置目标值
	void setTarget(float x, float y)
	{
		targetXraw = x;
		targetYraw = y;
	}
	//获取滤波后目标值
	void getTargetFiltered(float *x, float *y)
	{
		*x = targetXFiltered;
		*y = targetYFiltered;
	}
	//获取原始目标值
	void getTargetRaw(float *x, float *y)
	{
		*x = targetXraw;
		*y = targetYraw;
	}

	//设置目标X
	void setTargetX(float x)
	{
		targetXraw = x;
	}
	//设置目标Y
	void setTargetY(float y)
	{
		targetYraw = y;
	}
	//获取原始目标X
	float getTargetXRaw()
	{
		return targetXraw;
	}
	//获取原始目标Y
	float getTargetYRaw()
	{
		return targetYraw;
	}
	//获取滤波后目标X
	float getTargetXFiltered()
	{
		return targetXFiltered;
	}
	//获取滤波后目标Y
	float getTargetYFiltered()
	{
		return targetYFiltered;
	}

	//获取当前坐标
	void getPos(float* x, float *y)
	{
		*x = posX;
		*y = posY;
	}
	//获取当前X坐标
	float getPosX()
	{
		return posX;
	}
	//获取当前Y坐标
	float getPosY()
	{
		return posY;
	}

	//获取舵机输出
	void getOut(float* x, float* y)
	{
		*x = outX;
		*y = outY;
	}
	//获取当前X坐标
	float getOutX()
	{
		return outX;
	}
	//获取当前Y坐标
	float getOutY()
	{
		return outY;
	}

	////获取前馈控制量
	//void getFeedforward(float* x, float* y)
	//{
	//	*x = pidX.getFeedforward();
	//	*y = pidY.getFeedforward();
	//}
	////获取X前馈控制量
	//float getFeedforwardX()
	//{
	//	return pidX.getFeedforward();
	//}
	////获取Y前馈控制量
	//float getFeedforwardY()
	//{
	//	return pidY.getFeedforward();
	//}

	//小球是否在板上
	bool getIsBallOn()
	{
		return isBallOnPlate;
	}

	//树莓派是否在发送信息
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

	//绑定pid中断中回调的函数
	void attachAfterPIDEvent(void(*afterPIDEvent)(void))
	{
		this->afterPIDEvent.attach(afterPIDEvent);
	}

	//绑定pid中断中回调的函数
	template<typename T>
	void attachAfterPIDEvent(T *pObj, void (T::*afterPIDEvent)(void))
	{
		this->afterPIDEvent.attach(pObj, afterPIDEvent);
	}

	//获取刷新率
	float getFps()
	{
		return fpsPID.getOldFps();
	}

	//关闭树莓派
	void shutdownRasp()
	{
		uartNum.printf("s");
	}

};

//定位
const float BallOnPlateBase::maxPos = 600;
//PID
const float BallOnPlateBase::ratePID = 32;
const float BallOnPlateBase::intervalPID = 1 / ratePID;
//前馈系统
const float BallOnPlateBase::gzDenominator =
0.6125* intervalPID *intervalPID / (4 / 3 * M_PI / 200);//除以一个舵机百分比到舵机弧度的单位系数
const float BallOnPlateBase::feedforwardSysH[3] =
{ 1 / gzDenominator,-2 / gzDenominator,1 / gzDenominator };
//pid参数
const float BallOnPlateBase::factorPID = 3.2;


//路径点下标生成
class BallOnPlatePathIndex
{
protected:
	int pathLength;//路径的长度
public:
	int pathPointIndex[128];//存储路径的下标

	BallOnPlatePathIndex()
	{
	}

	//获取从src到dst圆所经过的路径
	//pathPointIndex数组指针指向路径的编号
	//0~8代表1~9号圆，9~12代表1~4号安全区域
	//pathLength代表路径的长度
	void computePathPoint(int src, int dst)
	{
		//为7题设计特例
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

		//如果src和dst相邻（包括对角线）或相同
		if (disX <= 1 && disY <= 1)
		{
			pathPointIndex[1] = dst;
			pathLength = 2;
		}
		//如果两个方向距离为2
		else if (disX == 2 && disY == 2)
		{
			//如果从0开始
			if (src == 0)
			{
				pathPointIndex[1] = 9;
				pathPointIndex[2] = 11;
				pathPointIndex[3] = 12;
			}
			//如果从6开始
			else if (src == 6)
			{
				pathPointIndex[1] = 11;
				pathPointIndex[2] = 12;
				pathPointIndex[3] = 10;
			}
			//如果从8开始
			else if (src == 8)
			{
				pathPointIndex[1] = 12;
				pathPointIndex[2] = 10;
				pathPointIndex[3] = 9;
			}
			//如果从2开始
			else if (src == 2)
			{
				pathPointIndex[1] = 10;
				pathPointIndex[2] = 9;
				pathPointIndex[3] = 11;
			}
			pathPointIndex[4] = dst;
			pathLength = 5;
		}
		//如果只有一个方向距离为2
		else
		{
			//如果路径方向水平
			if (disY <= 1)
			{
				int safeY = (srcY + dstY) / 2;
				limit(safeY, 0, 1);
				//如果起点在左
				if (srcX < dstX)
				{
					pathPointIndex[1] = 9 + safeY * 2;
					pathPointIndex[2] = 9 + safeY * 2 + 1;
				}
				//如果起点在右
				else
				{
					pathPointIndex[1] = 9 + safeY * 2 + 1;
					pathPointIndex[2] = 9 + safeY * 2;
				}
			}
			//如果路径方向竖直
			else
			{
				int safeX = (srcX + dstX) / 2;
				limit(safeX, 0, 1);
				//如果起点在上
				if (srcY < dstY)
				{
					pathPointIndex[1] = 9 + safeX;
					pathPointIndex[2] = 9 + safeX + 2;
				}
				//如果起点在下
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


	//获取路径长度
	int getPathLength()
	{
		return pathLength;
	}

	//获取路径的起点
	int getSrc()
	{
		return pathPointIndex[0];
	}

	//获取路径的终点
	int getDst()
	{
		return pathPointIndex[pathLength - 1];
	}

};

//一维路径生成类
//生成规定时间内从x到y的匀速路径
class SpeedControl
{
	//设置始末数值
	void setBeginEnd(float x, float y)
	{
		this->x = x;
		this->y = y;
	}

	//设置时间间隔和速度
	void setSpeed(float v)
	{
		v = abs(v);
		setTime(abs(y - x) / v);
	}

	//设置时间间隔和路径总时间
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

	//设置路径的终点起点和时间
	virtual void setPathTime(float x, float y, float time)
	{
		setBeginEnd(x, y);
		setTime(time);
	}

	//设置路径的终点起点和速度
	virtual void setPathSpeed(float x, float y, float speed)
	{
		setBeginEnd(x, y);
		setSpeed(speed);
	}

	//获取路径的下一点
	//返回true表示完成路径
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

	//是否到终点
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

	//停止路径生成，重置
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

	//设置路径的终点起点和时间
	virtual void setPathTime(float x, float y, float time)
	{
		filter.setInit(x);
		SpeedControl::setPathTime(x, y, time);
	}

	//设置路径的终点起点和速度
	virtual void setPathSpeed(float x, float y, float speed)
	{
		filter.setInit(x);
		SpeedControl::setPathSpeed(x, y, speed);
	}

	//获取路径的下一点
	//返回true表示完成路径
	virtual bool getNext(float *output)
	{
		bool isEnd = SpeedControl::getNext(output);
		*output = filter.getFilterOut(*output);
		return isEnd;
	}

	//设置滤波器截止频率
	void setStopFre(float fre)
	{
		filter.setStopFrq(fre);
	}
};

//任意两点路径获取与生成
class BallOnPlatePath :protected BallOnPlatePathIndex
{
	//切换到下一对路径点
	//返回true表示结束
	bool moveOn()
	{
		//结束则返回
		if (pathIndex >= getPathLength() - 1)
		{
			return true;
		}

		//获取实际坐标值
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
	float safeX[4], safeY[4];//4个安全圆的坐标
	SpeedControl generatorX, generatorY;//生成xy方向一维目标坐标
	int pathIndex;//标志当前在BallOnPlatePathIndex中路径的下标，范围0 ~ getPathLength()-2
	float speed;

public:
	static const float circleX[9], circleY[9];//9个圆的坐标

	BallOnPlatePath(float interval) :
		BallOnPlatePathIndex(),
		generatorX(interval), generatorY(interval),
		pathIndex(0),
		speed(1)
	{
		//计算安全圆的坐标
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

	//设置路径的起点终点和速度
	virtual void setPathSpeed(int src, int dst, float speed)
	{
		this->speed = speed;
		generatorX.stop();
		generatorY.stop();
		pathIndex = 0;
		computePathPoint(src, dst);
	}

	//获取下一点的坐标，返回是否完成
	virtual bool getNext(float *x, float *y)
	{
		//如果xy方向都到达终点，继续下一条路径
		if (generatorX.isEnd() && generatorY.isEnd())
		{
			if (moveOn())
			{
				//获取终点实际坐标值
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

	//获取小球圆坐标
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

	//设置滤波器的截止频率
	void setStopFre(float stopFre)
	{
		filterX.setStopFrq(stopFre);
		filterY.setStopFrq(stopFre);
	}

	//设置路径的起点终点和速度
	virtual void setPathSpeed(int src, int dst, float speed)
	{
		BallOnPlatePath::setPathSpeed(src, dst, speed);

		//获取起点实际坐标值
		float srcX, srcY;
		srcX = circleX[src]; srcY = circleY[src];

		//设置滤波器初始值
		filterX.setInit(srcX);
		filterY.setInit(srcY);
	}

	//获取下一点的坐标，返回是否完成
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

	//PID中断函数，添加路径刷新对目标点的设置
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

	//校准平板，成功返回true
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
			//校准i点
			setPath(caliCircle[i], 100);
			if (!isBallInCircleFor(caliCircle[i], 30, 1000, maxTime))
			{
				return false;
			}
			intergralX[i] = pidX.getIntegral();
			intergralY[i] = pidY.getIntegral();
		}

		//计算offset
		for (int i = 0; i < 5; i++)
		{
			offsetX += intergralX[i];
			offsetY += intergralY[i];
		}
		offsetX /= 5;
		offsetY /= 5;

		//计算factor
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

	//设置小球控制模式
	//在圆周模式下，从theta=pi/4*5开始绕区域5旋转n圈，至theta=pi/2*3
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

	//获取小球圆坐标
	void getCircleXY(int index, float *x, float *y)
	{
		*x = path.circleX[index];
		*y = path.circleY[index];
	}

	//获取小球圆X坐标
	float getCircleX(int index)
	{
		return path.circleX[index];
	}

	//获取小球圆Y坐标
	float getCircleY(int index)
	{
		return path.circleY[index];
	}


	//球是否进入目标圆
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

	//球是否进入目标圆time ms，超时时间maxTime
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

	//设置目标圆，从当前目标圆逼近
	void setPath(int dst, float speed)
	{
		setPath(targetCircle, dst, speed);

	}

	//设置路径
	void setPath(int src, int dst, float speed)
	{
		if (mode == BallOnPlate_Pos_Mode_Round)
		{
			setMode(BallOnPlate_Pos_Mode_Circle);
		}
		path.setPathSpeed(src, dst, speed);
		targetCircle = dst;

	}

	//设置圆周运动半径和速率
	void setRoundParams(float r, float fre)
	{
		this->roundR = r;
		this->fre = fre;
	}

	//刷新路径
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
			theta += 2 * M_PI*getIntervalPID()*fre;//转圈频率
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

	//停止路径
	void stopPath()
	{
		path.stop();
	}

	//获取当前目标圆
	int getTargetCircle()
	{
		return targetCircle;
	}
};
