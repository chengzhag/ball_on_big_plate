#pragma once
#include "ebox.h"
#include "uart_num.h"
#include "my_math.h"
#include "signal_stream.h"
#include "PID.h"
#include "servo.h"
#include "ws2812.h"
#include <math.h>

////为木板平板小球设计的死区PID
////在死区deadzone中，如果速度小于vLim，禁用I控制
//
////在目标速度小于vLim且误差小于errLim时，禁用I控制
////否则在误差绝对值减小时，禁用I控制
////否则在误差绝对值增大时，启用I控制
//class PIDBallOnPlate:public PIDFeedforward, public PIDIncompleteDiff
//{
//public:
//	PIDBallOnPlate
//};

////为木板平板小球设计的死区PID
////在死区deadzone中，如果速度小于vLim，禁用I控制
class PIDBallOnPlate :public PIDFeforGshifIntIncDiff, public PIDDeadzone
{
	float vLim;
public:
	PIDBallOnPlate(float kp = 0, float ki = 0, float kd = 0,
		float interval = 0.01, float stopFrq = 50, float deadzone = 0, float vLim = 0) :
		PIDPosition(kp, ki, kd, interval),
		PIDFeforGshifIntIncDiff(kp, ki, kd, interval, stopFrq),
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

		//如果在死区deadzone中，如果速度小于vLim，禁用I控制，减少D控制
		if ((abs(err) < deadzone) && abs(err - errOld) < vLim)
		{
			feedforward = feedforwardH.call(target);
			output = kp*err + integral + filter.getFilterOut(kd*0.5*(err - errOld))
				+ feedforward;//FunctionPointer未绑定时默认返回0
		}
		else
		{
			//超过输出范围停止积分继续增加
			if (((output > outputLimL && output < outputLimH) ||
				(output == outputLimH && err < 0) ||
				(output == outputLimL && err > 0))
				&& err - errOld < 0)
			{
				float ek = (err + errOld) / 2;
				integral += ki*fek(ek)*ek;
			}
			if (err>gearshiftPointL+ gearshiftPointH)
			{
				integral = 0;
			}
			feedforward = feedforwardH.call(target);
			output = kp*err + integral + filter.getFilterOut(kd*(err - errOld))
				+ feedforward;//FunctionPointer未绑定时默认返回0
		}
		limit<float>(output, outputLimL, outputLimH);

		errOld = err;
		return output;
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

				outX = filterOutX.getFilterOut(outX);
				outY = filterOutY.getFilterOut(outY);
			}
			else
			{
				outX = 0; outY = 0;
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
		posX(0.0/0.0), posY(0.0/0.0),
		isBallOnPlate(true),
		lastPosReceiveTime(0),
		//前馈系统
		feedforwardSysX((float*)feedforwardSysH, 3), feedforwardSysY((float*)feedforwardSysH, 3),
		targetXFiltered(maxPos / 2), targetYFiltered(maxPos / 2), targetXraw(targetXFiltered), targetYraw(targetYFiltered),
		//pid参数
		pidX(0.3f*factorPID, 0.5f*factorPID, 0.16f*factorPID, 1.f / ratePID, 5, 6, 2),
		pidY(0.3f*factorPID, 0.5f*factorPID, 0.16f*factorPID, 1.f / ratePID, 5, 6, 2),
		filterOutX(ratePID, 15), filterOutY(ratePID, 15), filterTargetX(ratePID, 0.5), filterTargetY(ratePID, 0.5),
		servoX(&PB9, 200, 0.75, 2.05), servoY(&PB8, 200, 0.85, 2.15),
		//照明
		ws2812(&PB0)
	{

	}

	//初始化PID、动力、定位、照明
	virtual void begin()
	{
		//PID
		fpsPID.begin();
		pidX.setTarget(maxPos / 2);
		pidX.setOutputLim(-100, 100);
		pidX.setGearshiftPoint(20, 100);
		pidX.attachFeedForwardH(&feedforwardSysX, &SysWithOnlyZero::getY);

		pidY.setTarget(maxPos / 2);
		pidY.setOutputLim(-100, 100);
		pidY.setGearshiftPoint(20, 100);
		pidY.attachFeedForwardH(&feedforwardSysY, &SysWithOnlyZero::getY);

		//动力
		servoX.begin();
		servoX.setPct(0);
		servoY.begin();
		servoY.setPct(0);

		//定位
		uartNum.begin(115200);

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
	float getTargetXraw()
	{
		return targetXraw;
	}
	//获取原始目标Y
	float getTargetYraw()
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

	//获取前馈控制量
	void getFeedforward(float* x, float* y)
	{
		*x = pidX.getFeedforward();
		*y = pidY.getFeedforward();
	}
	//获取X前馈控制量
	float getFeedforwardX()
	{
		return pidX.getFeedforward();
	}
	//获取Y前馈控制量
	float getFeedforwardY()
	{
		return pidY.getFeedforward();
	}

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
const float BallOnPlateBase::factorPID = 3.7;


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
		pathPointIndex[0] = src;
		
		int srcX = src % 3,srcY=src/3;
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
	SpeedControl(float interval):
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
		bool isEnd=false;
		stepNow += 1;
		if (stepNow > stepAll)
		{
			stepNow = stepAll;
			isEnd = true;
		}
		*next = x + (y - x)*stepNow / stepAll;
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
};

class SpeedControlSoft:public SpeedControl
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
	static const float circleX[9], circleY[9];//9个圆的坐标
	float safeX[4], safeY[4];//4个安全圆的坐标
	SpeedControl generatorX, generatorY;//生成xy方向一维目标坐标
	int pathIndex;//标志当前在BallOnPlatePathIndex中路径的下标，范围0 ~ getPathLength()-2
	float speed;
public:
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
	virtual void setPathTime(int src, int dst, float speed)
	{
		this->speed = speed;
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
				return true;
			}
		}
		generatorX.getNext(x);
		generatorY.getNext(y);
	}

	bool isEnd()
	{
		if (pathIndex >= getPathLength() - 1)
		{
			return true;
		}
		else {
			return false;
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
		filterX(1/ interval, stopFre), filterY(1 / interval, stopFre)
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
		BallOnPlatePath::setPathTime(src, dst, speed);
		
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
		if (isEnd==false)
		{
			*x = filterX.getFilterOut(xRaw);
			*y = filterY.getFilterOut(yRaw);
		}
		else
		{
			//获取终点实际坐标值
			float dstX, dstY;
			dstX = circleX[getDst()]; dstY = circleY[getDst()];

			*x = filterX.getFilterOut(dstX);
			*y = filterY.getFilterOut(dstY);
		}

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


class BallOnPlate:public BallOnPlateBase
{
protected:
	BallOnPlatePath path;
public:
	BallOnPlate() :
		path(getIntervalPID())
	{

	}


	//设置路径
	void startPath(int src, int dst, float speed)
	{
		path.setPathTime(src, dst, speed);
	}

	//刷新路径
	bool refreshPath()
	{
		float x, y;
		bool isEnd = path.getNext(&x, &y);
		setTarget(x, y);
		return isEnd;
	}

	//PID中断函数，添加路径刷新对目标点的设置
	virtual void posReceiveEvent(UartNum<float, 2>* uartNum)
	{
		if (!path.isEnd())
		{
			refreshPath();
		}
		BallOnPlateBase::posReceiveEvent(uartNum);
	}
};
