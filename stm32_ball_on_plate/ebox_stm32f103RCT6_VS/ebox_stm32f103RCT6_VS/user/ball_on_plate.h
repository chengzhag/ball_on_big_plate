#pragma once
#include "ebox.h"
#include "uart_num.h"
#include "my_math.h"
#include "signal_stream.h"
#include "PID.h"
#include "servo.h"
#include "ws2812.h"

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

	//收到定位坐标立即进行PID运算
	void posReceiveEvent(UartNum<float, 2>* uartNum)
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
		posX(-1), posY(-1),
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
	void begin()
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
