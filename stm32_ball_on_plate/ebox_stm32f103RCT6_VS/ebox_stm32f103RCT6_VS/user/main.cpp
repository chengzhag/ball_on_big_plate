#include "ebox.h"
#include "ball_on_plate.h"

//调试
#include "led.h"
#include "uart_vcan.h"

#include "my_math.h"
#include "signal_stream.h"
#include <math.h>

//交互
#include "Button.h"
#include "oled_i2c.h"
//操作系统
#include "freertos.h"
#include "task.h"
#include "queue.h"


using namespace std;

BallOnPlate ballOnplate;

//调试
UartVscan uartVscan(&uart1);
FpsCounter fpsUI;
float fpsUItemp;
float outX, outY;

//交互
const float rateUI = 10;
const float intervalUI = 1 / rateUI;
const uint32_t uiRefreshDelay = ((1000 * intervalUI + 0.5) / portTICK_RATE_MS);
Button keyL(&PB4, 1);
Button keyR(&PB1, 1);
Button keyU(&PC5, 1);
Button keyD(&PC2, 1);
Led led(&PD2, 1);
OLEDI2C oled(&i2c1);


//收到定位坐标立即进行PID运算
//将输出存入outXY到舵机刷新程序输出
//UI交互
int index = 0;
float circleR = 0;
float theta = 0;
const float rateCircle = 0.2;
void posReceiveEvent()
{
	//按键响应
	keyL.loop();
	keyR.loop();
	keyU.loop();
	keyD.loop();

	if (keyR.click())
	{
		index++;
	}
	if (keyL.click())
	{
		index--;
	}
	limit<int>(index, 0, 2);

	//按键响应
	float increase = 0;
	if (keyU.click())
	{
		increase++;
	}
	if (keyD.click())
	{
		increase--;
	}
	if (keyU.pressed_for(200, 0))
	{
		increase += 2;
	}
	if (keyD.pressed_for(200, 0))
	{
		increase -= 2;
	}
	if (keyR.pressed_for(5000, 1))
	{
		ballOnplate.shutdownRasp();
	}

	float targetXraw = ballOnplate.getTargetXraw();
	float targetYraw = ballOnplate.getTargetYraw();
	switch (index)
	{
	case 0:
		targetXraw += increase;
		limit<float>(targetXraw, 30, ballOnplate.getPosMax() - 30);
		break;
	case 1:
		targetYraw += increase;
		limit<float>(targetYraw, 30, ballOnplate.getPosMax() - 30);
		break;
	case 2:
		circleR = circleR + increase;
		limit<float>(circleR, 0, (ballOnplate.getPosMax() - 100) / 2);
		theta += 2 * PI *ballOnplate.getIntervalPID() * rateCircle;//0.5圈一秒
		targetXraw = ballOnplate.getPosMax() / 2 + circleR*sin(theta);
		targetYraw = ballOnplate.getPosMax() / 2 + circleR*cos(theta);
		break;
	default:
		break;
	}

	ballOnplate.setTarget(targetXraw, targetYraw);

	//float vscan[] = { 
	//	ballOnplate.getPosX(),ballOnplate.getPosY()
	//	,ballOnplate.getOutX(),ballOnplate.getOutY()
	//	,ballOnplate.getFeedforwardX(),ballOnplate.getFeedforwardY()
	//	,ballOnplate.getTargetXFiltered(),ballOnplate.getTargetYFiltered()
	//};
	//uartVscan.sendOscilloscope(vscan, 8);
	outX = ballOnplate.getOutX();
	outY = ballOnplate.getOutY();
}

//UI交互
void uiRefresh(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		float fpsPIDtemp;
		//判断是否树莓派已关机
		if (!ballOnplate.getIsPosReceiving())
		{
			fpsPIDtemp = 0;
		}
		else
		{
			fpsPIDtemp = ballOnplate.getFps();
		}

		switch (index)
		{
		case 0:
			oled.printf(0, 0, 2, "*%.1f %.1f %.0f  ", ballOnplate.getTargetXraw(), ballOnplate.getTargetYraw(), circleR);
			break;
		case 1:
			oled.printf(0, 0, 2, "%.1f *%.1f %.0f  ", ballOnplate.getTargetXraw(), ballOnplate.getTargetYraw(), circleR);
			break;
		case 2:
			oled.printf(0, 0, 2, "%.1f %.1f *%.0f  ", ballOnplate.getTargetXraw(), ballOnplate.getTargetYraw(), circleR);
			break;
		default:
			break;
		}
		oled.printf(0, 2, 2, "%-8.1f%-8.1f", ballOnplate.getPosX(), ballOnplate.getPosY());
		fpsUItemp = fpsUI.getFps();
		oled.printf(0, 4, 2, "%-8.0f%-8.0f", fpsPIDtemp, fpsUItemp);


		vTaskDelayUntil(&xLastWakeTime, uiRefreshDelay);
	}

}

//测试路径的数组输出
void showPathIndex(int* pathPointIndex, int pathLength)
{
	for (int i = 0; i < pathLength; i++)
	{
		uart1.printf("%d\t", pathPointIndex[i]);
	}
	uart1.printf("\r\n");
}


void setup()
{
	ebox_init();
	ballOnplate.attachAfterPIDEvent(posReceiveEvent);
	ballOnplate.begin();

	//调试
	uart1.begin(115200);
	fpsUI.begin();


	//交互
	keyD.begin();
	keyL.begin();
	keyR.begin();
	keyU.begin();
	led.begin();
	oled.begin();

	set_systick_user_event_per_sec(configTICK_RATE_HZ);
	attach_systick_user_event(xPortSysTickHandler);
	xTaskCreate(uiRefresh, "uiRefresh", 512, NULL, 0, NULL);
	vTaskStartScheduler();
}


int main(void)
{
	setup();


	while (1)
	{

	}

}


