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

BallOnPlateBase ballOnplate;
BallOnPlatePath path;

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

	//测试路径规划类
	for (int src = 0; src < 9; src++)
	{
		for (int dst = 0; dst < 9; dst++)
		{
			uart1.printf("%d到%d：\t", src, dst);
			path.computePathPoint(src, dst);
			showPathIndex(path.getPathPointIndex(), path.getPathLength());
		}
	}

	//测试数据
	//	0到0：	0	0
	//	0到1：	0	1
	//	0到2：	0	9	10	2
	//	0到3：	0	3
	//	0到4：	0	4
	//	0到5：	0	9	10	5
	//	0到6：	0	9	11	6
	//	0到7：	0	9	11	7
	//	0到8：	0	9	11	12	8
	//	1到0：	1	0
	//	1到1：	1	1
	//	1到2：	1	2
	//	1到3：	1	3
	//	1到4：	1	4
	//	1到5：	1	5
	//	1到6：	1	9	11	6
	//	1到7：	1	10	12	7
	//	1到8：	1	10	12	8
	//	2到0：	2	10	9	0
	//	2到1：	2	1
	//	2到2：	2	2
	//	2到3：	2	10	9	3
	//	2到4：	2	4
	//	2到5：	2	5
	//	2到6：	2	10	9	11	6
	//	2到7：	2	10	12	7
	//	2到8：	2	10	12	8
	//	3到0：	3	0
	//	3到1：	3	1
	//	3到2：	3	9	10	2
	//	3到3：	3	3
	//	3到4：	3	4
	//	3到5：	3	11	12	5
	//	3到6：	3	6
	//	3到7：	3	7
	//	3到8：	3	11	12	8
	//	4到0：	4	0
	//	4到1：	4	1
	//	4到2：	4	2
	//	4到3：	4	3
	//	4到4：	4	4
	//	4到5：	4	5
	//	4到6：	4	6
	//	4到7：	4	7
	//	4到8：	4	8
	//	5到0：	5	10	9	0
	//	5到1：	5	1
	//	5到2：	5	2
	//	5到3：	5	12	11	3
	//	5到4：	5	4
	//	5到5：	5	5
	//	5到6：	5	12	11	6
	//	5到7：	5	7
	//	5到8：	5	8
	//	6到0：	6	11	9	0
	//	6到1：	6	11	9	1
	//	6到2：	6	11	12	10	2
	//	6到3：	6	3
	//	6到4：	6	4
	//	6到5：	6	11	12	5
	//	6到6：	6	6
	//	6到7：	6	7
	//	6到8：	6	11	12	8
	//	7到0：	7	11	9	0
	//	7到1：	7	12	10	1
	//	7到2：	7	12	10	2
	//	7到3：	7	3
	//	7到4：	7	4
	//	7到5：	7	5
	//	7到6：	7	6
	//	7到7：	7	7
	//	7到8：	7	8
	//	8到0：	8	12	10	9	0
	//	8到1：	8	12	10	1
	//	8到2：	8	12	10	2
	//	8到3：	8	12	11	3
	//	8到4：	8	4
	//	8到5：	8	5
	//	8到6：	8	12	11	6
	//	8到7：	8	7
	//	8到8：	8	8





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


