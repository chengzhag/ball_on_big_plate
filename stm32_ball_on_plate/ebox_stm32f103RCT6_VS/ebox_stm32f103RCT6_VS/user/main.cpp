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

BallOnPlate ballOnPlate;

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


//按键交互
int numIndex = 0;
int task = 0;
TicToc timerTask,timerUI;
int stage = 0;//任务的状态，0代表停止，1代表准备完毕

//路径参数
int abcd[4] = { 0 };
const float speedTask6=150;

void posReceiveEvent()
{
	//按键响应
	keyL.loop();
	keyR.loop();
	keyU.loop();
	keyD.loop();

	if (keyR.click())
	{
		numIndex++;
	}
	if (keyL.click())
	{
		numIndex--;
	}

	//总共7个部分，0部分用作调试
	limit<int>(numIndex, 0, 5);

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
		increase += 1;
	}
	if (keyD.pressed_for(200, 0))
	{
		increase -= 1;
	}
	if (keyR.pressed_for(5000, 1))
	{
		ballOnPlate.shutdownRasp();
	}


	switch (numIndex)
	{
	case 0:
		stage = 0;//停止进行中的任务
		task += increase;
		limit(task, 0, 7);
		break;
	case 1:
		//选择键按下
		if (increase != 0)
		{
			//开始任务
			switch (task)
			{
			case 0://任务0，归中
				if (isnan(ballOnPlate.getPosX()))//防止准备时速度太快冲出平板
				{
					ballOnPlate.startPath(4, 4, 100);
				}
				else
				{
					ballOnPlate.startPath(4, 100);
				}
				stage = 1;
				break;
			case 1://任务1，2稳定5
				if (isnan(ballOnPlate.getPosX()))
				{
					ballOnPlate.startPath(1, 1, 100);
				}
				else
				{
					ballOnPlate.startPath(1, 100);
				}
				stage = 1;
				break;
			case 2://任务2，1~5停留2<15
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.startPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.startPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.startPath(0, 4, 100);
					stage = 2;
				}
				break;
			case 3://任务3，1~4停留2~5停留2<20
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.startPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.startPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.startPath(0, 3, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 4://任务4，1~9停留2<30
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.startPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.startPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.startPath(0, 8, 100);
					stage = 2;
				}
				
				break;
			case 5://任务5，1~2~6~9停留2<40
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.startPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.startPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.startPath(0, 1, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 6://任务6，手动输入先后经过ABCD
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.startPath(abcd[0], abcd[0], speedTask6);
					}
					else
					{
						ballOnPlate.startPath(abcd[0], speedTask6);
					}
					
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.startPath(abcd[0], abcd[1], speedTask6);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 7://任务7，

				break;
			default:
				break;
			}
		}
		break;
	case 2://A
		stage = 0;
		abcd[0] += increase;
		limit(abcd[0], 0, 8);
		break;
	case 3://B
		stage = 0;
		abcd[1] += increase;
		limit(abcd[1], 0, 8);
		break;
	case 4://C
		stage = 0;
		abcd[2] += increase;
		limit(abcd[2], 0, 8);
		break;
	case 5://D
		stage = 0;
		abcd[3] += increase;
		limit(abcd[3], 0, 8);
		break;
	default:
		break;
	}

	switch (task)
	{
	case 1:
		if (stage == 1 && !isnan(ballOnPlate.getPosX()))
		{
			timerUI.tic();
			stage = 2;
		}
		else if (stage == 2 && isnan(ballOnPlate.getPosX()))
		{
			stage = 1;
		}
		break;
	case 3:
		if (timerTask.toc() > 5000 && stage == 2)
		{
			ballOnPlate.startPath(3, 4, 100);
			stage = 3;
		}
		break;
	case 5:
		if (timerTask.toc() > 3000 && stage == 2)
		{
			ballOnPlate.startPath(1, 5, 100);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 4000 && stage == 3)
		{
			ballOnPlate.startPath(5, 8, 100);
			stage = 4;
		}
		break;
	case 6:
		if (timerTask.toc() > 5000 && stage == 2)
		{
			ballOnPlate.startPath(abcd[1], abcd[2], speedTask6);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 5000 && stage == 3)
		{
			ballOnPlate.startPath(abcd[2], abcd[3], speedTask6);
			stage = 4;
		}
		break;
	default:
		break;
	}

	
	float vscan[] = {
		ballOnPlate.getPosX(),ballOnPlate.getPosY()
		,ballOnPlate.getOutX(),ballOnPlate.getOutY()
		,ballOnPlate.getTargetXRaw(),ballOnPlate.getTargetYRaw()
		//,ballOnplate.getFeedforwardX(),ballOnplate.getFeedforwardY()
		,ballOnPlate.getTargetXFiltered(),ballOnPlate.getTargetYFiltered()
	};
	uartVscan.sendOscilloscope(vscan, 8);

	//调试
	outX = ballOnPlate.getOutX();
	outY = ballOnPlate.getOutY();
	//float point[3];
	//bool isEnd = path.getNext(point, point + 1);
	//if (isEnd)
	//{
	//	point[2] = 100;
	//}
	//uartVscan.sendOscilloscope(point, 3);
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
		if (!ballOnPlate.getIsPosReceiving())
		{
			fpsPIDtemp = 0;
		}
		else
		{
			fpsPIDtemp = ballOnPlate.getFps();
		}

		//显示任务标号
		oled.printf(0, 0, Oledi2c_Font_8x16, "task:");
		if (numIndex != 0)
		{
			oled.printf(40, 0, Oledi2c_Font_8x16, "%d",task);
		}
		else
		{
			oled.printf(40, 0, Oledi2c_Font_8x16_Inv, "%d", task);
		}

		//显示开始按键
		if (numIndex != 1)
		{
			if (stage == 0)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "start   ");
			}
			else if (stage == 1)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "ready   ");
			}
			else if (stage >= 2)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16, "working ");
			}
			
		}
		else
		{
			if (stage == 0)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "start");
			}
			else if (stage == 1)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "ready");
			}
			else if (stage >= 2)
			{
				oled.printf(64, 0, Oledi2c_Font_8x16_Inv, "working");
			}
		}

		//显示4位数
		for (int i = 0; i < 4; i++)
		{
			if (numIndex - 2 == i)
			{
				oled.printf(i * 8, 2, Oledi2c_Font_8x16_Inv, "%1d",abcd[i]+1);
			}
			else
			{
				oled.printf(i * 8, 2, Oledi2c_Font_8x16, "%1d", abcd[i] + 1);
			}
			
		}
		
		//显示计时
		if (stage >= 2)
		{
			oled.printf(64, 2, Oledi2c_Font_8x16, "%-8.1f", (float)timerUI.toc()/1000);
		}
		oled.printf(0, 5, Oledi2c_Font_6x8, "%-8.1f%-8.1f", ballOnPlate.getTargetXRaw(), ballOnPlate.getTargetYRaw());
		oled.printf(0, 6, Oledi2c_Font_6x8, "%-8.1f%-8.1f", ballOnPlate.getPosX(), ballOnPlate.getPosY());
		fpsUItemp = fpsUI.getFps();
		oled.printf(0, 7, Oledi2c_Font_6x8, "%-8.0f%-8.0f", fpsPIDtemp, fpsUItemp);

		vTaskDelayUntil(&xLastWakeTime, uiRefreshDelay);
	}

}

//显示生成的路径
BallOnPlatePathIndex path;
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
	ballOnPlate.attachAfterPIDEvent(posReceiveEvent);
	ballOnPlate.begin();
	ballOnPlate.startPath(4, 4, 100);//归中


	//调试
	uart1.begin(115200);
	fpsUI.begin();

	for (int src = 0; src < 13; src++)
	{
		for (int dst = 0; dst < 13; dst++)
		{
			uart1.printf("%d到%d：     \t", src, dst);
			path.computePathPoint(src, dst);
			showPathIndex(path.pathPointIndex, path.getPathLength());
		}
	}
	//测试结果
	//	0到0：     	0	0
	//	0到1：     	0	1
	//	0到2：     	0	9	10	2
	//	0到3：     	0	3
	//	0到4：     	0	4
	//	0到5：     	0	9	10	5
	//	0到6：     	0	9	11	6
	//	0到7：     	0	9	11	7
	//	0到8：     	0	9	11	12	8
	//	0到9：     	0	9	11	12	8
	//	0到10：     	0	9	11	12	8
	//	0到11：     	0	9	11	12	8
	//	0到12：     	0	9	11	12	8
	//	1到0：     	1	0
	//	1到1：     	1	1
	//	1到2：     	1	2
	//	1到3：     	1	3
	//	1到4：     	1	4
	//	1到5：     	1	5
	//	1到6：     	1	9	11	6
	//	1到7：     	1	10	12	7
	//	1到8：     	1	10	12	8
	//	1到9：     	1	10	12	8
	//	1到10：     	1	10	12	8
	//	1到11：     	1	10	12	8
	//	1到12：     	1	10	12	8
	//	2到0：     	2	10	9	0
	//	2到1：     	2	1
	//	2到2：     	2	2
	//	2到3：     	2	10	9	3
	//	2到4：     	2	4
	//	2到5：     	2	5
	//	2到6：     	2	10	9	11	6
	//	2到7：     	2	10	12	7
	//	2到8：     	2	10	12	8
	//	2到9：     	2	10	12	8
	//	2到10：     	2	10	12	8
	//	2到11：     	2	10	12	8
	//	2到12：     	2	10	12	8
	//	3到0：     	3	0
	//	3到1：     	3	1
	//	3到2：     	3	9	10	2
	//	3到3：     	3	3
	//	3到4：     	3	4
	//	3到5：     	3	11	12	5
	//	3到6：     	3	6
	//	3到7：     	3	7
	//	3到8：     	3	11	12	8
	//	3到9：     	3	11	12	8
	//	3到10：     	3	11	12	8
	//	3到11：     	3	11	12	8
	//	3到12：     	3	11	12	8
	//	4到0：     	4	0
	//	4到1：     	4	1
	//	4到2：     	4	2
	//	4到3：     	4	3
	//	4到4：     	4	4
	//	4到5：     	4	5
	//	4到6：     	4	6
	//	4到7：     	4	7
	//	4到8：     	4	8
	//	4到9：     	4	8
	//	4到10：     	4	8
	//	4到11：     	4	8
	//	4到12：     	4	8
	//	5到0：     	5	10	9	0
	//	5到1：     	5	1
	//	5到2：     	5	2
	//	5到3：     	5	12	11	3
	//	5到4：     	5	4
	//	5到5：     	5	5
	//	5到6：     	5	12	11	6
	//	5到7：     	5	7
	//	5到8：     	5	8
	//	5到9：     	5	8
	//	5到10：     	5	8
	//	5到11：     	5	8
	//	5到12：     	5	8
	//	6到0：     	6	11	9	0
	//	6到1：     	6	11	9	1
	//	6到2：     	6	11	12	10	2
	//	6到3：     	6	3
	//	6到4：     	6	4
	//	6到5：     	6	11	12	5
	//	6到6：     	6	6
	//	6到7：     	6	7
	//	6到8：     	6	11	12	8
	//	6到9：     	6	11	12	8
	//	6到10：     	6	11	12	8
	//	6到11：     	6	11	12	8
	//	6到12：     	6	11	12	8
	//	7到0：     	7	11	9	0
	//	7到1：     	7	12	10	1
	//	7到2：     	7	12	10	2
	//	7到3：     	7	3
	//	7到4：     	7	4
	//	7到5：     	7	5
	//	7到6：     	7	6
	//	7到7：     	7	7
	//	7到8：     	7	8
	//	7到9：     	7	8
	//	7到10：     	7	8
	//	7到11：     	7	8
	//	7到12：     	7	8
	//	8到0：     	8	12	10	9	0
	//	8到1：     	8	12	10	1
	//	8到2：     	8	12	10	2
	//	8到3：     	8	12	11	3
	//	8到4：     	8	4
	//	8到5：     	8	5
	//	8到6：     	8	12	11	6
	//	8到7：     	8	7
	//	8到8：     	8	8
	//	8到9：     	8	8
	//	8到10：     	8	8
	//	8到11：     	8	8
	//	8到12：     	8	8
	//	9到0：     	9	8
	//	9到1：     	9	8
	//	9到2：     	9	8
	//	9到3：     	9	8
	//	9到4：     	9	8
	//	9到5：     	9	8
	//	9到6：     	9	8
	//	9到7：     	9	8
	//	9到8：     	9	8
	//	9到9：     	9	9
	//	9到10：     	9	10
	//	9到11：     	9	11
	//	9到12：     	9	11
	//	10到0：     	10	11
	//	10到1：     	10	11
	//	10到2：     	10	11
	//	10到3：     	10	11
	//	10到4：     	10	11
	//	10到5：     	10	11
	//	10到6：     	10	11
	//	10到7：     	10	11
	//	10到8：     	10	11
	//	10到9：     	10	9
	//	10到10：     	10	10
	//	10到11：     	10	12
	//	10到12：     	10	12
	//	11到0：     	11	12
	//	11到1：     	11	12
	//	11到2：     	11	12
	//	11到3：     	11	12
	//	11到4：     	11	12
	//	11到5：     	11	12
	//	11到6：     	11	12
	//	11到7：     	11	12
	//	11到8：     	11	12
	//	11到9：     	11	9
	//	11到10：     	11	10
	//	11到11：     	11	11
	//	11到12：     	11	12
	//	12到0：     	12	12
	//	12到1：     	12	12
	//	12到2：     	12	12
	//	12到3：     	12	12
	//	12到4：     	12	12
	//	12到5：     	12	12
	//	12到6：     	12	12
	//	12到7：     	12	12
	//	12到8：     	12	12
	//	12到9：     	12	9
	//	12到10：     	12	10
	//	12到11：     	12	11
	//	12到12：     	12	12


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


