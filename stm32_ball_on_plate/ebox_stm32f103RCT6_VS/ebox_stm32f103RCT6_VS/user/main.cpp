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

	limit<int>(numIndex, 0, 6);

	//总共7个部分，0部分用作调试
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
					ballOnPlate.setPath(4, 4, 100);
				}
				else
				{
					ballOnPlate.setPath(4, 100);
				}
				stage = 1;
				break;
			case 1://任务1，2稳定5
				if (isnan(ballOnPlate.getPosX()))
				{
					ballOnPlate.setPath(1, 1, 100);
				}
				else
				{
					ballOnPlate.setPath(1, 100);
				}
				stage = 1;
				break;
			case 2://任务2，1~5停留2<15
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 4, 100);
					stage = 2;
				}
				break;
			case 3://任务3，1~4停留2~5停留2<20
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 3, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 4://任务4，1~9停留2<30
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 8, 100);
					stage = 2;
				}
				
				break;
			case 5://任务5，1~2~6~9停留2<40
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(0, 0, 100);
					}
					else
					{
						ballOnPlate.setPath(0, 100);
					}
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(0, 1, 100);
					timerTask.tic();
					stage = 2;
				}
				break;
			case 6://任务6，手动输入先后经过ABCD
				if (stage == 0)
				{
					if (isnan(ballOnPlate.getPosX()))
					{
						ballOnPlate.setPath(abcd[0], abcd[0], speedTask6);
					}
					else
					{
						ballOnPlate.setPath(abcd[0], speedTask6);
					}
					
					stage = 1;
				}
				else if (stage == 1)
				{
					timerUI.tic();
					ballOnPlate.setPath(abcd[0], abcd[1], speedTask6);
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
	case 6:
		if (increase != 0)
		{
			ballOnPlate.shutdownRasp();
		}
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
			ballOnPlate.setPath(3, 4, 100);
			stage = 3;
		}
		break;
	case 5:
		if (timerTask.toc() > 3000 && stage == 2)
		{
			ballOnPlate.setPath(1, 5, 100);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 4000 && stage == 3)
		{
			ballOnPlate.setPath(5, 8, 100);
			stage = 4;
		}
		break;
	case 6:
		if (timerTask.toc() > 5000 && stage == 2)
		{
			ballOnPlate.setPath(abcd[1], abcd[2], speedTask6);
			stage = 3;
			timerTask.tic();
		}
		else if (timerTask.toc() > 5000 && stage == 3)
		{
			ballOnPlate.setPath(abcd[2], abcd[3], speedTask6);
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

		//显示关机按键
		if (numIndex==6)
		{
			oled.printf(24, 4, Oledi2c_Font_6x8_Inv, "shutdown");
		}
		else
		{
			oled.printf(24, 4, Oledi2c_Font_6x8, "shutdown");
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

void setup()
{
	ebox_init();
	ballOnPlate.attachAfterPIDEvent(posReceiveEvent);
	ballOnPlate.begin();
	ballOnPlate.setPath(4, 4, 100);//归中


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


